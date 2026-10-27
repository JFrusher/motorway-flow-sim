"""Scripted lorry-on-lorry overtake (an "elephant race") and its traffic impact.

Two speed-limited lorries share lane 1. At a set time the faster one pulls
into lane 2 and crawls past at the small speed difference between their
limiters, holding lanes 1 and 2 at lorry speed until it can pull back in.
Comparing against a run with the same seed and no overtake isolates the cost.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np
import pandas as pd

from .config import LORRY_LENGTH, LORRY_PROFILE, SimConfig
from .simulation import Simulation

MPH = 0.44704  # m/s
HARD_BRAKE = 3.0  # m/s^2, deceleration counted as a hard-braking event


@dataclass
class ElephantRace:
    """Controller for the scripted overtake. Call :meth:`place` once, then :meth:`update`
    before every simulation step."""

    fast_mph: float = 56.0
    slow_mph: float = 55.0
    start_x: float = 600.0  # front of the slow lorry at t = 0
    overtake_at: float = 150.0  # s, earliest time the fast lorry pulls out
    return_gap: float = 15.0  # m clear ahead of the slow lorry before pulling back in
    overtake: bool = True  # False = baseline: the fast lorry stays behind
    state: str = field(default="following", init=False)  # following -> overtaking -> done
    t_out: float | None = field(default=None, init=False)
    t_back: float | None = field(default=None, init=False)

    def place(self, sim: Simulation) -> None:
        """Put both lorries in lane 1, the fast one at equilibrium distance behind."""
        slow_p = replace(LORRY_PROFILE, v0=self.slow_mph * MPH)
        fast_p = replace(LORRY_PROFILE, v0=self.fast_mph * MPH)
        v = slow_p.v0
        self.slow = sim.add_vehicle(0, self.start_x, v, slow_p, is_lorry=True)
        gap = fast_p.s0 + v * fast_p.tau
        self.fast = sim.add_vehicle(0, self.start_x - LORRY_LENGTH - gap, v, fast_p, is_lorry=True)

    @property
    def lorry_ids(self) -> list[int]:
        return [self.slow.id, self.fast.id]

    def update(self, sim: Simulation) -> None:
        """Advance the overtake state machine (pull out, pass, pull back in)."""
        if not self.overtake or self.state == "done":
            return
        if max(self.fast.x, self.slow.x) > sim.road.length:
            self.state = "done"  # a lorry left the road; the overtake ends unfinished
            return
        if self.state == "following" and sim.t >= self.overtake_at:
            if sim.force_lane_change(self.fast, self.fast.lane + 1):
                self.state, self.t_out = "overtaking", sim.t
        elif self.state == "overtaking" and self.fast.rear - self.slow.x > self.return_gap:
            if sim.force_lane_change(self.fast, 0):
                self.state, self.t_back = "done", sim.t

    def blocking_time(self, t_end: float) -> float:
        """Seconds the overtaking lorry spent in lane 2 (to ``t_end`` if it never returned)."""
        if self.t_out is None:
            return 0.0
        return (self.t_back if self.t_back is not None else t_end) - self.t_out


@dataclass
class RaceRun:
    """Everything recorded from one run."""

    cfg: SimConfig
    race: ElephantRace
    history: pd.DataFrame
    entry_queue: pd.DataFrame
    arrivals: int


def run_race(cfg: SimConfig, race: ElephantRace) -> RaceRun:
    """Simulate ``cfg.duration`` seconds with the given race controller."""
    sim = Simulation(cfg)
    race.place(sim)
    for _ in range(round(cfg.duration / cfg.dt)):
        race.update(sim)
        sim.step()
    arrivals = sim.inserted - 2 + sum(sim.pending)
    return RaceRun(cfg, race, sim.history(), sim.entry_queue(), arrivals)


# ----------------------------------------------------------------- metrics
def lead_position(df: pd.DataFrame, lorry_ids: list[int]) -> pd.Series:
    """Front position of whichever lorry is further ahead, indexed by time.

    Only times when every lorry is still on the road are included, so the
    reference point does not jump when the first one leaves.
    """
    grouped = df[df.id.isin(lorry_ids)].groupby("t").x
    lead = grouped.max()
    return lead[grouped.size() == len(lorry_ids)]


def passing_flow(df: pd.DataFrame, lorry_ids: list[int], bin_s: float = 30.0) -> pd.Series:
    """Net rate (veh/h) at which cars overtake the leading lorry, per time bin.

    This is the throughput of the moving bottleneck formed by the lorries.
    """
    lead = lead_position(df, lorry_ids)
    cars = df[~df.id.isin(lorry_ids) & df.t.isin(lead.index)].sort_values(["id", "t"])
    rel = cars.x.to_numpy() - lead.reindex(cars.t).to_numpy()
    prev = pd.Series(rel).groupby(cars.id.to_numpy()).shift().to_numpy()
    crossing = np.where((prev < 0) & (rel >= 0), 1, np.where((prev >= 0) & (rel < 0), -1, 0))
    bins = (cars.t.to_numpy() // bin_s) * bin_s
    counts = pd.Series(crossing).groupby(bins).sum()
    return (counts * 3600 / bin_s).rename("veh_per_h")


def queue_behind(
    df: pd.DataFrame, lorry_ids: list[int], hold_speed: float, max_gap: float = 150.0
) -> pd.DataFrame:
    """Platoon held up behind the lorries at each time.

    Starting from the leading lorry, walk upstream through cars slower than
    ``hold_speed`` (any lane) while consecutive ones are within ``max_gap``.
    Returns columns ``t, length_m, vehicles``; ``.attrs["held_ids"]`` is the set
    of every car that was part of the platoon at some point.
    """
    lead = lead_position(df, lorry_ids)
    cars = df[~df.id.isin(lorry_ids) & (df.v < hold_speed)]
    by_t = dict(tuple(cars.groupby("t")))
    rows, held = [], set()
    for t, lead_x in lead.items():
        snap = by_t.get(t)
        snap = snap[snap.x < lead_x].sort_values("x", ascending=False) if snap is not None else []
        xs = snap.x.to_numpy() if len(snap) else np.empty(0)
        gaps = -np.diff(np.r_[lead_x, xs])
        n = int(np.argmax(gaps > max_gap)) if np.any(gaps > max_gap) else len(xs)
        if n:
            held.update(snap.id.to_numpy()[:n].tolist())
        rows.append((t, lead_x - xs[n - 1] if n else 0.0, n))
    out = pd.DataFrame(rows, columns=["t", "length_m", "vehicles"])
    out.attrs["held_ids"] = held
    return out


def overtake_window(run: RaceRun) -> tuple[float, float]:
    """``(start, end)`` of the overtake: pull-out to pull-in, or to the end of the run."""
    t_end = float(run.history.t.max())
    start = run.race.t_out if run.race.t_out is not None else run.race.overtake_at
    return start, run.race.t_back if run.race.t_back is not None else t_end


def run_metrics(run: RaceRun, window: tuple[float, float] | None = None) -> dict[str, float]:
    """Headline statistics for one run. Window-based figures (queue growth, passing
    flow) use ``window``, defaulting to this run's own overtake window."""
    df, cfg, race = run.history, run.cfg, run.race
    dt_rec = cfg.record_interval
    w0, w1 = window or overtake_window(run)
    cars = df[~df.id.isin(race.lorry_ids)].sort_values(["id", "t"])
    by_id = cars.groupby("id")
    braking = cars.a < -HARD_BRAKE
    hard_brakes = int((braking & ~braking.groupby(cars.id).shift(fill_value=False)).sum())
    changed = by_id.lane.diff().fillna(0) != 0
    hold = race.fast_mph * MPH + 1.0
    queue = queue_behind(df, race.lorry_ids, hold)
    t_end = float(df.t.max())
    growth = queue[(queue.t >= w0) & (queue.t <= w1)]
    if len(growth):  # growth phase: window start until the queue peaks
        growth = growth[growth.t <= growth.t[growth.length_m.idxmax()]]
    flow = passing_flow(df, race.lorry_ids)
    in_overtake = flow[(flow.index >= w0) & (flow.index < w1)]
    return {
        "total_time_spent_veh_h": (len(df) + run.entry_queue.queued.sum()) * dt_rec / 3600,
        "arrivals": run.arrivals,
        "mean_car_speed_kmh": float(cars.v.mean() * 3.6),
        "hard_brake_events": hard_brakes,
        "lane_changes": int(changed.sum()),
        "moves_into_fast_lane": int((changed & (cars.lane == cfg.num_lanes - 1)).sum()),
        "max_queue_m": float(queue.length_m.max()),
        "max_queue_vehicles": int(queue.vehicles.max()),
        "stuck_vehicle_min": float(queue.vehicles.sum() * dt_rec / 60),
        "drivers_held_up": len(queue.attrs["held_ids"]),
        "queue_growth_kmh": (
            float(np.polyfit(growth.t, growth.length_m, 1)[0] * 3.6) if len(growth) > 5 else 0.0
        ),
        "passing_flow_during_overtake_veh_h": (
            float(in_overtake.mean()) if len(in_overtake) else float(flow.mean())
        ),
        "blocking_s": race.blocking_time(t_end),
        "overtake_completed": race.t_back is not None,
    }


def impact(baseline: RaceRun, overtake: RaceRun) -> dict:
    """Metrics of both runs plus the cost of the overtake relative to the baseline.

    Both runs are measured over the overtake run's window, so the baseline's
    passing flow and queue growth cover the same period.
    """
    window = overtake_window(overtake)
    b, o = run_metrics(baseline, window), run_metrics(overtake, window)
    extra_h = o["total_time_spent_veh_h"] - b["total_time_spent_veh_h"]
    return {
        "baseline": b,
        "overtake": o,
        "extra_vehicle_hours": extra_h,
        "extra_delay_per_driver_s": extra_h * 3600 / max(o["arrivals"], 1),
        "extra_drivers_held_up": o["drivers_held_up"] - b["drivers_held_up"],
        "extra_delay_per_held_driver_s": extra_h * 3600 / max(o["drivers_held_up"], 1),
        "extra_hard_brakes": o["hard_brake_events"] - b["hard_brake_events"],
        "extra_lane_changes": o["lane_changes"] - b["lane_changes"],
        "extra_stuck_vehicle_min": o["stuck_vehicle_min"] - b["stuck_vehicle_min"],
        "passing_flow_drop_pct": 100
        * (
            1
            - o["passing_flow_during_overtake_veh_h"]
            / max(b["passing_flow_during_overtake_veh_h"], 1e-9)
        ),
    }
