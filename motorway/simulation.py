"""Core time-stepping loop: IDM longitudinal update, MOBIL lane changes,
open-boundary inflow/outflow and trajectory recording."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pandas as pd

from .config import CAR_LENGTH, LORRY_LENGTH, LORRY_PROFILE, SimConfig, make_profile
from .road import Road
from .vehicle import (
    Vehicle,
    apply_safety_clamp,
    idm_acceleration,
    mobil_incentive,
    mobil_is_safe,
    wake_adjusted,
)

ADJACENT_WAKE_STRENGTH = 0.5
CREEP_SPEED = 1.0  # m/s
RECORD_COLUMNS = ["t", "id", "lane", "x", "v", "a", "is_lorry"]


class Simulation:
    """Microscopic motorway simulation with open boundaries.

    Vehicles enter at x = 0 (Poisson arrivals, queued if the entry is
    blocked) and leave once their front passes ``road_length``.
    """

    def __init__(self, config: SimConfig | None = None) -> None:
        self.cfg = config or SimConfig()
        cfg = self.cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.road = Road(cfg.road_length, cfg.num_lanes, cfg.speed_limit)
        self.t = 0.0
        self.step_count = 0
        self.inserted = 0
        self.exited = 0
        self.pending = [0] * cfg.num_lanes
        self._next_id = 0
        self._record_every = max(1, round(cfg.record_interval / cfg.dt))
        self._mobil_every = max(1, round(cfg.lane_change_interval / cfg.dt))
        self._records: list[tuple] = []
        self._yields: list[tuple[Vehicle, Vehicle]] = []  # (follower, merging vehicle)
        weights = np.array(list(cfg.profile_mix.values()), dtype=float)
        self._kinds = list(cfg.profile_mix)
        self._weights = weights / weights.sum()

        if cfg.bottleneck_lorry_x is not None:
            speed = cfg.bottleneck_lorry_speed
            profile = replace(LORRY_PROFILE, v0=speed) if speed > 0 else LORRY_PROFILE
            lorry = self._new_vehicle(
                cfg.bottleneck_lorry_lane,
                cfg.bottleneck_lorry_x,
                speed,
                LORRY_LENGTH,
                profile,
                is_lorry=True,
                fixed=speed == 0,
            )
            self.road.place_obstacle(lorry)
            self.inserted += 1
        self._record()

    # ------------------------------------------------------------------ setup
    def _new_vehicle(self, lane, x, v, length, profile, is_lorry=False, fixed=False) -> Vehicle:
        veh = Vehicle(self._next_id, lane, x, v, length, profile, is_lorry=is_lorry, fixed=fixed)
        self._next_id += 1
        return veh

    def add_vehicle(
        self, lane: int, x: float, v: float, profile=None, is_lorry=False, fixed=False
    ) -> Vehicle:
        """Manually place a vehicle (used by tests and notebooks)."""
        if profile is None:
            profile = (
                LORRY_PROFILE
                if is_lorry
                else make_profile("normal", self.cfg.speed_limit, self.rng, 0.0)
            )
        length = LORRY_LENGTH if is_lorry else CAR_LENGTH
        veh = self._new_vehicle(lane, x, v, length, profile, is_lorry, fixed)
        self.road.insert(veh)
        self.inserted += 1
        return veh

    @property
    def vehicle_count(self) -> int:
        """Number of vehicles currently on the road."""
        return len(self.road)

    # --------------------------------------------------------------- dynamics
    def _accelerations(
        self,
        vehs: list[Vehicle],
        leaders: list[Vehicle | None],
        side_wake: dict[int, float] | None = None,
    ) -> np.ndarray:
        """Vectorised IDM + wake + safety clamp for ``vehs`` following ``leaders``."""
        if not vehs:
            return np.empty(0)
        cfg = self.cfg
        side_wake = side_wake or {}
        v = np.array([q.v for q in vehs])
        v0 = np.array([q.profile.v0 for q in vehs])
        a = np.array([q.profile.a for q in vehs])
        b = np.array([q.profile.b for q in vehs])
        tau = np.array([q.profile.tau for q in vehs])
        s0 = np.array([q.profile.s0 for q in vehs])
        gap = np.array([ld.rear - q.x if ld else np.inf for q, ld in zip(vehs, leaders)])
        dv = np.array([q.v - ld.v if ld else 0.0 for q, ld in zip(vehs, leaders)])
        strength = np.array(
            [self._wake_scale(ld) if ld is not None and ld.is_lorry else 0.0 for ld in leaders]
        )
        strength = np.where(gap < cfg.wake_length, strength, 0.0)
        strength = np.maximum(strength, [side_wake.get(q.id, 0.0) for q in vehs])

        a_eff, tau_eff = wake_adjusted(
            a, tau, strength, cfg.wake_accel_gain, cfg.wake_headway_factor
        )
        acc = idm_acceleration(v, v0, a_eff, b, tau_eff, s0, gap, dv)
        acc = apply_safety_clamp(acc, gap, s0, cfg.b_max)
        acc[[q.fixed for q in vehs]] = 0.0
        return acc

    def _wake_scale(self, lorry: Vehicle) -> float:
        """Drag reduction scales with dynamic pressure (~v^2); a parked lorry has no wake."""
        return min(1.0, (lorry.v / self.cfg.speed_limit) ** 2)

    def _side_wake(self) -> dict[int, float]:
        """Wake strength for vehicles trailing a lorry in an adjacent lane."""
        out: dict[int, float] = {}
        for lorry in (q for lane in self.road.lanes for q in lane if q.is_lorry and q.v > 0):
            strength = ADJACENT_WAKE_STRENGTH * self._wake_scale(lorry)
            for lane in (lorry.lane - 1, lorry.lane + 1):
                if 0 <= lane < self.road.num_lanes:
                    for q in self.road.lanes[lane]:
                        if lorry.rear - self.cfg.wake_length < q.x < lorry.rear:
                            out[q.id] = max(out.get(q.id, 0.0), strength)
        return out

    def _blocked(self, x: float, lane: int) -> bool:
        """True if a stationary obstacle lies ahead in ``lane`` within the lookahead."""
        return any(
            ob.fixed and ob.lane == lane and 0 < ob.rear - x < self.cfg.obstruction_lookahead
            for ob in self.road.obstacles
        )

    def _pairs(self) -> tuple[list[Vehicle], list[Vehicle | None]]:
        vehs, leaders = [], []
        for lane in self.road.lanes:
            vehs.extend(lane)
            if lane:
                leaders.extend(lane[1:] + [None])
        return vehs, leaders

    def update_accelerations(self) -> None:
        """Recompute and store every vehicle's current acceleration.

        Followers asked to yield (zipper merge) also treat the merging
        vehicle as a leader, braking at most at their comfortable rate ``b``.
        """
        vehs, leaders = self._pairs()
        for q, acc in zip(vehs, self._accelerations(vehs, leaders, self._side_wake())):
            q.a = float(acc)
        for follower, merger in self._yields:
            courtesy = float(self._accelerations([follower], [merger])[0])
            follower.a = max(min(follower.a, courtesy), -follower.profile.b)

    def evaluate_lane_change(self, veh: Vehicle, target: int) -> float | None:
        """MOBIL evaluation of moving ``veh`` into lane ``target``.

        Returns the biased incentive if the change is safe and worthwhile,
        otherwise ``None``. Uses each vehicle's stored current ``a``.
        """
        cfg = self.cfg
        if veh.is_lorry or not 0 <= target < self.road.num_lanes or target == veh.lane:
            return None
        new_leader, new_follower = self.road.neighbours(veh.x, target)
        if new_leader is not None and new_leader.rear - veh.x <= 0:
            return None  # physical overlap
        if new_follower is not None and veh.rear - new_follower.x <= 0:
            return None
        old_leader, old_follower = self.road.leader(veh), self.road.follower(veh)

        subjects, leads = [veh], [new_leader]
        if new_follower is not None:
            subjects.append(new_follower)
            leads.append(veh)
        if old_follower is not None:
            subjects.append(old_follower)
            leads.append(old_leader)
        acc = list(self._accelerations(subjects, leads))
        a_self_new = acc.pop(0)
        a_nf_new = acc.pop(0) if new_follower is not None else 0.0
        a_of_new = acc.pop(0) if old_follower is not None else 0.0
        a_nf_old = new_follower.a if new_follower is not None else 0.0
        a_of_old = old_follower.a if old_follower is not None else 0.0

        # A crawling follower cannot be endangered (its IDM output near standstill
        # is dominated by the s0 term); ignoring it lets zipper merges complete.
        crawling = new_follower is not None and new_follower.v < CREEP_SPEED
        if not mobil_is_safe(0.0 if crawling else a_nf_new, a_self_new, cfg.b_safe):
            return None
        gain = mobil_incentive(
            veh.a, a_self_new, a_nf_old, a_nf_new, a_of_old, a_of_new, veh.profile.politeness
        )
        gain += cfg.keep_left_bias if target < veh.lane else -cfg.keep_left_bias
        gain += cfg.obstruction_bias * (
            self._blocked(veh.x, veh.lane) - self._blocked(veh.x, target)
        )
        return gain if gain > cfg.lane_change_threshold else None

    def _lane_changes(self) -> None:
        self._yields = []
        for veh in self.road.vehicles:
            blocked = not veh.is_lorry and self._blocked(veh.x, veh.lane)
            if not blocked and (veh.id + self.step_count) % self._mobil_every:
                continue  # stagger MOBIL so each driver decides once per interval
            if self.t - veh.last_lane_change < self.cfg.lane_change_cooldown:
                continue
            options = [
                (g, lane)
                for lane in (veh.lane - 1, veh.lane + 1)
                if (g := self.evaluate_lane_change(veh, lane)) is not None
            ]
            if options:
                self.road.change_lane(veh, max(options)[1])
                veh.last_lane_change = self.t
            elif blocked:
                self._request_yield(veh)

    def _request_yield(self, veh: Vehicle) -> None:
        """Ask the follower in a free adjacent lane to open a gap for ``veh``."""
        for lane in (veh.lane + 1, veh.lane - 1):
            if 0 <= lane < self.road.num_lanes and not self._blocked(veh.x, lane):
                _, follower = self.road.neighbours(veh.x, lane)
                if follower is not None and veh.rear > follower.x and not follower.fixed:
                    self._yields.append((follower, veh))
                return

    def _integrate(self) -> None:
        """Ballistic update; vehicles that would reverse stop exactly at v = 0."""
        dt = self.cfg.dt
        vehs = self.road.vehicles
        if not vehs:
            return
        v = np.array([q.v for q in vehs])
        acc = np.array([q.a for q in vehs])
        v_new = v + acc * dt
        stops = v_new < 0
        dx = np.where(stops, -(v**2) / (2 * np.minimum(acc, -1e-9)), v * dt + 0.5 * acc * dt**2)
        v_new = np.maximum(v_new, 0.0)
        if not (np.all(np.isfinite(dx)) and np.all(np.isfinite(v_new))):
            raise RuntimeError(f"non-finite state at t={self.t:.2f}s; reduce dt")
        for q, d, vn in zip(vehs, dx, v_new):
            if not q.fixed:
                q.x += float(d)
                q.v = float(vn)
        self.road.resort()

    def _exits(self) -> None:
        for lane in self.road.lanes:
            while lane and lane[-1].x > self.road.length:
                self.road.remove(lane[-1])
                self.exited += 1

    def _inflow(self) -> None:
        cfg = self.cfg
        lam = cfg.inflow / 3600.0 / cfg.num_lanes * cfg.dt
        for lane in range(cfg.num_lanes):
            self.pending[lane] += int(self.rng.poisson(lam))
            if self.pending[lane] and self._try_insert(lane):
                self.pending[lane] -= 1

    def _try_insert(self, lane: int) -> bool:
        cfg = self.cfg
        is_lorry = lane == 0 and self.rng.random() < cfg.lorry_fraction
        if is_lorry:
            profile = LORRY_PROFILE
        else:
            kind = self._kinds[self.rng.choice(len(self._kinds), p=self._weights)]
            profile = make_profile(kind, cfg.speed_limit, self.rng)
        v = profile.v0
        vehs = self.road.lanes[lane]
        if vehs:
            # Enter at the leader's speed only if the IDM equilibrium gap is
            # available; otherwise wait (queue) rather than inject a crawling car.
            v = min(v, vehs[0].v)
            if vehs[0].rear < profile.s0 + v * profile.tau:  # front enters at x = 0
                return False
        length = LORRY_LENGTH if is_lorry else CAR_LENGTH
        self.road.insert(self._new_vehicle(lane, 0.0, max(v, 0.0), length, profile, is_lorry))
        self.inserted += 1
        return True

    def _record(self) -> None:
        t = round(self.t, 6)
        self._records.extend(
            (t, q.id, q.lane, q.x, q.v, q.a, q.is_lorry) for q in self.road.vehicles
        )

    # ------------------------------------------------------------------ public
    def step(self) -> None:
        """Advance the simulation by one timestep ``dt``."""
        self.update_accelerations()
        self._lane_changes()
        self.update_accelerations()
        self._integrate()
        self._exits()
        self._inflow()
        self.t += self.cfg.dt
        self.step_count += 1
        if self.step_count % self._record_every == 0:
            self._record()

    def run(self, duration: float | None = None) -> pd.DataFrame:
        """Run for ``duration`` seconds (default: config duration) and return history."""
        steps = round((duration if duration is not None else self.cfg.duration) / self.cfg.dt)
        for _ in range(steps):
            self.step()
        return self.history()

    def history(self) -> pd.DataFrame:
        """Recorded trajectories (downsampled to ``record_interval``)."""
        return pd.DataFrame(self._records, columns=RECORD_COLUMNS)
