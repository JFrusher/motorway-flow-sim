"""Elephant race: what a lorry overtaking another lorry costs everyone behind.

python scripts/elephant_race.py [--output output/elephant_race] [--workers N]

Runs, each against a same-seed baseline where the lorries stay in lane 1:
  * 3-lane motorway, 56 vs 55 mph (headline) plus a speed-gap sweep
  * 2-lane motorway, 56 vs 55 mph (the overtake blocks the whole carriageway)
Writes dashboard.png, speed_gap_sweep.png, report.md and metrics.json.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from motorway import SimConfig  # noqa: E402
from motorway.overtake import (  # noqa: E402
    MPH,
    ElephantRace,
    RaceRun,
    impact,
    overtake_window,
    passing_flow,
    queue_behind,
    run_race,
)

FAST_MPH = 56.0
HEADLINE_GAP = 1.0
SWEEP_GAPS = [0.5, 1.0, 2.0, 3.0, 5.0]
FLOW_PER_LANE = 1400  # veh/h/lane: busy but free-flowing without the overtake


ROAD_M, DURATION_S = 20000, 780  # long enough for the slowest overtake to finish and clear


def scenario(lanes: int) -> SimConfig:
    return SimConfig(
        road_length=ROAD_M,
        duration=DURATION_S,
        num_lanes=lanes,
        inflow=FLOW_PER_LANE * lanes,
        bottleneck_lorry_x=None,
    )


def simulate(case: tuple[int, float, bool]) -> tuple[tuple[int, float, bool], RaceRun]:
    lanes, gap, overtake = case
    race = ElephantRace(fast_mph=FAST_MPH, slow_mph=FAST_MPH - gap, overtake=overtake)
    return case, run_race(scenario(lanes), race)


# ------------------------------------------------------------------ figures
def speed_field(run: RaceRun, dx: float = 200.0, dt: float = 10.0) -> np.ndarray:
    cars = run.history[~run.history.id.isin(run.race.lorry_ids)]
    grid = cars.assign(ix=(cars.x // dx).astype(int), it=(cars.t // dt).astype(int))
    field = grid.groupby(["ix", "it"]).v.mean().unstack()
    nx, nt = int(run.cfg.road_length // dx), int(run.cfg.duration // dt) + 1
    return field.reindex(index=range(nx), columns=range(nt)).to_numpy() * 3.6


def heatmap(ax, run: RaceRun, title: str, baseline: RaceRun | None = None):
    """Speed field, or with ``baseline`` the speed lost relative to it (same seed)."""
    extent = [0, run.cfg.duration, 0, run.cfg.road_length / 1000]
    if baseline is None:
        im = ax.imshow(
            speed_field(run),
            origin="lower",
            aspect="auto",
            cmap="RdYlGn",
            vmin=40,
            vmax=125,
            extent=extent,
        )
    else:
        loss = speed_field(run) - speed_field(baseline)
        im = ax.imshow(
            loss, origin="lower", aspect="auto", cmap="RdBu", vmin=-30, vmax=30, extent=extent
        )
    for lid, style in zip(run.race.lorry_ids, ["-", "--"]):
        tr = run.history[run.history.id == lid]
        ax.plot(tr.t, tr.x / 1000, "k", ls=style, lw=1.3)
    if run.race.t_out is not None:
        for t in overtake_window(run):
            ax.axvline(t, color="black", ls=":", lw=1)
    ax.set(title=title, xlabel="Time (s)", ylabel="Position (km)")
    return im


def dashboard(runs: dict, results: dict, path: Path) -> None:
    b3, o3 = runs[(3, HEADLINE_GAP, False)], runs[(3, HEADLINE_GAP, True)]
    b2, o2 = runs[(2, HEADLINE_GAP, False)], runs[(2, HEADLINE_GAP, True)]
    fig = plt.figure(figsize=(15, 13))
    gs = fig.add_gridspec(3, 3, height_ratios=[1, 1, 0.8], hspace=0.38, wspace=0.28)
    ax0 = fig.add_subplot(gs[0, 0])
    im = heatmap(ax0, o3, "3 lanes with overtake: car speed")
    fig.colorbar(im, ax=ax0, label="km/h", pad=0.02)
    ax1, ax2 = fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[0, 2])
    heatmap(ax1, o3, "3 lanes: speed change vs no overtake", b3)
    im = heatmap(ax2, o2, "2 lanes: speed change vs no overtake", b2)
    fig.colorbar(im, ax=[ax1, ax2], label="km/h (red = slower)", pad=0.01)
    fig.text(
        0.5,
        0.905,
        "Dotted lines: overtake starts and ends. Solid/dashed black: the two " "lorries.",
        ha="center",
        fontsize=10,
        color="0.3",
    )

    ax = fig.add_subplot(gs[1, :2])
    for b, o, label, c in [(b3, o3, "3 lanes", "C0"), (b2, o2, "2 lanes", "C3")]:
        for run, ls, kind in [(b, "--", "baseline"), (o, "-", "overtake")]:
            flow = passing_flow(run.history, run.race.lorry_ids).rolling(2, min_periods=1).mean()
            ax.plot(flow.index, flow.values, color=c, ls=ls, label=f"{label}, {kind}")
    ax.set(
        title="Cars getting past the lorries", xlabel="Time (s)", ylabel="Overtaking rate (veh/h)"
    )
    ax.legend(ncol=2, fontsize=9)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[1, 2])
    hold = FAST_MPH * MPH + 1.0
    for b, o, label, c in [(b3, o3, "3 lanes", "C0"), (b2, o2, "2 lanes", "C3")]:
        for run, ls in [(b, "--"), (o, "-")]:
            q = queue_behind(run.history, run.race.lorry_ids, hold)
            ax.plot(q.t, q.vehicles, color=c, ls=ls, label=label if ls == "-" else None)
    ax.set(title="Cars held at lorry speed", xlabel="Time (s)", ylabel="Vehicles")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[2, :])
    ax.axis("off")
    r3, r2 = results[(3, HEADLINE_GAP)], results[(2, HEADLINE_GAP)]
    rows = [
        (
            "Overtake duration",
            lambda r: f"{r['overtake']['blocking_s']:.0f} s"
            + ("" if r["overtake"]["overtake_completed"] else " (unfinished)"),
        ),
        (
            "Overtaking rate past the lorries",
            lambda r: (
                f"{r['baseline']['passing_flow_during_overtake_veh_h']:.0f} -> "
                f"{r['overtake']['passing_flow_during_overtake_veh_h']:.0f} veh/h "
                f"(-{r['passing_flow_drop_pct']:.0f}%)"
            ),
        ),
        (
            "Most cars held at lorry speed",
            lambda r: (
                f"{r['baseline']['max_queue_vehicles']} -> {r['overtake']['max_queue_vehicles']}"
            ),
        ),
        (
            "Platoon length behind lorries",
            lambda r: (
                f"{r['baseline']['max_queue_m']:.0f} -> {r['overtake']['max_queue_m']:.0f} m"
            ),
        ),
        (
            "Drivers held up behind lorries",
            lambda r: (f"{r['baseline']['drivers_held_up']} -> {r['overtake']['drivers_held_up']}"),
        ),
        (
            "Extra time stuck behind lorries",
            lambda r: f"{r['extra_stuck_vehicle_min']:.0f} veh-min",
        ),
        (
            "Extra delay",
            lambda r: (
                f"{r['extra_vehicle_hours'] * 60:.0f} veh-min in total, "
                f"{r['extra_delay_per_held_driver_s']:.0f} s per held-up driver"
            ),
        ),
        (
            "Extra lane changes / hard brakes",
            lambda r: (f"{r['extra_lane_changes']:+d} / {r['extra_hard_brakes']:+d}"),
        ),
    ]
    table = ax.table(
        cellText=[[name, f(r3), f(r2)] for name, f in rows],
        colLabels=["56 vs 55 mph elephant race", "3-lane motorway", "2-lane motorway"],
        cellLoc="left",
        colLoc="left",
        loc="center",
        colWidths=[0.3, 0.33, 0.33],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1, 1.7)
    fig.suptitle(
        f"Elephant race: a {FAST_MPH:.0f} mph lorry overtaking a "
        f"{FAST_MPH - HEADLINE_GAP:.0f} mph lorry ({FLOW_PER_LANE} veh/h/lane)",
        fontsize=15,
        y=0.995,
    )
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)


def sweep_plot(results: dict, path: Path) -> None:
    gaps = SWEEP_GAPS
    rs = [results[(3, g)] for g in gaps]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.3))
    durations = [r["overtake"]["blocking_s"] for r in rs]
    done = [r["overtake"]["overtake_completed"] for r in rs]
    bars = ax1.bar([str(g) for g in gaps], durations, color="C0")
    for bar, ok in zip(bars, done):
        if not ok:
            bar.set_hatch("//")
            bar.set_facecolor("C1")
    ax1.set(
        title="How long lanes 1 and 2 are blocked",
        xlabel="Speed difference (mph)",
        ylabel="Overtake duration (s)",
    )
    ax1.text(
        0.98,
        0.95,
        "hatched = still side by side\nwhen the lorries left the road",
        transform=ax1.transAxes,
        ha="right",
        va="top",
        fontsize=9,
    )

    ax2.plot(gaps, [r["extra_stuck_vehicle_min"] for r in rs], "o-", label="Extra veh-min stuck")
    ax2.plot(gaps, [r["extra_vehicle_hours"] * 60 for r in rs], "s-", label="Extra veh-min delay")
    ax2.set(
        title="Cost to traffic behind (3 lanes)",
        xlabel="Speed difference (mph)",
        ylabel="Vehicle-minutes",
    )
    ax2.legend()
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------- report
def report(results: dict, path: Path) -> None:
    def row(name, key, fmt="{:.1f}"):
        cells = []
        for lanes in (3, 2):
            r = results[(lanes, HEADLINE_GAP)]
            cells += [fmt.format(r["baseline"][key]), fmt.format(r["overtake"][key])]
        return f"| {name} | " + " | ".join(cells) + " |"

    lines = [
        "# Elephant race report",
        "",
        f"A {FAST_MPH:.0f} mph lorry overtakes a {FAST_MPH - HEADLINE_GAP:.0f} mph lorry. "
        f"Demand is {FLOW_PER_LANE} veh/h/lane on a {ROAD_M / 1000:.0f} km road, simulated "
        f"for {DURATION_S} s. "
        "Baselines use the same seed with both lorries kept in lane 1.",
        "",
        "| Metric | 3-lane baseline | 3-lane overtake | 2-lane baseline | 2-lane overtake |",
        "|---|---|---|---|---|",
        row("Overtake duration (s)", "blocking_s", "{:.0f}"),
        row("Overtaking rate past lorries (veh/h)", "passing_flow_during_overtake_veh_h", "{:.0f}"),
        row("Max cars held at lorry speed", "max_queue_vehicles", "{:.0f}"),
        row("Max platoon length (m)", "max_queue_m", "{:.0f}"),
        row("Platoon growth rate (km/h)", "queue_growth_kmh"),
        row("Vehicle-minutes stuck", "stuck_vehicle_min", "{:.0f}"),
        row("Drivers held up", "drivers_held_up", "{:.0f}"),
        row("Total time spent (veh-h)", "total_time_spent_veh_h", "{:.2f}"),
        row("Mean car speed (km/h)", "mean_car_speed_kmh"),
        row("Lane changes", "lane_changes", "{:.0f}"),
        row("Moves into the outside lane", "moves_into_fast_lane", "{:.0f}"),
        row("Hard-braking events (< -3 m/s²)", "hard_brake_events", "{:.0f}"),
        "",
        "## Speed-gap sweep (3 lanes)",
        "",
        "| Gap (mph) | Overtake (s) | Finished | Extra veh-min stuck | Extra delay (veh-min) |"
        " Overtaking-rate drop |",
        "|---|---|---|---|---|---|",
    ]
    for g in SWEEP_GAPS:
        r = results[(3, g)]
        lines.append(
            f"| {g} | {r['overtake']['blocking_s']:.0f} | "
            f"{'yes' if r['overtake']['overtake_completed'] else 'no'} | "
            f"{r['extra_stuck_vehicle_min']:.0f} | {r['extra_vehicle_hours'] * 60:.1f} | "
            f"{r['passing_flow_drop_pct']:.0f}% |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--output", type=Path, default=Path("output/elephant_race"))
    parser.add_argument("--workers", type=int, default=None, help="parallel processes")
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)

    cases = [(3, g, ot) for g in SWEEP_GAPS for ot in (False, True)]
    cases += [(2, HEADLINE_GAP, ot) for ot in (False, True)]
    start = time.perf_counter()
    with ProcessPoolExecutor(args.workers) as pool:
        runs = dict(pool.map(simulate, cases))
    print(f"{len(cases)} runs in {time.perf_counter() - start:.0f}s")

    results = {
        (lanes, g): impact(runs[(lanes, g, False)], runs[(lanes, g, True)])
        for lanes, g, ot in cases
        if ot
    }
    dashboard(runs, results, args.output / "dashboard.png")
    sweep_plot(results, args.output / "speed_gap_sweep.png")
    report(results, args.output / "report.md")
    as_json = {f"{lanes}_lanes_gap_{g}mph": r for (lanes, g), r in results.items()}
    (args.output / "metrics.json").write_text(
        json.dumps(as_json, indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o)),
        encoding="utf-8",
    )
    for name in ("dashboard.png", "speed_gap_sweep.png", "report.md", "metrics.json"):
        print(f"Wrote {args.output / name}")
    print((args.output / "report.md").read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
