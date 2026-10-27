"""Space-time diagrams and fundamental diagrams from recorded trajectories."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.figure import Figure

from .config import SimConfig

REQUIRED_COLUMNS = {"t", "id", "lane", "x", "v", "is_lorry"}


def _check(df: pd.DataFrame) -> None:
    missing = REQUIRED_COLUMNS - set(df.columns)
    if missing:
        raise ValueError(f"trajectory data missing columns: {sorted(missing)}")
    if df.empty:
        raise ValueError("trajectory data is empty")


def space_time_diagram(df: pd.DataFrame, num_lanes: int, speed_limit: float) -> Figure:
    """Position vs. time, one panel per lane, coloured by speed.

    Congestion appears as red bands; their upstream-sloping edges are the
    shockwaves. Lorry trajectories are overlaid in black.
    """
    _check(df)
    fig, axes = plt.subplots(
        num_lanes, 1, figsize=(11, 2.8 * num_lanes), sharex=True, squeeze=False
    )
    sc = None
    for lane, ax in zip(range(num_lanes - 1, -1, -1), axes[:, 0]):  # fast lane on top
        d = df[df.lane == lane]
        cars = d[~d.is_lorry]
        sc = ax.scatter(
            cars.t,
            cars.x / 1000,
            c=cars.v * 3.6,
            s=0.4,
            cmap="RdYlGn",
            vmin=0,
            vmax=speed_limit * 3.6 * 1.1,
            rasterized=True,
        )
        for _, tr in d[d.is_lorry].groupby("id"):
            ax.plot(tr.t, tr.x / 1000, color="black", lw=1.4)
        ax.set_ylabel(f"Lane {lane + 1}\nx (km)")
        ax.margins(x=0)
    axes[-1, 0].set_xlabel("Time (s)")
    fig.suptitle("Space-time diagram (black = lorry)")
    fig.colorbar(sc, ax=axes[:, 0], label="Speed (km/h)", pad=0.01)
    return fig


def fundamental_diagram_data(
    df: pd.DataFrame,
    record_interval: float,
    road_length: float,
    num_lanes: int,
    cell_length: float = 250.0,
    cell_duration: float = 30.0,
) -> pd.DataFrame:
    """Edie's generalised density/flow over space-time cells.

    For a cell of area |A| = dx * dt, density k = (total time spent) / |A| and
    flow q = (total distance travelled) / |A|. Each recorded sample stands for
    ``record_interval`` seconds of travel. Values are per lane.

    Returns columns: density (veh/km/lane), flow (veh/h/lane), speed (km/h).
    """
    _check(df)
    if cell_length <= 0 or cell_duration <= 0:
        raise ValueError("cell sizes must be positive")
    d = df[(df.x >= 0) & (df.x < road_length)]
    cells = d.assign(cx=(d.x // cell_length).astype(int), ct=(d.t // cell_duration).astype(int))
    g = cells.groupby(["cx", "ct"]).agg(n=("v", "size"), dist=("v", "sum"))
    area = cell_length * cell_duration
    k = g.n * record_interval / area  # veh/m (all lanes)
    q = g.dist * record_interval / area  # veh/s (all lanes)
    out = pd.DataFrame(
        {
            "density": k * 1000 / num_lanes,
            "flow": q * 3600 / num_lanes,
            "speed": q / k * 3.6,
        }
    )
    return out.reset_index(drop=True)


def fundamental_diagram(fd: pd.DataFrame) -> Figure:
    """Flow-density and speed-density scatter plots."""
    if fd.empty:
        raise ValueError("no fundamental diagram data")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
    ax1.scatter(fd.density, fd.flow, s=10, alpha=0.6, c=fd.speed, cmap="RdYlGn")
    ax1.set(xlabel="Density (veh/km/lane)", ylabel="Flow (veh/h/lane)", title="Flow-density")
    ax2.scatter(fd.density, fd.speed, s=10, alpha=0.6, c=fd.speed, cmap="RdYlGn")
    ax2.set(xlabel="Density (veh/km/lane)", ylabel="Speed (km/h)", title="Speed-density")
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
    fig.suptitle("Fundamental diagram (Edie, 250 m x 30 s cells)")
    fig.tight_layout()
    return fig


def save_figures(df: pd.DataFrame, cfg: SimConfig, out_dir: str | Path) -> list[Path]:
    """Render and save the standard figure set; returns written paths."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = [out / "space_time.png", out / "fundamental_diagram.png"]
    figs = [
        space_time_diagram(df, cfg.num_lanes, cfg.speed_limit),
        fundamental_diagram(
            fundamental_diagram_data(df, cfg.record_interval, cfg.road_length, cfg.num_lanes)
        ),
    ]
    for fig, path in zip(figs, paths):
        fig.savefig(path, dpi=150, bbox_inches="tight")
        plt.close(fig)
    return paths


def summary(df: pd.DataFrame) -> dict[str, float]:
    """Headline statistics of a run (speeds over cars only, excluding lorries)."""
    _check(df)
    v = df.v[~df.is_lorry]
    return {
        "mean_speed_kmh": float(v.mean() * 3.6),
        "min_speed_kmh": float(v.min() * 3.6),
        "share_below_30kmh": float(np.mean(v < 30 / 3.6)),
        "vehicles_seen": int(df.id.nunique()),
    }
