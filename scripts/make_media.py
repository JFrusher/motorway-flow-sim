"""Regenerate the README media in docs/ from real simulation runs.

python scripts/make_media.py        (takes a few minutes)

Writes docs/breakdown.gif, docs/wake.gif, docs/space_time.gif and the static PNGs.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import pygame  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from PIL import Image  # noqa: E402

from motorway import Simulation, load_config  # noqa: E402
from motorway.analytics import save_figures, space_time_diagram  # noqa: E402
from run_simulation import draw_road  # noqa: E402

DOCS = ROOT / "docs"


def to_image(surface: pygame.Surface) -> Image.Image:
    return Image.frombytes("RGB", surface.get_size(), pygame.image.tobytes(surface, "RGB"))


def save_gif(frames: list[Image.Image], path: Path, ms_per_frame: int) -> None:
    frames = [f.quantize(colors=64, dither=Image.Dither.NONE) for f in frames]
    frames[0].save(
        path, save_all=True, append_images=frames[1:], duration=ms_per_frame, loop=0, optimize=True
    )
    print(f"Wrote {path} ({path.stat().st_size / 1e6:.1f} MB, {len(frames)} frames)")


def caption(surface, font, text: str) -> None:
    surface.blit(font.render(text, True, (230, 230, 230)), (30, 12))


def advance(sim: Simulation, seconds: float) -> None:
    for _ in range(round(seconds / sim.cfg.dt)):
        sim.step()


def breakdown_media(font) -> None:
    """Whole-road view of the default scenario, then its space-time build-up."""
    cfg = load_config(ROOT / "config.yaml")
    sim = Simulation(cfg)
    surface = pygame.Surface((960, 400))
    advance(sim, 100)
    frames = []
    while sim.t < 400:
        advance(sim, 2.0)
        draw_road(surface, sim, font, lane_px=10)
        caption(surface, font, f"Broken-down lorry in lane 1 at 3 km    t = {sim.t:3.0f} s")
        frames.append(to_image(surface))
    save_gif(frames, DOCS / "breakdown.gif", 80)

    advance(sim, cfg.duration - sim.t)
    df = sim.history()
    save_figures(df, cfg, DOCS)

    fig = space_time_diagram(df, cfg.num_lanes, cfg.speed_limit)
    fig.set_size_inches(9, 6.6)
    t_max = df.t.max()
    curtains = [ax.axvspan(0, t_max, color="white", zorder=5) for ax in fig.axes[:-1]]

    def update(t):
        for c in curtains:
            c.set_x(t)
        return curtains

    times = list(range(0, int(t_max) + 8, 8)) + [int(t_max)] * 12  # hold the final frame
    FuncAnimation(fig, update, frames=times).save(
        DOCS / "space_time.gif", writer=PillowWriter(fps=12), dpi=80
    )
    plt.close(fig)
    print(f"Wrote {DOCS / 'space_time.gif'}")


def wake_media(font) -> None:
    """Camera following the slow lorry: drafting cars and overtakes."""
    sim = Simulation(load_config(ROOT / "config_moving_lorry.yaml"))
    lorry = sim.road.obstacles[0]
    surface = pygame.Surface((960, 190))
    advance(sim, 150)
    frames = []
    for _ in range(140):
        advance(sim, 0.5)
        draw_road(surface, sim, font, lorry.x - 380, lorry.x + 60, rows=1, lane_px=30, top=70)
        caption(
            surface,
            font,
            f"Lorry at 12 m/s, shaded zone = 40 m slipstream    t = {sim.t:3.0f} s",
        )
        frames.append(to_image(surface))
    save_gif(frames, DOCS / "wake.gif", 60)


def main() -> int:
    DOCS.mkdir(exist_ok=True)
    pygame.font.init()
    font = pygame.font.Font(None, 22)
    wake_media(font)
    breakdown_media(font)
    return 0


if __name__ == "__main__":
    sys.exit(main())
