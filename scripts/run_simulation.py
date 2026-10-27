"""CLI entry point.

Headless:  python scripts/run_simulation.py --config config.yaml
Live view: python scripts/run_simulation.py --render
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib  # noqa: E402

matplotlib.use("Agg")

from motorway import Simulation, load_config  # noqa: E402
from motorway.analytics import save_figures, summary  # noqa: E402

WIDTH, HEIGHT = 1400, 820
ROWS = 4  # road is wrapped onto this many rows to fit the window
LANE_PX = 12
MARGIN = 30
FPS = 30
STEPS_PER_FRAME = 2  # 30 fps x 2 x 0.1 s = 6x real time


def speed_colour(v: float, v_max: float) -> tuple[int, int, int]:
    """Red (stopped) -> yellow -> green (free flow)."""
    f = max(0.0, min(1.0, v / v_max))
    return (255, int(510 * f), 0) if f < 0.5 else (int(510 * (1 - f)), 220, 0)


def draw_road(
    surface,
    sim: Simulation,
    font,
    x_start: float = 0.0,
    x_end: float | None = None,
    rows: int = ROWS,
    lane_px: int = LANE_PX,
    top: int = 60,
) -> None:
    """Draw road section [x_start, x_end) wrapped onto ``rows`` rows of ``surface``.

    Cars are coloured by speed; lorries are blue with a white bounding box and,
    when moving, a shaded slipstream zone behind them.
    """
    import pygame

    cfg = sim.cfg
    x_end = cfg.road_length if x_end is None else x_end
    width, height = surface.get_size()
    seg_len = (x_end - x_start) / rows
    scale = (width - 2 * MARGIN) / seg_len
    row_h = cfg.num_lanes * lane_px
    row_gap = (height - top - rows * row_h) / rows
    v_max = cfg.speed_limit * 1.1

    surface.fill((25, 25, 30))
    for row in range(rows):
        y0 = top + row * (row_h + row_gap)
        pygame.draw.rect(surface, (60, 60, 66), (MARGIN, y0, width - 2 * MARGIN, row_h))
        for lane in range(1, cfg.num_lanes):
            y = y0 + lane * lane_px
            pygame.draw.line(surface, (120, 120, 120), (MARGIN, y), (width - MARGIN, y))
        label = font.render(f"{(x_start + row * seg_len) / 1000:.2f} km", True, (150, 150, 150))
        surface.blit(label, (MARGIN, y0 - 18))

    surface.set_clip(pygame.Rect(MARGIN, 0, width - 2 * MARGIN, height))
    for q in sim.road.vehicles:
        if not x_start <= q.x < x_end:
            continue
        row = int((q.x - x_start) // seg_len)
        sx = MARGIN + (q.x - x_start - row * seg_len) * scale
        sy = top + row * (row_h + row_gap) + (cfg.num_lanes - 1 - q.lane) * lane_px
        w = max(2.0, q.length * scale)
        rect = pygame.Rect(sx - w, sy + 2, w, lane_px - 3)
        if q.is_lorry:
            if q.v > 0:  # parked lorry has no slipstream
                wake_w = cfg.wake_length * scale
                pygame.draw.rect(
                    surface, (40, 70, 120), (sx - w - wake_w, sy + 1, wake_w, lane_px - 1)
                )
            pygame.draw.rect(surface, (90, 90, 230), rect)
            pygame.draw.rect(surface, (255, 255, 255), rect.inflate(4, 4), 1)
        else:
            pygame.draw.rect(surface, speed_colour(q.v, v_max), rect)
    surface.set_clip(None)


def render(sim: Simulation) -> None:
    """Real-time Pygame view. Space pauses, Esc/close quits."""
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("motorway-flow-sim")
    font = pygame.font.Font(None, 20)  # built-in font; SysFont scans system fonts (slow)
    clock = pygame.time.Clock()
    steps_total = round(sim.cfg.duration / sim.cfg.dt)
    paused = False

    while sim.step_count < steps_total:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
            ):
                pygame.quit()
                return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                paused = not paused
        if not paused:
            for _ in range(STEPS_PER_FRAME):
                sim.step()

        draw_road(screen, sim, font)
        vehicles = sim.road.vehicles
        mean_v = sum(q.v for q in vehicles) / len(vehicles) * 3.6 if vehicles else 0.0
        hud = (
            f"t={sim.t:6.1f}s  vehicles={len(vehicles):4d}  mean={mean_v:5.1f} km/h  "
            f"in={sim.inserted} out={sim.exited} queued={sum(sim.pending)}"
            f"{'  [PAUSED]' if paused else ''}   space=pause esc=quit"
        )
        screen.blit(font.render(hud, True, (230, 230, 230)), (MARGIN, 12))
        pygame.display.flip()
        clock.tick(FPS)
    pygame.quit()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--config", type=Path, help="YAML config file")
    parser.add_argument("--render", action="store_true", help="live Pygame visualisation")
    parser.add_argument("--duration", type=float, help="override simulated seconds")
    parser.add_argument("--seed", type=int, help="override random seed")
    parser.add_argument("--output", type=Path, default=Path("output"), help="output directory")
    parser.add_argument("--no-plots", action="store_true", help="skip figure generation")
    args = parser.parse_args(argv)

    try:
        cfg = load_config(args.config, duration=args.duration, seed=args.seed)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    sim = Simulation(cfg)

    start = time.perf_counter()
    if args.render:
        render(sim)
    else:
        sim.run()
    print(f"Simulated {sim.t:.1f}s in {time.perf_counter() - start:.1f}s wall time")

    df = sim.history()
    args.output.mkdir(parents=True, exist_ok=True)
    csv_path = args.output / "trajectories.csv"
    df.to_csv(csv_path, index=False)
    print(f"Wrote {csv_path} ({len(df)} rows)")
    if not args.no_plots and not df.empty:
        for path in save_figures(df, cfg, args.output):
            print(f"Wrote {path}")
    for key, value in summary(df).items():
        print(f"  {key}: {value:.3f}" if isinstance(value, float) else f"  {key}: {value}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
