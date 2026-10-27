"""Regenerate analytical figures from a saved trajectories CSV.

python scripts/generate_plots.py --input output/trajectories.csv --config config.yaml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib  # noqa: E402

matplotlib.use("Agg")

import pandas as pd  # noqa: E402

from motorway import load_config  # noqa: E402
from motorway.analytics import save_figures  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--input", type=Path, default=Path("output/trajectories.csv"))
    parser.add_argument(
        "--config", type=Path, help="config used for the run (road length, lanes, record interval)"
    )
    parser.add_argument("--output", type=Path, default=Path("output"))
    args = parser.parse_args(argv)

    try:
        cfg = load_config(args.config)
        df = pd.read_csv(args.input)
        paths = save_figures(df, cfg, args.output)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    for path in paths:
        print(f"Wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
