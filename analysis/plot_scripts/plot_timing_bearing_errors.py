#!/usr/bin/env python3
"""Plot logged bearing error over elapsed time for one or more timing runs."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="+", type=Path, help="bearing-error CSV files")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/plots/timing_bearing_error.png"),
        help="PNG destination (default: %(default)s)",
    )
    args = parser.parse_args()

    fig, ax = plt.subplots(figsize=(11, 5))
    for path in args.csv:
        data = pd.read_csv(path).dropna(subset=["t", "angle_error_deg"])
        if data.empty:
            raise ValueError(f"{path}: no bearing-error rows")
        elapsed_s = data["t"] - data["t"].iloc[0]
        mean_error = data["angle_error_deg"].mean()
        mae = data["angle_error_deg"].abs().mean()
        print(
            f"{path}: n={len(data)}, mean={mean_error:+.2f} deg, "
            f"mean absolute error={mae:.2f} deg"
        )
        ax.plot(
            elapsed_s,
            data["angle_error_deg"],
            ".",
            ms=2,
            label=(
                f"{path.stem} (mean {mean_error:+.2f} deg, "
                f"MAE {mae:.2f} deg)"
            ),
        )

    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set(xlabel="elapsed time [s]", ylabel="signed bearing error [deg]")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
