#!/usr/bin/env python3

"""Plot particle-filter error metrics for the D3 report."""

import argparse
import csv
import os


def parse_args():
    parser = argparse.ArgumentParser(description="Plot D3 particle-filter metrics.")
    parser.add_argument("--csv", required=True, help="CSV log from particle_filter.py")
    parser.add_argument("--output", required=True, help="Output PNG path")
    return parser.parse_args()


def load_rows(csv_path):
    rows = []
    with open(csv_path, "r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append({key: float(value) for key, value in row.items()})
    return rows


def main():
    args = parse_args()
    rows = load_rows(args.csv)
    if not rows:
        raise SystemExit(f"No rows found in {args.csv}")

    import matplotlib.pyplot as plt

    iterations = [row["iteration"] for row in rows]
    position_errors = [row["position_error_m"] for row in rows]
    yaw_errors = [row["yaw_error_rad"] for row in rows]
    neff = [row["neff"] for row in rows]
    max_weight = [row["max_weight"] for row in rows]

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(iterations, position_errors, color="#1f77b4", linewidth=2)
    axes[0].set_ylabel("Position error (m)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(iterations, yaw_errors, color="#ff7f0e", linewidth=2)
    axes[1].set_ylabel("Yaw error (rad)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(iterations, neff, color="#2ca02c", linewidth=2, label="N_eff")
    axes[2].plot(
        iterations,
        max_weight,
        color="#d62728",
        linewidth=1.5,
        label="max weight",
    )
    axes[2].set_ylabel("Particle stats")
    axes[2].set_xlabel("Correction iteration")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="best")

    fig.suptitle("Particle Filter Localization Metrics")
    fig.tight_layout()
    output_dir = os.path.dirname(args.output)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    print(f"wrote {os.path.abspath(args.output)}")


if __name__ == "__main__":
    main()
