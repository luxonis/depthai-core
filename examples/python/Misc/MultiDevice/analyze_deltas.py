#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import EngFormatter


DELTA_COL = "delta_t [s]"


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze frame delta CSV (pandas).")
    parser.add_argument("csv_path", type=Path, help="Path to frame_deltas_*.csv")
    parser.add_argument("--bins", type=int, default=100, help="Histogram bins (default: 100)")
    parser.add_argument(
        "--max-points",
        type=int,
        default=2_000_000,
        help="Max points to plot for time-series (downsamples if larger).",
    )
    parser.add_argument(
        "--start-frame",
        type=int,
        default=1000,
        help="Start frame index for statistics.",
    )
    args = parser.parse_args()

    # Load CSV, skip metadata lines
    df = pd.read_csv(args.csv_path, comment="#")

    required_cols = {"frame_idx", DELTA_COL}
    if not required_cols.issubset(df.columns):
        raise ValueError(f"Expected columns: {required_cols}. Found: {list(df.columns)}")

    # Clean and coerce types
    df["frame_idx"] = pd.to_numeric(df["frame_idx"], errors="coerce")
    df[DELTA_COL] = pd.to_numeric(df[DELTA_COL], errors="coerce")
    df = df.dropna(subset=["frame_idx", DELTA_COL])
    df = df[df[DELTA_COL] > 0.0]  # required for log scale

    # Statistics from frame_idx >= start-frame
    stats_df = df[df["frame_idx"] >= args.start_frame]
    if stats_df.empty:
        raise ValueError(f"No data with frame_idx >= {args.start_frame}")

    s = stats_df[DELTA_COL]
    mean_s = s.mean()
    std_s = s.std(ddof=1)
    max_s = s.max()
    p99_s = s.quantile(0.99)
    p999_s = s.quantile(0.999)

    threshold_s = 1e-3  # 1 ms
    under_1ms_pct = (s < threshold_s).mean() * 100.0

    print(f"Samples used (stats): {len(stats_df)}  (frame_idx >= {args.start_frame})")
    print(f"Mean delta:    {mean_s:.6e} s")
    print(f"Std dev:       {std_s:.6e} s")
    print(f"Max delta:     {max_s:.6e} s")
    print(f"P99 delta:     {p99_s:.6e} s")
    print(f"P999 delta:    {p999_s:.6e} s")
    print(f"Frames < 1 ms: {under_1ms_pct:.2f} %")

    # Downsample for plotting only
    if len(df) > args.max_points:
        step = max(1, len(df) // args.max_points)
        plot_df = df.iloc[::step]
        print(f"Plotting every {step}th sample ({len(plot_df)} points).")
    else:
        plot_df = df

    # Single window with two plots (report-friendly)
    fig, (ax_ts, ax_hist) = plt.subplots(
        1, 2, figsize=(12, 4.5), constrained_layout=True
    )

    eng_s = EngFormatter(unit="s")

    # Time-series plot
    ax_ts.plot(plot_df["frame_idx"], plot_df[DELTA_COL])
    ax_ts.set_yscale("log")
    ax_ts.yaxis.set_major_formatter(eng_s)
    ax_ts.set_xlabel("frame_idx")
    ax_ts.set_ylabel("delta")
    ax_ts.set_title("Frame delta over frame index")

    # Histogram
    ax_hist.hist(df[DELTA_COL], bins=args.bins)
    ax_hist.set_yscale("log")
    ax_hist.xaxis.set_major_formatter(eng_s)
    ax_hist.set_xlabel("delta")
    ax_hist.set_ylabel("count")
    ax_hist.set_title("Distribution of frame deltas")

    # Vertical line at 1 ms
    ax_hist.axvline(
        threshold_s,
        color="black",
        linestyle="--",
        linewidth=1.5,
        label="1 ms",
    )
    ax_hist.legend()

    plt.show()


if __name__ == "__main__":
    main()
