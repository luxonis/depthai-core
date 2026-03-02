#!/usr/bin/env python3
import argparse
import csv
import os
import re
import statistics
import subprocess
import sys
from datetime import datetime
from pathlib import Path


COMPARE_RE = re.compile(
    r"\[AUTOCAL_COMPARE\]\s+off_avg_fps=([-+]?\d*\.?\d+)\s+on_avg_fps=([-+]?\d*\.?\d+)\s+drop_fps=([-+]?\d*\.?\d+)\s+drop_pct=([-+]?\d*\.?\d+)"
)
BATCH_RE = re.compile(r"\[AUTOCAL_BENCH\].*avg_fps=([-+]?\d*\.?\d+)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run regression_camera_concurrency_autocal_compare_test repeatedly and plot off/on/drop histograms."
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=100,
        help="Number of runs (default: 100).",
    )
    parser.add_argument(
        "--command",
        type=str,
        default="./build/tests/regression_camera_concurrency_autocal_compare_test",
        help="Parent compare command (used when --mode=parent).",
    )
    parser.add_argument(
        "--off-command",
        type=str,
        default="./build/tests/regression_camera_concurrency_autocal_off_test",
        help="OFF command (used when --mode=direct).",
    )
    parser.add_argument(
        "--on-command",
        type=str,
        default="./build/tests/regression_camera_concurrency_autocal_on_test",
        help="ON command (used when --mode=direct).",
    )
    parser.add_argument(
        "--mode",
        choices=["direct", "parent"],
        default="direct",
        help="Benchmark mode: direct runs OFF/ON child tests; parent runs compare test.",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=2,
        help="Retries per run when command fails (default: 2).",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("build") / "autocal_benchmark_results",
        help="Directory for CSV and histogram output.",
    )
    parser.add_argument(
        "--depthai-level",
        type=str,
        default="info",
        help="DEPTHAI_LEVEL for child command (default: info).",
    )
    parser.add_argument(
        "--continue-on-fail",
        action="store_true",
        help="Continue collecting runs even if some command executions fail.",
    )
    return parser.parse_args()


def summarize(name: str, values: list[float]) -> str:
    if not values:
        return f"{name}: no data"
    mean_v = statistics.mean(values)
    median_v = statistics.median(values)
    stdev_v = statistics.pstdev(values) if len(values) > 1 else 0.0
    return (
        f"{name}: n={len(values)} mean={mean_v:.4f} median={median_v:.4f} "
        f"std={stdev_v:.4f} min={min(values):.4f} max={max(values):.4f}"
    )


def plot_histograms(off_vals: list[float], on_vals: list[float], drop_vals: list[float], output_png: Path) -> bool:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    bins = 20

    axes[0].hist(off_vals, bins=bins, color="#1f77b4", alpha=0.85, edgecolor="black")
    axes[0].set_title("OFF avg FPS")
    axes[0].set_xlabel("FPS")
    axes[0].set_ylabel("Count")

    axes[1].hist(on_vals, bins=bins, color="#ff7f0e", alpha=0.85, edgecolor="black")
    axes[1].set_title("ON_START avg FPS")
    axes[1].set_xlabel("FPS")
    axes[1].set_ylabel("Count")

    axes[2].hist(drop_vals, bins=bins, color="#2ca02c", alpha=0.85, edgecolor="black")
    axes[2].set_title("FPS drop (OFF - ON)")
    axes[2].set_xlabel("FPS")
    axes[2].set_ylabel("Count")

    fig.tight_layout()
    fig.savefig(output_png, dpi=150)
    plt.close(fig)
    return True


def main() -> int:
    args = parse_args()
    if args.runs <= 0:
        print("--runs must be > 0", file=sys.stderr)
        return 2

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.out_dir / f"autocal_compare_{timestamp}.csv"
    png_path = args.out_dir / f"autocal_compare_{timestamp}.png"

    env = os.environ.copy()
    if args.depthai_level:
        env["DEPTHAI_LEVEL"] = args.depthai_level

    off_vals: list[float] = []
    on_vals: list[float] = []
    drop_vals: list[float] = []
    drop_pct_vals: list[float] = []

    failures = 0
    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["run", "off_avg_fps", "on_avg_fps", "drop_fps", "drop_pct", "returncode"])
        from tqdm import tqdm
        for i in tqdm(range(1, args.runs + 1)):
            ok = False
            off = on = drop = drop_pct = 0.0
            final_rc = -1
            last_output = ""
            for attempt in range(args.retries + 1):
                if args.mode == "parent":
                    proc = subprocess.run(args.command, shell=True, capture_output=True, text=True, env=env)
                    output = (proc.stdout or "") + (proc.stderr or "")
                    m = COMPARE_RE.search(output)
                    final_rc = proc.returncode
                    last_output = output
                    if proc.returncode == 0 and m is not None:
                        off = float(m.group(1))
                        on = float(m.group(2))
                        drop = float(m.group(3))
                        drop_pct = float(m.group(4))
                        ok = True
                        break
                else:
                    env_off = env.copy()
                    env_off["DEPTHAI_AUTOCALIBRATION"] = "OFF"
                    env_off["AUTOCAL_BENCH_STRICT"] = "0"
                    proc_off = subprocess.run(args.off_command, shell=True, capture_output=True, text=True, env=env_off)
                    out_off = (proc_off.stdout or "") + (proc_off.stderr or "")
                    m_off = None
                    for mm in BATCH_RE.finditer(out_off):
                        m_off = mm

                    env_on = env.copy()
                    env_on["DEPTHAI_AUTOCALIBRATION"] = "ON_START"
                    env_on["AUTOCAL_BENCH_STRICT"] = "0"
                    proc_on = subprocess.run(args.on_command, shell=True, capture_output=True, text=True, env=env_on)
                    out_on = (proc_on.stdout or "") + (proc_on.stderr or "")
                    m_on = None
                    for mm in BATCH_RE.finditer(out_on):
                        m_on = mm

                    final_rc = 0 if (proc_off.returncode == 0 and proc_on.returncode == 0) else (proc_off.returncode or proc_on.returncode)
                    last_output = f"OFF output:\n{out_off}\n\nON output:\n{out_on}"

                    if proc_off.returncode == 0 and proc_on.returncode == 0 and m_off is not None and m_on is not None:
                        off = float(m_off.group(1))
                        on = float(m_on.group(1))
                        drop = off - on
                        drop_pct = (drop / off) * 100.0 if off > 0 else 0.0
                        ok = True
                        break

            if not ok:
                failures += 1
                writer.writerow([i, "", "", "", "", final_rc])
                print(f"[{i}/{args.runs}] FAILED rc={final_rc}")
                if args.mode == "parent":
                    print("  Could not find [AUTOCAL_COMPARE] line in output.")
                else:
                    print("  Could not parse [AUTOCAL_BENCH] avg_fps line in OFF/ON output.")
                if not args.continue_on_fail:
                    print("Stopping due to failure. Use --continue-on-fail to keep going.")
                    print("Last output tail:")
                    print(last_output[-2000:])
                    return 1
                continue

            off_vals.append(off)
            on_vals.append(on)
            drop_vals.append(drop)
            drop_pct_vals.append(drop_pct)
            writer.writerow([i, off, on, drop, drop_pct, 0])

            print(f"[{i}/{args.runs}] off={off:.4f} on={on:.4f} drop={drop:.4f} ({drop_pct:.4f}%)")

    print("\nSummary")
    print(summarize("OFF avg FPS", off_vals))
    print(summarize("ON avg FPS", on_vals))
    print(summarize("Drop FPS", drop_vals))
    print(summarize("Drop %", drop_pct_vals))
    print(f"Failures: {failures}")
    print(f"CSV: {csv_path}")

    if off_vals and on_vals and drop_vals:
        if plot_histograms(off_vals, on_vals, drop_vals, png_path):
            print(f"Histogram: {png_path}")
        else:
            print("Histogram skipped: matplotlib is not installed.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
