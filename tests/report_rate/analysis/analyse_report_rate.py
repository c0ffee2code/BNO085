"""
Analyse BNO085 rotation vector vs AS5600 encoder CSV data at different report rates.

Reads CSV files with columns: T (ms), ENC (encoder angle deg), IMU (imu angle deg), Lag (ms)
Computes tracking quality metrics and prints a comparison table.

Usage:
    python analyse_report_rate.py <file.csv>                    # single file stats
    python analyse_report_rate.py <file1.csv> <file2.csv>       # side-by-side comparison


Methodology
-----------

The AS5600 magnetic encoder is treated as ground truth — it measures the lever's
angle directly from the magnet on the rotation axis, with no filtering or fusion.
Each CSV row is a simultaneous snapshot: the test script reads the encoder, checks
the IMU, and logs both together.

Metrics:

  Lag — transport/processing delay reported by the Pico (time between the current
  loop iteration and the IMU report's internal timestamp). This is NOT angular phase
  lag, just how stale the IMU reading is when we consume it.

  Angle error (MAE, RMS, Max) — per-sample |ENC - IMU|. This is the total error
  the flight controller would see, combining systematic bias and dynamic tracking
  error. We intentionally do not subtract the bias before computing error, because
  pre-calibration accuracy is what we're evaluating. MAE treats all errors equally;
  RMS penalises large spikes more heavily.

  Bias — mean of (ENC - IMU), signed. A consistent non-zero value means the IMU has
  a fixed offset from the encoder, removable by tare() calibration.

  Pearson correlation — how well the two signals move together, ignoring offset and
  scale. Separates "does the IMU follow the shape of the motion" from "is it offset
  by a constant". High correlation + high MAE = good tracking but needs calibration.

  Fast/slow motion MAE — we estimate angular velocity from consecutive encoder
  readings, rank all samples by speed, then compute MAE separately for the fastest
  25% and slowest 25%. This answers whether the IMU tracks better during rapid
  manoeuvres or gentle motion — critical for choosing a report rate.

  Trail percentage — for each sample where the encoder is actually moving (>1 deg
  change), we check whether the IMU is "behind" the encoder in the direction of
  motion. Above 50% means the IMU systematically lags behind direction changes.

  Range — total angular excursion (max - min) for encoder vs IMU. If the IMU range
  is smaller, its internal sensor fusion filter is smoothing out peak excursions.
"""

import csv
import math
import sys
from pathlib import Path


def load_csv(path):
    """Load CSV with columns T, ENC, IMU, Lag into list of dicts with float values."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({
                "T": float(r["T"]),
                "ENC": float(r["ENC"]),
                "IMU": float(r["IMU"]),
                "Lag": float(r["Lag"]),
            })
    return rows


def compute_stats(rows):
    """Compute all metrics for a single dataset."""
    n = len(rows)
    if n < 2:
        return None

    # --- Lag stats ---
    lags = [r["Lag"] for r in rows]
    lag_mean = sum(lags) / n
    lag_sorted = sorted(lags)
    lag_median = lag_sorted[n // 2] if n % 2 else (lag_sorted[n // 2 - 1] + lag_sorted[n // 2]) / 2
    lag_min = min(lags)
    lag_max = max(lags)
    lag_std = math.sqrt(sum((l - lag_mean) ** 2 for l in lags) / n)

    # --- Angle error ---
    abs_errors = [abs(r["ENC"] - r["IMU"]) for r in rows]
    mae = sum(abs_errors) / n
    abs_errors_sorted = sorted(abs_errors)
    median_ae = abs_errors_sorted[n // 2]
    max_ae = max(abs_errors)
    rms_error = math.sqrt(sum(e ** 2 for e in abs_errors) / n)

    # --- Bias ---
    diffs = [r["ENC"] - r["IMU"] for r in rows]
    bias = sum(diffs) / n

    # --- Pearson correlation ---
    enc_vals = [r["ENC"] for r in rows]
    imu_vals = [r["IMU"] for r in rows]
    enc_mean = sum(enc_vals) / n
    imu_mean = sum(imu_vals) / n
    cov = sum((e - enc_mean) * (i - imu_mean) for e, i in zip(enc_vals, imu_vals)) / n
    enc_std = math.sqrt(sum((e - enc_mean) ** 2 for e in enc_vals) / n)
    imu_std = math.sqrt(sum((i - imu_mean) ** 2 for i in imu_vals) / n)
    correlation = cov / (enc_std * imu_std) if enc_std > 0 and imu_std > 0 else 0.0

    # --- Sample rate ---
    duration_s = (rows[-1]["T"] - rows[0]["T"]) / 1000.0
    actual_hz = (n - 1) / duration_s if duration_s > 0 else 0
    dts = [rows[i + 1]["T"] - rows[i]["T"] for i in range(n - 1)]
    dt_mean = sum(dts) / len(dts)
    dts_sorted = sorted(dts)
    dt_median = dts_sorted[len(dts) // 2]

    # --- Angular velocity (deg/s) for fast/slow motion split ---
    velocities = []
    for i in range(1, n):
        dt_s = (rows[i]["T"] - rows[i - 1]["T"]) / 1000.0
        if dt_s > 0:
            vel = abs(rows[i]["ENC"] - rows[i - 1]["ENC"]) / dt_s
            velocities.append((i, vel))

    velocities_sorted = sorted(velocities, key=lambda x: x[1])
    quarter = len(velocities_sorted) // 4

    slow_indices = {v[0] for v in velocities_sorted[:quarter]}
    fast_indices = {v[0] for v in velocities_sorted[-quarter:]}

    fast_errors = [abs(rows[i]["ENC"] - rows[i]["IMU"]) for i in fast_indices]
    slow_errors = [abs(rows[i]["ENC"] - rows[i]["IMU"]) for i in slow_indices]
    mae_fast = sum(fast_errors) / len(fast_errors) if fast_errors else 0
    mae_slow = sum(slow_errors) / len(slow_errors) if slow_errors else 0

    # --- Motion tracking direction (does IMU trail encoder?) ---
    trail_count = 0
    lead_count = 0
    for i in range(1, n):
        motion_dir = rows[i]["ENC"] - rows[i - 1]["ENC"]
        imu_offset = rows[i]["ENC"] - rows[i]["IMU"]
        if abs(motion_dir) > 1.0:  # ignore near-stationary
            if motion_dir * imu_offset > 0:
                trail_count += 1
            else:
                lead_count += 1
    total_motion = trail_count + lead_count
    trail_pct = (trail_count / total_motion * 100) if total_motion > 0 else 0

    # --- Range ---
    enc_range = max(enc_vals) - min(enc_vals)
    imu_range = max(imu_vals) - min(imu_vals)

    return {
        "n_samples": n,
        "duration_s": duration_s,
        "actual_hz": actual_hz,
        "dt_mean_ms": dt_mean,
        "dt_median_ms": dt_median,
        "lag_mean": lag_mean,
        "lag_median": lag_median,
        "lag_min": lag_min,
        "lag_max": lag_max,
        "lag_std": lag_std,
        "mae": mae,
        "median_ae": median_ae,
        "max_ae": max_ae,
        "rms_error": rms_error,
        "bias": bias,
        "correlation": correlation,
        "mae_fast": mae_fast,
        "mae_slow": mae_slow,
        "trail_pct": trail_pct,
        "enc_range": enc_range,
        "imu_range": imu_range,
    }


def print_comparison(label_a, stats_a, label_b, stats_b):
    """Print side-by-side comparison table."""

    def row(name, key, fmt=".2f"):
        va = stats_a[key]
        vb = stats_b[key]
        print(f"  {name:<32} {va:>10{fmt}}   {vb:>10{fmt}}")

    w = 60
    print("=" * w)
    print(f"  {'Metric':<32} {label_a:>10}   {label_b:>10}")
    print("-" * w)

    print("\n  --- Sample Rate ---")
    row("Samples", "n_samples", ".0f")
    row("Duration (s)", "duration_s", ".1f")
    row("Achieved Hz", "actual_hz", ".1f")
    row("Mean dt (ms)", "dt_mean_ms", ".1f")
    row("Median dt (ms)", "dt_median_ms", ".1f")

    print("\n  --- Lag (ms) ---")
    row("Mean", "lag_mean", ".2f")
    row("Median", "lag_median", ".1f")
    row("Min", "lag_min", ".1f")
    row("Max", "lag_max", ".1f")
    row("Std Dev", "lag_std", ".2f")

    print("\n  --- Angle Error (deg) ---")
    row("MAE (overall)", "mae", ".2f")
    row("MAE (fast motion)", "mae_fast", ".2f")
    row("MAE (slow motion)", "mae_slow", ".2f")
    row("Median AE", "median_ae", ".2f")
    row("Max AE", "max_ae", ".2f")
    row("RMS Error", "rms_error", ".2f")
    row("Bias (ENC-IMU)", "bias", ".2f")

    print("\n  --- Correlation & Tracking ---")
    row("Pearson r", "correlation", ".4f")
    row("IMU trails motion (%)", "trail_pct", ".1f")
    row("Encoder range (deg)", "enc_range", ".1f")
    row("IMU range (deg)", "imu_range", ".1f")

    print("=" * w)


def print_single(label, stats):
    """Print stats for a single dataset."""
    w = 50
    print("=" * w)
    print(f"  {label}")
    print("-" * w)
    for key, val in stats.items():
        if isinstance(val, float):
            print(f"  {key:<32} {val:>10.3f}")
        else:
            print(f"  {key:<32} {val:>10}")
    print("=" * w)


def main():
    if len(sys.argv) < 2:
        print("Usage: python analyse_report_rate.py <file.csv> [file2.csv]")
        sys.exit(1)

    files = [Path(f) for f in sys.argv[1:]]
    datasets = []
    for f in files:
        rows = load_csv(f)
        datasets.append((f.stem, compute_stats(rows)))
        print(f"Loaded {f.name}: {len(rows)} samples")

    print()

    if len(datasets) == 2:
        print_comparison(datasets[0][0], datasets[0][1], datasets[1][0], datasets[1][1])
    else:
        for label, stats in datasets:
            print_single(label, stats)


if __name__ == "__main__":
    main()
