"""
tools/noise_floor.py -- BNO085 GRV + gyro noise characterisation

Two subcommands:

  capture   -- stream sensor data from Pico to a timestamped CSV
  analyse   -- report noise floor, accuracy, lag, and formula comparison

Usage:
  python tools/noise_floor.py capture                  # begin_calibration() active
  python tools/noise_floor.py capture --no-calibration # ME inactive, accuracy stays 0

  python tools/noise_floor.py analyse tools/captures/<file>.csv

Captures are saved under tools/captures/ as pure CSV (header + data, no metadata rows).
The filename encodes the condition: noise_YYYY-MM-DD_HH-MM-SS_cal.csv or _nocal.csv

Prerequisites -- deploy drivers to Pico:
  python -m mpremote connect COM7 cp src/bno08x.py :bno08x.py + cp src/i2c.py :i2c.py
"""

import argparse
import math
import subprocess
import sys
from datetime import datetime
from pathlib import Path

# ---- shared constants --------------------------------------------------------

PYTHON       = sys.executable
COM_PORT     = "COM7"
GRV_HZ       = 100
GYRO_HZ      = 300
DURATION_S   = 120
RAD2DEG      = 57.2957795
CSV_HEADER   = "T_MS,QR,QI,QJ,QK,GRV_ACC,GRV_LAG_MS,GYRO_X,GYRO_ACC,GYRO_LAG_MS"
CAPTURES_DIR = Path(__file__).parent / "captures"

# ---- Pico-side code ----------------------------------------------------------

_PICO_TEMPLATE = """\
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff
from bno08x import SensorResetError
from i2c import BNO08X_I2C

GRV_HZ      = {grv_hz}
GYRO_HZ     = {gyro_hz}
DURATION_MS = {duration_ms}

i2c       = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
reset_pin = Pin(2, Pin.OUT)
int_pin   = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin)

imu.game_quaternion.enable(GRV_HZ)
imu.gyro.enable(GYRO_HZ)
{cal_line}

start = ticks_ms()
while ticks_diff(ticks_ms(), start) < 500:
    imu.update_sensors()

print("{csv_header}")
print("READY")

run_start    = ticks_ms()
last_gyro_ts = 0.0

while ticks_diff(ticks_ms(), run_start) < DURATION_MS:
    try:
        if imu.update_sensors() == 0:
            continue
    except SensorResetError as e:
        print("ERROR:" + str(e))
        break
    g = imu.gyro.get()
    if g.sensor_ts_ms == last_gyro_ts:
        continue
    last_gyro_ts = g.sensor_ts_ms
    now_ms   = ticks_diff(ticks_ms(), run_start)
    grv      = imu.game_quaternion.get()
    grv_lag  = imu.bno_start_diff(grv.host_ts_ms) - grv.sensor_ts_ms
    gyro_lag = imu.bno_start_diff(g.host_ts_ms)   - g.sensor_ts_ms
    print("%d,%.6f,%.6f,%.6f,%.6f,%d,%.2f,%.6f,%d,%.2f" % (
        now_ms,
        grv.data[0], grv.data[1], grv.data[2], grv.data[3],
        grv.accuracy, grv_lag,
        g.data[0], g.accuracy, gyro_lag))

print("DONE")
"""


def _build_pico_code(use_calibration):
    cal_line = ("imu.begin_calibration()"
                if use_calibration
                else "# begin_calibration() skipped -- ME inactive, accuracy stays 0")
    return _PICO_TEMPLATE.format(
        grv_hz=GRV_HZ,
        gyro_hz=GYRO_HZ,
        duration_ms=DURATION_S * 1000,
        cal_line=cal_line,
        csv_header=CSV_HEADER,
    )


# ---- Welford online stats ----------------------------------------------------

def _welford(n, mean, m2, x):
    n += 1
    d = x - mean
    mean += d / n
    m2 += d * (x - mean)
    return n, mean, m2


def _std(n, m2):
    return math.sqrt(m2 / (n - 1)) if n > 1 else 0.0


# ---- capture subcommand ------------------------------------------------------

def cmd_capture(args):
    use_calibration = not args.no_calibration
    suffix    = "cal" if use_calibration else "nocal"
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
    out_path  = CAPTURES_DIR / f"noise_{timestamp}_{suffix}.csv"

    print(f"Capturing: calibration={'ON' if use_calibration else 'OFF'}, "
          f"GRV {GRV_HZ} Hz, gyro {GYRO_HZ} Hz, {DURATION_S} s")
    print(f"Output: {out_path}")
    print()
    print(f"{'t(s)':>7}  {'n':>6}  {'Hz':>5}  {'GRV_acc':>7}  "
          f"{'roll(deg)':>10}  {'roll_std(deg)':>13}")
    print("-" * 62)

    pico_code = _build_pico_code(use_calibration)
    proc = subprocess.Popen(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "exec", pico_code],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )

    n = 0
    roll_n, roll_mean, roll_m2 = 0, 0.0, 0.0
    last_print_n = 0
    PRINT_EVERY  = max(1, GYRO_HZ * 10)

    try:
        with out_path.open("w", newline="") as f:
            for raw in proc.stdout:
                line = raw.rstrip()
                if line == CSV_HEADER:
                    f.write(line + "\n")
                    continue
                if line in ("READY", "DONE"):
                    continue
                if line.startswith("ERROR:"):
                    print(f"\nSENSOR RESET: {line[6:]}")
                    break
                parts = line.split(",")
                if len(parts) != 10:
                    continue

                f.write(line + "\n")
                n += 1

                qr   = float(parts[1])
                qi   = float(parts[2])
                roll = 2.0 * math.atan2(qi, qr) * RAD2DEG
                grv_acc = int(parts[5])
                t_s  = int(parts[0]) / 1000.0

                roll_n, roll_mean, roll_m2 = _welford(roll_n, roll_mean, roll_m2, roll)

                if n - last_print_n >= PRINT_EVERY:
                    hz = n / t_s if t_s > 0 else 0.0
                    print(f"{t_s:7.1f}  {n:6d}  {hz:5.1f}  {grv_acc:>7}  "
                          f"{roll_mean:10.4f}  {_std(roll_n, roll_m2):13.5f}")
                    last_print_n = n

    except KeyboardInterrupt:
        proc.terminate()

    proc.wait()
    print(f"\nDone: {n} samples -> {out_path.name}")
    return out_path


# ---- analyse subcommand ------------------------------------------------------

def _load_csv(path):
    rows   = []
    header = None
    with open(path, newline="") as f:
        for line in f:
            line = line.rstrip()
            if not line or line.startswith("#"):
                continue
            if header is None:
                header = line.split(",")
                continue
            parts = line.split(",")
            if len(parts) == len(header):
                rows.append(parts)

    if not rows:
        sys.exit(f"No data rows in {path}")

    try:
        import numpy as np
    except ImportError:
        sys.exit("numpy is required: pip install numpy")

    cols = {}
    for j, h in enumerate(header):
        cols[h] = np.array([float(row[j]) for row in rows])
    return cols


def _roll_simple(qr, qi):
    import numpy as np
    return 2.0 * np.arctan2(qi, qr) * RAD2DEG


def _roll_full(qr, qi, qj, qk):
    import numpy as np
    return np.degrees(np.arctan2(2.0 * (qr * qi + qj * qk),
                                 1.0 - 2.0 * (qi ** 2 + qj ** 2)))


def cmd_analyse(args):
    import numpy as np

    csv_path = Path(args.csv)
    if not csv_path.exists():
        sys.exit(f"File not found: {csv_path}")

    cols = _load_csv(csv_path)
    n    = len(cols["T_MS"])
    t_ms = cols["T_MS"]
    qr, qi, qj, qk = cols["QR"], cols["QI"], cols["QJ"], cols["QK"]
    grv_acc  = cols["GRV_ACC"].astype(int)
    grv_lag  = cols["GRV_LAG_MS"]
    gyro_x   = cols["GYRO_X"] * RAD2DEG
    gyro_acc = cols["GYRO_ACC"].astype(int)
    gyro_lag = cols["GYRO_LAG_MS"]

    duration_s = (t_ms[-1] - t_ms[0]) / 1000.0
    actual_hz  = (n - 1) / duration_s if duration_s > 0 else 0.0

    print(f"File     : {csv_path.name}")
    print(f"Samples  : {n}  ({actual_hz:.1f} Hz effective, {duration_s:.1f} s)")

    r_simple   = _roll_simple(qr, qi)
    r_full     = _roll_full(qr, qi, qj, qk)
    divergence = r_full - r_simple

    print()
    print("=== Roll noise floor ===")
    for label, r in [("simplified  2*atan2(qi,qr)", r_simple),
                     ("full Tait-Bryan            ", r_full)]:
        print(f"  {label}")
        print(f"    mean  {r.mean():+9.5f} deg")
        print(f"    std   {r.std():9.5f} deg")
        print(f"    p95   {np.percentile(np.abs(r - r.mean()), 95):9.5f} deg")
    print(f"  Formula divergence (full - simplified)")
    print(f"    mean  {divergence.mean():+9.5f} deg")
    print(f"    std   {divergence.std():9.5f} deg")

    print()
    print("=== GRV accuracy ===")
    for level in range(4):
        print(f"  acc={level}: {np.mean(grv_acc == level) * 100:5.1f}% of samples")
    idx3 = np.where(grv_acc >= 3)[0]
    if len(idx3):
        print(f"  First acc=3 at t={t_ms[idx3[0]] / 1000.0:.2f} s")
    else:
        print(f"  acc=3 never reached")

    print()
    print("=== Transport lag ===")
    for label, lag in [("GRV lag ", grv_lag), ("Gyro lag", gyro_lag)]:
        print(f"  {label}  mean={lag.mean():.2f} ms  std={lag.std():.2f} ms  "
              f"p95={np.percentile(lag, 95):.2f} ms  max={lag.max():.2f} ms")

    print()
    print("=== Gyro X ===")
    print(f"  bias (mean)  {gyro_x.mean():+9.5f} deg/s")
    print(f"  noise (std)  {gyro_x.std():9.5f} deg/s")
    print(f"  gyro acc     mode={int(np.bincount(gyro_acc).argmax())}  mean={gyro_acc.mean():.2f}")

    # plots
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib not found -- skipping plots")
        return

    t_s  = t_ms / 1000.0
    stem = csv_path.stem

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"BNO085 noise capture -- {stem}")
    axes[0].plot(t_s, r_simple, lw=0.5, label="simplified")
    axes[0].plot(t_s, r_full,   lw=0.5, alpha=0.7, label="full Tait-Bryan")
    axes[0].set_ylabel("roll (deg)")
    axes[0].legend(fontsize=8)
    axes[0].set_title(f"Roll  std={r_simple.std():.5f} deg")
    axes[1].plot(t_s, divergence, lw=0.5, color="purple")
    axes[1].axhline(divergence.mean(), color="purple", lw=0.8, ls="--")
    axes[1].set_ylabel("full - simple (deg)")
    axes[1].set_title(f"Formula divergence  mean={divergence.mean():+.5f} deg  std={divergence.std():.5f} deg")
    axes[2].plot(t_s, grv_acc, lw=0.8, drawstyle="steps-post", color="green")
    axes[2].set_ylabel("GRV acc")
    axes[2].set_ylim(-0.2, 3.5)
    axes[2].set_xlabel("time (s)")
    plt.tight_layout()
    roll_plot = csv_path.with_name(stem + "_roll.png")
    fig.savefig(roll_plot, dpi=120)
    plt.close(fig)
    print(f"\nPlot: {roll_plot}")

    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    fig.suptitle(f"BNO085 lag and gyro noise -- {stem}")
    axes[0].hist(grv_lag,  bins=40, edgecolor="none", color="steelblue")
    axes[0].set_xlabel("GRV lag (ms)")
    axes[0].set_title(f"GRV lag  mean={grv_lag.mean():.2f} ms")
    axes[1].hist(gyro_lag, bins=40, edgecolor="none", color="orange")
    axes[1].set_xlabel("Gyro lag (ms)")
    axes[1].set_title(f"Gyro lag  mean={gyro_lag.mean():.2f} ms")
    axes[2].hist(gyro_x,   bins=60, edgecolor="none", color="gray")
    axes[2].set_xlabel("Gyro X (deg/s)")
    axes[2].set_title(f"Gyro X  bias={gyro_x.mean():+.5f}  std={gyro_x.std():.5f} deg/s")
    plt.tight_layout()
    lag_plot = csv_path.with_name(stem + "_lag.png")
    fig.savefig(lag_plot, dpi=120)
    plt.close(fig)
    print(f"Plot: {lag_plot}")


# ---- entry point -------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="BNO085 noise floor characterisation tool.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_cap = sub.add_parser("capture", help="Stream sensor data from Pico to CSV")
    p_cap.add_argument("--no-calibration", action="store_true",
                       help="Skip begin_calibration() -- ME inactive, gyro acc stays 0")

    p_an = sub.add_parser("analyse", help="Analyse a captured CSV")
    p_an.add_argument("csv", help="Path to CSV file from the capture subcommand")

    args = parser.parse_args()
    if args.cmd == "capture":
        cmd_capture(args)
    else:
        cmd_analyse(args)


if __name__ == "__main__":
    main()