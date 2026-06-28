"""
tools/read_gyro.py — Calibrated Gyroscope noise characterisation.

Pico streams raw angular rate components (rad/s); all unit conversion and
Welford statistics run on the host. Use this to measure gyro bias and noise
floor while the sensor is stationary.

Prerequisites — deploy drivers to Pico first:
  python -m mpremote connect COM7 cp src/bno08x.py :bno08x.py + cp src/i2c.py :i2c.py

Usage:
  python tools/read_gyro.py
  Ctrl+C to stop early (prints final stats before exit).
"""

import subprocess
import sys

PYTHON    = sys.executable
COM_PORT  = "COM7"
RATE_HZ   = 300
N_SAMPLES = 1000

# Pico code: init sensor, drain, then stream gx,gy,gz,accuracy as CSV lines (rad/s).
PICO_CODE = """\
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff
from bno08x import SensorResetError
from i2c import BNO08X_I2C

RATE_HZ  = 300
N        = 1000

i2c       = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
reset_pin = Pin(2, Pin.OUT)
int_pin   = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin)
imu.gyro.enable(RATE_HZ)
imu.game_quaternion.enable(20)  # fusion must run for ME to report accuracy
imu.begin_calibration()

start = ticks_ms()
while ticks_diff(ticks_ms(), start) < 800:
    imu.update_sensors()

print("READY")
cnt     = 0
last_ts = -1.0
while cnt < N:
    try:
        imu.update_sensors()
    except SensorResetError as e:
        print(f"ERROR:{e}")
        break
    g = imu.gyro.get()
    if g.sensor_ts_ms == 0.0 or g.sensor_ts_ms == last_ts:
        continue
    last_ts = g.sensor_ts_ms
    gx, gy, gz = g.data
    print(f"{gx},{gy},{gz},{g.accuracy}")
    cnt += 1
print("DONE")
"""

RAD2DEG = 57.2957795


def welford(n, mean, M2, x):
    n += 1
    d = x - mean
    mean += d / n
    M2 += d * (x - mean)
    return n, mean, M2


def std(n, M2):
    return (M2 / (n - 1)) ** 0.5 if n > 1 else 0.0


def print_stats(cnt, x_mean, x_M2, x_min, x_max,
                      y_mean, y_M2, y_min, y_max,
                      z_mean, z_M2, z_min, z_max):
    print(f"--- stats after {cnt} samples (°/s) ---")
    print(f"    gx: mean={x_mean:+8.5f}  std={std(cnt, x_M2):.5f}  [{x_min:+.5f}, {x_max:+.5f}]")
    print(f"    gy: mean={y_mean:+8.5f}  std={std(cnt, y_M2):.5f}  [{y_min:+.5f}, {y_max:+.5f}]")
    print(f"    gz: mean={z_mean:+8.5f}  std={std(cnt, z_M2):.5f}  [{z_min:+.5f}, {z_max:+.5f}]")


def main():
    proc = subprocess.Popen(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "exec", PICO_CODE],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )

    cnt = 0
    x_n = y_n = z_n = 0
    x_mean = x_M2 = y_mean = y_M2 = z_mean = z_M2 = 0.0
    x_min = y_min = z_min = 1e9
    x_max = y_max = z_max = -1e9

    print(f"  n    gx(°/s)    gy(°/s)    gz(°/s)   acc")
    print("-" * 48)

    try:
        for line in proc.stdout:
            line = line.rstrip()
            if line == "READY" or line == "DONE":
                continue
            if line.startswith("ERROR:"):
                print(f"SENSOR RESET: {line[6:]}")
                break

            parts = line.split(",")
            if len(parts) != 4:
                continue

            gx = float(parts[0]) * RAD2DEG
            gy = float(parts[1]) * RAD2DEG
            gz = float(parts[2]) * RAD2DEG
            acc = parts[3].strip()

            x_n, x_mean, x_M2 = welford(x_n, x_mean, x_M2, gx)
            y_n, y_mean, y_M2 = welford(y_n, y_mean, y_M2, gy)
            z_n, z_mean, z_M2 = welford(z_n, z_mean, z_M2, gz)
            x_min = min(x_min, gx); x_max = max(x_max, gx)
            y_min = min(y_min, gy); y_max = max(y_max, gy)
            z_min = min(z_min, gz); z_max = max(z_max, gz)
            cnt += 1

            print(f"{cnt:4d}  {gx:+9.5f}  {gy:+9.5f}  {gz:+9.5f}   {acc}")

            if cnt % 100 == 0:
                print_stats(cnt, x_mean, x_M2, x_min, x_max,
                                  y_mean, y_M2, y_min, y_max,
                                  z_mean, z_M2, z_min, z_max)
    except KeyboardInterrupt:
        proc.terminate()

    proc.wait()

    if cnt > 0 and cnt % 100 != 0:
        print()
        print_stats(cnt, x_mean, x_M2, x_min, x_max,
                         y_mean, y_M2, y_min, y_max,
                         z_mean, z_M2, z_min, z_max)


if __name__ == "__main__":
    main()
