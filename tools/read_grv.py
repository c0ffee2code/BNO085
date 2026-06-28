"""
tools/read_grv.py — Game Rotation Vector noise characterisation.

Pico streams raw quaternion components; all math (Euler angles, Welford statistics)
runs on the host. Use this to measure attitude jitter while the sensor is stationary.

Prerequisites — deploy drivers to Pico first:
  python -m mpremote connect COM7 cp src/bno08x.py :bno08x.py + cp src/i2c.py :i2c.py

Usage:
  python tools/read_grv.py
  Ctrl+C to stop early (prints final stats before exit).
"""

import subprocess
import sys
from math import atan2, asin, sqrt

PYTHON    = sys.executable
COM_PORT  = "COM7"
RATE_HZ   = 100
N_SAMPLES = 1000

# Pico code: init sensor, drain, then stream qr,qi,qj,qk,accuracy as CSV lines.
PICO_CODE = """\
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff
from bno08x import SensorResetError
from i2c import BNO08X_I2C

RATE_HZ  = 100
N        = 1000

i2c       = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
reset_pin = Pin(2, Pin.OUT)
int_pin   = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin)
imu.game_quaternion.enable(RATE_HZ)

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
    gq = imu.game_quaternion.get()
    if gq.sensor_ts_ms == 0.0 or gq.sensor_ts_ms == last_ts:
        continue
    last_ts = gq.sensor_ts_ms
    qr, qi, qj, qk = gq.data
    print(f"{qr},{qi},{qj},{qk},{gq.accuracy}")
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


def print_stats(cnt, r_mean, r_M2, r_min, r_max,
                      p_mean, p_M2, p_min, p_max,
                      y_mean, y_M2, y_min, y_max, norm_mean):
    print(f"--- stats after {cnt} samples ---")
    print(f"    roll : mean={r_mean:+8.4f}°  std={std(cnt, r_M2):.4f}°  [{r_min:+.4f}, {r_max:+.4f}]")
    print(f"    pitch: mean={p_mean:+8.4f}°  std={std(cnt, p_M2):.4f}°  [{p_min:+.4f}, {p_max:+.4f}]")
    print(f"    yaw  : mean={y_mean:+8.4f}°  std={std(cnt, y_M2):.4f}°  [{y_min:+.4f}, {y_max:+.4f}]")
    print(f"    norm : mean={norm_mean:.6f}")


def main():
    proc = subprocess.Popen(
        [PYTHON, "-m", "mpremote", "connect", COM_PORT, "exec", PICO_CODE],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )

    cnt = 0
    r_n = p_n = y_n = 0
    r_mean = r_M2 = p_mean = p_M2 = y_mean = y_M2 = 0.0
    r_min = p_min = y_min = 1e9
    r_max = p_max = y_max = -1e9
    norm_sum = 0.0

    print(f"  n    qr       qi       qj       qk    norm   roll(°)  pitch(°)   yaw(°)  acc")
    print("-" * 86)

    try:
        for line in proc.stdout:
            line = line.rstrip()
            if line == "READY" or line == "DONE":
                continue
            if line.startswith("ERROR:"):
                print(f"SENSOR RESET: {line[6:]}")
                break

            parts = line.split(",")
            if len(parts) != 5:
                continue

            qr, qi, qj, qk, acc = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), parts[4].strip()
            norm  = sqrt(qr*qr + qi*qi + qj*qj + qk*qk)
            roll  = atan2(2*(qr*qi + qj*qk), 1 - 2*(qi*qi + qj*qj)) * RAD2DEG
            pitch = asin(max(-1.0, min(1.0, 2*(qr*qj - qk*qi))))    * RAD2DEG
            yaw   = atan2(2*(qr*qk + qi*qj), 1 - 2*(qj*qj + qk*qk)) * RAD2DEG

            r_n, r_mean, r_M2 = welford(r_n, r_mean, r_M2, roll)
            p_n, p_mean, p_M2 = welford(p_n, p_mean, p_M2, pitch)
            y_n, y_mean, y_M2 = welford(y_n, y_mean, y_M2, yaw)
            r_min = min(r_min, roll);  r_max = max(r_max, roll)
            p_min = min(p_min, pitch); p_max = max(p_max, pitch)
            y_min = min(y_min, yaw);   y_max = max(y_max, yaw)
            norm_sum += norm
            cnt += 1

            print(f"{cnt:4d}  {qr:+7.4f}  {qi:+7.4f}  {qj:+7.4f}  {qk:+7.4f}  {norm:.5f}"
                  f"  {roll:+7.3f}  {pitch:+7.3f}  {yaw:+8.3f}  {acc}")

            if cnt % 100 == 0:
                print_stats(cnt, r_mean, r_M2, r_min, r_max,
                                  p_mean, p_M2, p_min, p_max,
                                  y_mean, y_M2, y_min, y_max, norm_sum / cnt)
    except KeyboardInterrupt:
        proc.terminate()

    proc.wait()

    if cnt > 0 and cnt % 100 != 0:
        print()
        print_stats(cnt, r_mean, r_M2, r_min, r_max,
                         p_mean, p_M2, p_min, p_max,
                         y_mean, y_M2, y_min, y_max, norm_sum / cnt)


if __name__ == "__main__":
    main()
