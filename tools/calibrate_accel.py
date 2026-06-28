"""
tools/calibrate_accel.py — Accelerometer ME calibration and DCD save.

Run when the accel accuracy is 0, or after the sensor PCB was remounted or
its orientation changed. Requires rotating the sensor through 4–6 faces.

Prerequisites — deploy drivers to Pico first:
  python tools/deploy.py

Usage:
  python tools/calibrate_accel.py
"""

import subprocess
import sys

PYTHON   = sys.executable
COM_PORT = "COM7"

PICO_CODE = """\
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff
from i2c import BNO08X_I2C

STABLE_SECS = 5

i2c       = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
reset_pin = Pin(2, Pin.OUT)
int_pin   = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin)

imu.acceleration.enable(20)
imu.game_quaternion.enable(20)  # fusion must run for ME to converge

imu.begin_accel_calibration()

print("\\n=== Accelerometer Calibration ===")
print("  Rotate through 4-6 faces (~1 second each). Monitoring accuracy (target: accel >= 2)...\\n")

start_good = None
last_print  = ticks_ms()

while True:
    imu.update_sensors()
    if ticks_diff(ticks_ms(), last_print) < 200:
        continue
    last_print = ticks_ms()

    acc = imu.acceleration.get().accuracy

    if acc >= 2:
        if start_good is None:
            start_good = ticks_ms()
            print(f"  Accel={acc}  GOOD — holding {STABLE_SECS}s to confirm...")
        elapsed = ticks_diff(ticks_ms(), start_good) / 1000.0
        remaining = STABLE_SECS - elapsed
        print(f"  Accel={acc}  {remaining:.1f}s remaining")
        if elapsed >= STABLE_SECS:
            break
    else:
        if start_good is not None:
            print("  Lost calibration — keep rotating through more orientations.")
        start_good = None
        print(f"  Accel={acc}  — rotate to a new face")

imu.save_calibration_data()
print("\\n  Calibration saved to BNO085 flash (DCD).")
"""

subprocess.run(
    [PYTHON, "-m", "mpremote", "connect", COM_PORT, "exec", PICO_CODE],
)
