"""
tools/calibrate_gyro.py — Gyroscope ME calibration and DCD save.

Run when the gyro accuracy is 0 after power-on, after a temperature change,
or after a long idle period. Sensor must be completely still throughout.

Prerequisites — deploy drivers to Pico first:
  python tools/deploy.py

Usage:
  python tools/calibrate_gyro.py
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

imu.gyro.enable(20)
imu.game_quaternion.enable(20)  # fusion must run for ME to converge

imu.begin_gyro_calibration()

print("\\n=== Gyroscope Calibration ===")
print("  Monitoring accuracy (target: gyro >= 2)...\\n")

start_good = None
last_print  = ticks_ms()

while True:
    imu.update_sensors()
    if ticks_diff(ticks_ms(), last_print) < 200:
        continue
    last_print = ticks_ms()

    acc = imu.gyro.get().accuracy

    if acc >= 2:
        if start_good is None:
            start_good = ticks_ms()
            print(f"  Gyro={acc}  GOOD — holding {STABLE_SECS}s to confirm...")
        elapsed = ticks_diff(ticks_ms(), start_good) / 1000.0
        remaining = STABLE_SECS - elapsed
        print(f"  Gyro={acc}  {remaining:.1f}s remaining")
        if elapsed >= STABLE_SECS:
            break
    else:
        if start_good is not None:
            print("  Lost calibration — hold the device completely still.")
        start_good = None
        print(f"  Gyro={acc}  — keep still")

imu.save_calibration_data()
print("\\n  Calibration saved to BNO085 flash (DCD).")
"""

subprocess.run(
    [PYTHON, "-m", "mpremote", "connect", COM_PORT, "exec", PICO_CODE],
)
