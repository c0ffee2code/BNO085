"""
tools/calibrate_mag.py — Magnetometer ME calibration and DCD save.

Run when moving to a new environment, after a power cycle near magnetic
interference, or when RV heading accuracy is poor. Requires rotating the
sensor freely through all axes.

Prerequisites — deploy drivers to Pico first:
  python tools/deploy.py

Usage:
  python tools/calibrate_mag.py
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

# Mag needs 50 Hz; Rotation Vector (full fusion with mag) must run alongside.
imu.magnetic.enable(50)
imu.quaternion.enable(20)

imu.begin_mag_calibration()

print("\\n=== Magnetometer Calibration ===")
print("  Rotate through roll/pitch/yaw axes (~180 deg each, figure-8 helps).")
print("  Monitoring accuracy (target: mag >= 2)...\\n")

start_good = None
last_print  = ticks_ms()

while True:
    imu.update_sensors()
    if ticks_diff(ticks_ms(), last_print) < 200:
        continue
    last_print = ticks_ms()

    acc = imu.magnetic.get().accuracy

    if acc >= 2:
        if start_good is None:
            start_good = ticks_ms()
            print(f"  Mag={acc}  GOOD — holding {STABLE_SECS}s to confirm...")
        elapsed = ticks_diff(ticks_ms(), start_good) / 1000.0
        remaining = STABLE_SECS - elapsed
        print(f"  Mag={acc}  {remaining:.1f}s remaining")
        if elapsed >= STABLE_SECS:
            break
    else:
        if start_good is not None:
            print("  Lost calibration — keep rotating the sensor.")
        start_good = None
        print(f"  Mag={acc}  — keep rotating (figure-8 motion helps)")

imu.save_calibration_data()
print("\\n  Calibration saved to BNO085 flash (DCD).")
print("  Magnetometer ready — you can now run the tare script.")
"""

subprocess.run(
    [PYTHON, "-m", "mpremote", "connect", COM_PORT, "exec", PICO_CODE],
)
