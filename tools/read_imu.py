"""
tools/read_imu.py — Stream live BNO085 readings to terminal via mpremote REPL.

Prints game quaternion (GRV 0x08, 4-tuple) plus heading accuracy from
rotation vector (RV 0x05, 5-tuple) side by side.  Good for verifying:
  - quaternion norm ≈ 1.0  (Q-point sanity)
  - heading accuracy field present  (5-tuple fix)
  - Euler angles look correct for bench orientation

Prerequisites — deploy drivers to Pico first:
  python -m mpremote connect COM7 cp src/bno08x.py :bno08x.py + cp src/i2c.py :i2c.py

Usage:
  python tools/read_imu.py
  Ctrl+C to stop.
"""

import subprocess
import sys

PYTHON   = sys.executable
COM_PORT = "COM7"   # adjust: python -m mpremote connect list
RATE_HZ  = 50       # sensor report rate (Hz)

PICO_CODE = """\
from machine import I2C, Pin
from math import atan2, asin, sqrt
from utime import ticks_ms, ticks_diff
from i2c import BNO08X_I2C

RATE_HZ = 50

i2c       = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
reset_pin = Pin(2, Pin.OUT)
int_pin   = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin)

imu.game_quaternion.enable(RATE_HZ)
imu.quaternion.enable(RATE_HZ)

# drain initial packets so both features have received their first response
start = ticks_ms()
while ticks_diff(ticks_ms(), start) < 800:
    imu.update_sensors()

print("  n    qr      qi      qj      qk    norm   roll   pitch    yaw   hacc   acc")
print("-" * 82)

RAD2DEG = 57.2957795
last_ts = -1.0
n = 0

while n < 2000:
    imu.update_sensors()
    gq = imu.game_quaternion.get()
    if gq.sensor_ts_ms == 0.0 or gq.sensor_ts_ms == last_ts:
        continue
    last_ts = gq.sensor_ts_ms

    qr, qi, qj, qk = gq.data
    norm  = sqrt(qr*qr + qi*qi + qj*qj + qk*qk)
    roll  = atan2(2*(qr*qi + qj*qk), 1 - 2*(qi*qi + qj*qj)) * RAD2DEG
    pitch = asin(max(-1.0, min(1.0, 2*(qr*qj - qk*qi))))    * RAD2DEG
    yaw   = atan2(2*(qr*qk + qi*qj), 1 - 2*(qj*qj + qk*qk)) * RAD2DEG

    rv = imu.quaternion.get()
    hacc_deg = rv.data[4] * RAD2DEG if rv.sensor_ts_ms > 0.0 else -1.0

    print(f"{n:3d}  {qr:6.3f}  {qi:6.3f}  {qj:6.3f}  {qk:6.3f}  {norm:.4f}"
          f"  {roll:5.1f}  {pitch:5.1f}  {yaw:6.1f}  {hacc_deg:5.1f}  {gq.accuracy}")
    n += 1
"""

try:
    subprocess.run([PYTHON, "-m", "mpremote", "connect", COM_PORT, "exec", PICO_CODE])
except KeyboardInterrupt:
    pass
