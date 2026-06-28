"""
tools/tare.py — Tare the BNO085 to physical zero using a levelling jig.

Two-phase procedure:

  Phase 1 — Mount + settle
    Mount the sensor in the levelling jig at the zero position.
    Wait SETTLE_SECS for Game Rotation Vector to stabilise, then
    show pre-tare roll/pitch/yaw so you can confirm the bias.

  Phase 2 — Apply tare + verify
    Tare is applied (all axes, Game Rotation Vector basis).
    Post-tare readings are shown — roll/pitch/yaw should be ~0°.
    Optionally persist the tare offset to BNO085 flash.

Why GRV basis (basis=1):
  imu.tare(axes, basis=1) targets the Game Rotation Vector, driven by
  accel + gyro only — no magnetometer. Using basis=0 (Rotation Vector)
  bakes in the current mag heading; if mag accuracy is 0 on the bench
  (near motors, metal structures) the tare produces a ~45° tilted frame.
  Observed and documented in decision/004-sensor-calibration.md.

Prerequisites — deploy drivers to Pico first:
  python tools/deploy.py

Usage:
  python tools/tare.py
"""

import subprocess
import sys

PYTHON   = sys.executable
COM_PORT = "COM7"

PICO_CODE = """\
from machine import I2C, Pin
from math import atan2, asin
from utime import ticks_ms, ticks_diff, sleep_ms
from i2c import BNO08X_I2C
import select, sys as _sys

RATE_HZ     = 344
SETTLE_SECS = 15
RAD2DEG     = 57.2957795

i2c       = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
reset_pin = Pin(2, Pin.OUT)
int_pin   = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin)


def euler(qr, qi, qj, qk):
    roll  = atan2(2*(qr*qi + qj*qk), 1 - 2*(qi*qi + qj*qj)) * RAD2DEG
    pitch = asin(max(-1.0, min(1.0, 2*(qr*qj - qk*qi))))    * RAD2DEG
    yaw   = atan2(2*(qr*qk + qi*qj), 1 - 2*(qj*qj + qk*qk)) * RAD2DEG
    return roll, pitch, yaw


def wait_for_enter(msg):
    print(msg)
    while True:
        imu.update_sensors()
        if select.select([_sys.stdin], [], [], 0)[0]:
            _sys.stdin.readline()
            return


# =========================================================================
# Phase 1 — Mount + settle
# =========================================================================
wait_for_enter(
    "\\n=== Tare Procedure ===\\n"
    "\\nPhase 1: Mount sensor in the levelling jig at zero position."
    "\\nPress ENTER when mounted and stable.")

imu.game_quaternion.enable(RATE_HZ)

print(f"\\nSettling {SETTLE_SECS}s — keep the sensor still.\\n")

settle_start = ticks_ms()
last_print   = ticks_ms()
last_ts      = -1.0

while ticks_diff(ticks_ms(), settle_start) < SETTLE_SECS * 1000:
    imu.update_sensors()
    gq = imu.game_quaternion.get()
    if gq.sensor_ts_ms == last_ts or ticks_diff(ticks_ms(), last_print) < 500:
        continue
    last_ts    = gq.sensor_ts_ms
    last_print = ticks_ms()
    roll, pitch, yaw = euler(*gq.data)
    remaining = (SETTLE_SECS * 1000 - ticks_diff(ticks_ms(), settle_start)) / 1000
    print(f"  roll={roll:+.3f}  pitch={pitch:+.3f}  yaw={yaw:+.3f}  acc={gq.accuracy}  ({remaining:.0f}s)")

# Final pre-tare reading
imu.update_sensors()
gq = imu.game_quaternion.get()
roll_pre, pitch_pre, yaw_pre = euler(*gq.data)
print(f"\\nPre-tare  roll={roll_pre:+.3f}°  pitch={pitch_pre:+.3f}°  yaw={yaw_pre:+.3f}°")

# =========================================================================
# Phase 2 — Apply tare + verify
# =========================================================================
print("\\nApplying tare (all axes, GRV basis — no magnetometer required)...")
imu.tare(0x07, 1)

# Drain packets the tare command may have triggered.
for _ in range(50):
    imu.update_sensors()
    sleep_ms(10)

print("Tare applied. Post-tare readings:\\n")
last_ts = -1.0
shown   = 0
while shown < 5:
    imu.update_sensors()
    gq = imu.game_quaternion.get()
    if gq.sensor_ts_ms == last_ts:
        continue
    last_ts = gq.sensor_ts_ms
    roll, pitch, yaw = euler(*gq.data)
    print(f"  roll={roll:+.3f}°  pitch={pitch:+.3f}°  yaw={yaw:+.3f}°  acc={gq.accuracy}")
    shown += 1

# =========================================================================
# Persist tare
# =========================================================================
print("\\nSave tare to BNO085 flash? [y/N]: ", end="")
resp = input().strip().lower()
if resp == "y":
    imu.save_tare_data()
    print("Tare saved — survives power cycle.")
else:
    print("Tare NOT saved — lost on power cycle.")
"""

subprocess.run(
    [PYTHON, "-m", "mpremote", "connect", COM_PORT, "exec", PICO_CODE],
)
