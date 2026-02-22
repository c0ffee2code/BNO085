"""
Magnetometer-only calibration

Run this when the magnetometer needs recalibration — e.g. after moving to a
new room, after a power cycle near magnetic interference, or when the tare
script shows accuracy < 2.

Procedure:
  1. Move away from magnetic interferers (PC towers, monitors, metal furniture).
  2. Run this script.
  3. Rotate the device ~180 degrees and back on each axis:
       - Roll  (twist left-right)
       - Pitch (tilt forward-back)
       - Yaw   (turn like a compass / figure-8)
     Speed: ~2 seconds per rotation. Repeat a few times.
  4. Script monitors accuracy and saves DCD once mag >= 2.

Note: Magnetometer calibrates for the LOCAL magnetic field. Recalibrate any
time the device moves to a significantly different environment.

Accuracy scale: 0=Unreliable  1=Low  2=Medium (target)  3=High

References:
  - specification/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf (Section 2.2)
  - SH-2 Reference Manual §6.4.7.1 (Configure ME Calibration, P2=Mag)
  - BNO085 Tare Function Usage Guide p2 (mag accuracy prerequisite for tare)
"""

from machine import I2C, Pin
from utime import ticks_ms, ticks_diff, sleep_ms
from i2c import BNO08X_I2C

STABLE_SECS = 5

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
reset_pin = Pin(2, Pin.OUT)
int_pin = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin, debug=False)

# Magnetometer needs 50 Hz for proper calibration (per spec Section 2.2).
# Rotation Vector (full fusion with mag) must run alongside.
imu.magnetic.enable(50)
imu.quaternion.enable(20)

imu.begin_mag_calibration()
imu.calibration_status()


def wait_for_enter(message):
    import select, sys
    print(message)
    while True:
        imu.update_sensors()
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return


print("\n=== Magnetometer Calibration ===\n")

wait_for_enter(
    "  Rotate the device ~180 degrees and back on each axis:\n"
    "    - Roll  (twist left-right)\n"
    "    - Pitch (tilt forward-back)\n"
    "    - Yaw   (turn like a compass / figure-8)\n"
    "  Speed: ~2 seconds per rotation. Repeat a few times.\n"
    "  Stay away from metal furniture and electronics.\n\n"
    "  Press ENTER when done.")

print("\n  Monitoring accuracy. Target: mag >= 2.\n")

start_good = None
last_print = ticks_ms()

while True:
    imu.update_sensors()

    if ticks_diff(ticks_ms(), last_print) < 200:
        continue
    last_print = ticks_ms()

    _, _, _, mag_acc, _ = imu.magnetic.full

    if mag_acc >= 2:
        if start_good is None:
            start_good = ticks_ms()
            print(f"\n  Mag >= 2. Holding for {STABLE_SECS}s to confirm stability...\n")

        remaining = STABLE_SECS - ticks_diff(ticks_ms(), start_good) / 1000.0
        print(f"  Mag={mag_acc}  GOOD — {remaining:.1f}s remaining")

        if ticks_diff(ticks_ms(), start_good) / 1000.0 >= STABLE_SECS:
            break
    else:
        if start_good is not None:
            print("\n  Lost calibration — keep rotating the sensor.\n")
        start_good = None
        print(f"  Mag={mag_acc}  — keep rotating (figure-8 motion helps)")

imu.save_calibration_data()
print("\n  Calibration saved to BNO085 flash (DCD).")
print("  Magnetometer is ready — you can now run the tare script.")