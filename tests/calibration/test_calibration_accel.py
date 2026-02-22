"""
Accelerometer-only calibration

Run this when only the accelerometer needs recalibration (e.g. after the
sensor PCB was remounted or its orientation changed).

Procedure:
  1. Detach the IMU from the lever so you can rotate it freely.
  2. Run this script.
  3. Hold the board in 4-6 distinct orientations (~1s each).
     Think of it as a cube — rest each face down in turn.
  4. Script monitors accuracy and saves DCD once accel >= 2.

Accuracy scale: 0=Unreliable  1=Low  2=Medium (target)  3=High

References:
  - specification/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf (Section 2.2)
  - SH-2 Reference Manual §6.4.7.1 (Configure ME Calibration, P0=Accel)
"""

from machine import I2C, Pin
from utime import ticks_ms, ticks_diff, sleep_ms
from i2c import BNO08X_I2C

STABLE_SECS = 5

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
reset_pin = Pin(2, Pin.OUT)
int_pin = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin, debug=False)

imu.acceleration.enable(20)
imu.game_quaternion.enable(20)  # fusion must run for calibration to converge

imu.begin_accel_calibration()
imu.calibration_status()


def wait_for_enter(message):
    import select, sys
    print(message)
    while True:
        imu.update_sensors()
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return


print("\n=== Accelerometer Calibration ===\n")

wait_for_enter(
    "  Hold the device in 4-6 different orientations, ~1 second each.\n"
    "  Think of it as a cube — rest each face down in turn.\n"
    "  Order doesn't matter. 4-5 faces is usually enough.\n\n"
    "  Press ENTER when done.")

print("\n  Monitoring accuracy. Target: accel >= 2.\n")

start_good = None
last_print = ticks_ms()

while True:
    imu.update_sensors()

    if ticks_diff(ticks_ms(), last_print) < 200:
        continue
    last_print = ticks_ms()

    _, _, _, accel_acc, _ = imu.acceleration.full

    if accel_acc >= 2:
        if start_good is None:
            start_good = ticks_ms()
            print(f"\n  Accel >= 2. Holding for {STABLE_SECS}s to confirm stability...\n")

        remaining = STABLE_SECS - ticks_diff(ticks_ms(), start_good) / 1000.0
        print(f"  Accel={accel_acc}  GOOD — {remaining:.1f}s remaining")

        if ticks_diff(ticks_ms(), start_good) / 1000.0 >= STABLE_SECS:
            break
    else:
        if start_good is not None:
            print("\n  Lost calibration — keep rotating through more orientations.\n")
        start_good = None
        print(f"  Accel={accel_acc}  — rotate to a new face")

imu.save_calibration_data()
print("\n  Calibration saved to BNO085 flash (DCD).")