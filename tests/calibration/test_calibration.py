"""
Milestone 1: Sensor calibration

Calibrates accelerometer, gyroscope, and magnetometer on the BNO085.
Must be performed before tare (Milestone 2) and ideally in the final
mounting position (on the lever assembly).

The IMU board must be detachable from the lever for this procedure, since
calibration requires rotating the device into multiple orientations.

Calibration procedure per spec (1000-4044 v1.6, Section 2.2):
  1. Accelerometer — hold device in 4-6 unique orientations for ~1s each.
     Think of it as a cube: place each face down in turn.
  2. Gyroscope — set device down on a flat surface for 2-3 seconds.
  3. Magnetometer — rotate device ~180 deg and back on each axis (roll, pitch,
     yaw) at ~2 seconds per rotation. Continue until accuracy >= 2.

Environment: calibrate away from magnetic interferers (PC towers, monitors,
metal furniture). The magnetometer calibrates for the local magnetic field,
so recalibrate if the device moves to a different environment.

Accuracy scale (status bits):
  0 = Unreliable
  1 = Low accuracy
  2 = Medium accuracy  <-- minimum target
  3 = High accuracy

The script monitors accuracy in real time and saves calibration data (DCD)
to BNO085 flash once all sensors reach accuracy >= 2 and hold it for 5 seconds.

References:
  - specification/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf
  - specification/IMU BNO08x v1.17.pdf, Section 3 (Calibration and Interpretation)
  - https://github.com/bradcar/bno08x_i2c_spi_MicroPython/blob/main/examples/test_calibration.py
"""

from machine import I2C, Pin
from utime import ticks_ms, ticks_diff, sleep_ms
from i2c import BNO08X_I2C

# === Configuration ===
STABLE_SECS = 5  # How long all sensors must stay >= 2 before saving

# === Hardware Setup ===
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
reset_pin = Pin(2, Pin.OUT)
int_pin = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin, debug=False)

# === Enable sensors needed for calibration ===
# Magnetometer needs 50 Hz for proper calibration (per spec Section 2.2)
imu.acceleration.enable(20)
imu.gyro.enable(20)
imu.magnetic.enable(50)
imu.game_quaternion.enable(20)  # Required by spec — fusion must run for calibration to converge

# === Begin calibration mode ===
imu.begin_calibration()
imu.calibration_status()

# === Helper: drain sensor packets while waiting for user ===
def wait_for_enter(message):
    """Print instructions, then drain sensor data until user presses ENTER.
    Uses non-blocking stdin poll so update_sensors() keeps running."""
    import select
    import sys
    print(message)
    while True:
        imu.update_sensors()
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return

# === Guide the user step by step ===
print("\n=== BNO085 Sensor Calibration ===\n")

wait_for_enter(
    "  STEP 1 — Accelerometer (cube method):\n"
    "    Hold the device in 4-6 different orientations, at least 1 second each.\n"
    "    Imagine the board is a cube — place each face down in turn.\n"
    "    Order doesn't matter. 4-5 faces is usually enough.\n\n"
    "    Press ENTER when done.")

wait_for_enter(
    "\n  STEP 2 — Gyroscope:\n"
    "    Set the device down on a flat surface and leave it still for 2-3 seconds.\n\n"
    "    Press ENTER when done.")

wait_for_enter(
    "\n  STEP 3 — Magnetometer:\n"
    "    Rotate the device ~180 degrees and back on each axis:\n"
    "      - Roll (twist left-right)\n"
    "      - Pitch (tilt forward-back)\n"
    "      - Yaw (turn like a compass)\n"
    "    Speed: ~2 seconds per rotation. Repeat a few times.\n\n"
    "    Press ENTER when done.")

print("\n  Monitoring accuracy. Target: all three sensors >= 2.\n"
      "  Keep adjusting if needed — the script will guide you.\n")

# === Monitor calibration progress ===
start_good = None
last_print = ticks_ms()

while True:
    imu.update_sensors()

    # Print every 200 ms
    if ticks_diff(ticks_ms(), last_print) < 200:
        continue
    last_print = ticks_ms()

    _, _, _, accel_acc, _ = imu.acceleration.full
    _, _, _, mag_acc, _ = imu.magnetic.full
    _, _, _, gyro_acc, _ = imu.gyro.full

    all_good = all(x >= 2 for x in (accel_acc, mag_acc, gyro_acc))

    if all_good:
        if start_good is None:
            start_good = ticks_ms()
            print(f"\n  All sensors >= 2. Holding for {STABLE_SECS}s to confirm stability...\n")

        elapsed_good = ticks_diff(ticks_ms(), start_good) / 1000.0
        remaining = STABLE_SECS - elapsed_good
        print(f"  Accel={accel_acc}  Gyro={gyro_acc}  Mag={mag_acc}  "
              f"GOOD — {remaining:.1f}s remaining")

        if elapsed_good >= STABLE_SECS:
            print(f"\n  Calibration stable for {STABLE_SECS}s.")
            break
    else:
        if start_good is not None:
            print("\n  Lost calibration, resetting timer. Keep moving the sensor.\n")
        start_good = None
        print(f"  Accel={accel_acc}  Gyro={gyro_acc}  Mag={mag_acc}  "
              f"— {'move sensor' if mag_acc < 2 else 'hold still' if gyro_acc < 2 else 'rotate faces'}")

# === Save calibration data ===
imu.save_calibration_data()
print("\n  Calibration saved to BNO085 flash (DCD).")
print("  This persists across power cycles.")
print("\n  You can now proceed to tare (Milestone 2).")
