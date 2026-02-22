"""
Tare calibration — before/after bias comparison

Two-phase procedure:

  PHASE 1 — Attach to bench + settle
    Attach sensor at zero position.
    Wait SETTLE_SECS for Game Rotation Vector to stabilise from accelerometer,
    then collect pre-tare readings.

  PHASE 2 — Tare + verify
    Tare is applied (Game Rotation Vector basis — no magnetometer required).
    Post-tare readings are collected and compared.
    Optionally persist the tare offset to BNO085 flash.

Why Game Rotation Vector basis for tare:
  imu.tare(axes, basis=1) targets the Game Rotation Vector (GRV), which is
  driven by accelerometer + gyroscope only.  No magnetometer is involved.
  This avoids reference-frame corruption in environments where the local magnetic
  field differs from the calibration environment (near motors, ESCs, metal
  structures, or after moving the device to a new room).

  With basis=0 (Rotation Vector) the tare bakes in the current magnetometer
  heading.  If mag accuracy is 0 on the bench, that heading is arbitrary and
  the tare produces a ~45 deg tilted reference frame — observed in testing.

  For yaw accuracy (heading), recalibrate the magnetometer in the operating
  environment with tests/calibration/test_calibration_mag.py and switch
  basis back to 0.

  Spec reference: Tare Usage Guide p2–3, SH-2 §6.4.4.1.

Output CSVs (on Pico flash):
  /data/tare_before.csv  — pre-tare IMU vs encoder
  /data/tare_after.csv   — post-tare IMU vs encoder

References:
  - specification/BNO080-BNO085-Tare-Function-Usage-Guide.pdf
  - specification/IMU BNO08x v1.17.pdf, Section 4.1.1
"""

from micropython import const
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff, sleep_ms
from as5600 import AS5600, to_degrees
from i2c import BNO08X_I2C

# === Configuration ===
RATE_HZ = const(344)
SAMPLES_PER_PHASE = const(2000)
AXIS_CENTER = const(411)  # raw reading when lever is at physical zero (was 422, corrected from tare run)
SETTLE_SECS = const(10)

# === Ensure output directory exists ===
try:
    import os
    os.mkdir("/data")
except OSError:
    pass

# === Hardware Setup ===
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)
encoder = AS5600(i2c=i2c)
reset_pin = Pin(2, Pin.OUT)
int_pin = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin, debug=False)


def wait_for_enter(message):
    """Print instructions and drain sensor packets until user presses ENTER."""
    import select, sys
    print(message)
    while True:
        imu.update_sensors()
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return


# =========================================================================
# PHASE 1: Sensor on bench — enable, settle + collect pre-tare
# =========================================================================
wait_for_enter(
    "\n=== Phase 1: Attach to bench ===\n"
    "  Attach the sensor at the ZERO position.\n"
    "  Press ENTER when attached and stable.")

imu.game_quaternion.enable(RATE_HZ)

print(f"\nWaiting {SETTLE_SECS}s for Game Rotation Vector to stabilise.")
print("Keep the lever fixed at zero.\n")

settle_start = ticks_ms()
last_print = ticks_ms()
while ticks_diff(ticks_ms(), settle_start) < SETTLE_SECS * 1000:
    imu.update_sensors()
    if imu.game_quaternion.updated and ticks_diff(ticks_ms(), last_print) >= 500:
        last_print = ticks_ms()
        yaw, pitch, roll, acc, ts_ms = imu.game_quaternion.euler_full
        enc = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
        remaining = (SETTLE_SECS * 1000 - ticks_diff(ticks_ms(), settle_start)) / 1000
        print(f"  ENC: {enc:+.2f}  IMU roll: {roll:+.2f}  bias: {roll - enc:+.2f}  ({remaining:.0f}s left)")


def collect_samples(output_file):
    """Collect SAMPLES_PER_PHASE readings into a CSV file."""
    start_ms = ticks_ms()
    count = 0
    i2c_errors = 0

    f = open(output_file, "w")
    f.write("T,ENC,IMU,Lag,ENC_RAW\n")

    try:
        while count < SAMPLES_PER_PHASE:
            now_ms = ticks_ms()
            raw_angle = encoder.read_raw_angle()
            encoder_angle = to_degrees(raw_angle, AXIS_CENTER)

            try:
                imu.update_sensors()
            except OSError:
                i2c_errors += 1
                if i2c_errors > 10:
                    print(f"\n  Too many I2C errors ({i2c_errors}), aborting collection.")
                    break
                sleep_ms(10)
                continue

            if imu.game_quaternion.updated:
                yaw, pitch, roll, acc, ts_ms = imu.game_quaternion.euler_full
                imu_now_ms = imu.bno_start_diff(now_ms)
                lag = imu_now_ms - ts_ms
                elapsed = ticks_diff(now_ms, start_ms)
                f.write(f"{elapsed},{encoder_angle:.2f},{roll:.2f},{lag:.1f},{raw_angle}\n")
                count += 1
    except KeyboardInterrupt:
        pass
    finally:
        f.close()

    elapsed_s = ticks_diff(ticks_ms(), start_ms) / 1000.0
    hz = count / elapsed_s if elapsed_s > 0 else 0
    if i2c_errors:
        print(f"  WARNING: {i2c_errors} I2C error(s) during collection")
    print(f"  {count} samples in {elapsed_s:.1f}s ({hz:.1f} Hz) -> {output_file}")
    return count


print("\n--- Pre-tare readings ---")
print("Collecting readings with current (untared) orientation...\n")
collect_samples("/data/tare_before.csv")


# =========================================================================
# PHASE 2: Apply tare + collect post-tare
# =========================================================================
imu.update_sensors()
enc_at_tare = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
if imu.game_quaternion.updated:
    _, _, roll_at_tare, _, _ = imu.game_quaternion.euler_full
    print(f"\nAt tare moment — ENC: {enc_at_tare:+.2f}  IMU roll: {roll_at_tare:+.2f}  bias: {roll_at_tare - enc_at_tare:+.2f}")
else:
    print(f"\nAt tare moment — ENC: {enc_at_tare:+.2f}  (IMU not updated)")

print("Applying tare (all axes, Game Rotation Vector basis — no mag required)...")
imu.tare(0x07, 1)

# Drain any packets the tare command may have triggered before collecting data.
# Insufficient settling here can leave the I2C bus in a bad state.
for _ in range(50):
    imu.update_sensors()
    sleep_ms(10)

print("Tare applied. Post-tare check:")
for _ in range(5):
    imu.update_sensors()
    if imu.game_quaternion.updated:
        yaw, pitch, roll, acc, ts_ms = imu.game_quaternion.euler_full
        enc = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
        print(f"  ENC: {enc:+.2f}  IMU roll: {roll:+.2f}  bias: {roll - enc:+.2f}")
    sleep_ms(200)

print("\n--- Post-tare readings ---")
print("Collecting readings with tared orientation...\n")
collect_samples("/data/tare_after.csv")

# === Persist tare ===
save = input("\nSave tare to BNO085 flash? [y/N]: ").strip().lower()
if save == "y":
    imu.save_tare_data()
    print("Tare saved (survives power cycle).")
else:
    print("Tare NOT saved (lost on power cycle).")

print("\nDone. Retrieve files with:")
print("  mpremote cp :/data/tare_before.csv .")
print("  mpremote cp :/data/tare_after.csv .")
print("\nCompare with:")
print("  python analyse_report_rate.py tare_before.csv tare_after.csv")