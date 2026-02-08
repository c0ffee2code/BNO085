"""
Milestone 1: Tare calibration — before/after bias comparison

Lever is fixed near zero position. The script collects IMU vs encoder readings
in two phases: before tare and after tare. Both CSVs use the same format so
they can be compared with analyse_report_rate.py.

PREREQUISITE: Run sensor calibration first (Milestone 1). All-axes tare uses the
Rotation Vector which fuses all three sensors. Without calibrated magnetometer,
accelerometer, and gyroscope, the tare computes a broken reference frame — in our
testing this inverted the roll axis (correlation -0.99 instead of +0.99).

Procedure:
  1. Complete sensor calibration (magnetometer figure-8, accelerometer multi-face,
     gyro ZRO) and save with save_calibration_data().
  2. Point the assembly toward magnetic North, ensure lever is level.
  3. Fix the lever near zero (screwdriver or jig).
  4. Run this script.
  5. Phase 1 collects pre-tare readings (expect ~2 deg bias).
  6. Script applies tare automatically.
  7. Phase 2 collects post-tare readings (expect bias near zero).
  8. Download both CSVs and compare.

Output format (CSV on flash):
  T,ENC,IMU,Lag

Tare per BNO08X spec (Section 4.1.1, page 42):
  - Tare captures the current orientation and computes a rotation offset so all
    future outputs are relative to an East/North/Up frame at the current position.
  - Two types: all axes (0x07 = Z+Y+X, zeroes tilt and heading) or Z-only (0x04,
    heading only). We use all-axes tare.
  - Sent as Command Request (0xF2) on SHTP Channel 2 with ME command 0x03.
  - Persist tare (subcommand 0x01) writes the offset to BNO085 flash so it
    survives power cycles.
  - The spec warns: "the BNO08X must have resolved magnetic North before applying
    the tare function. Otherwise when the magnetometer calibrates the heading will
    change." This matters for yaw axis accuracy. For our current roll-axis-only
    tests we ignore this requirement — the roll angle is determined by the
    accelerometer and gyroscope, not the magnetometer. We will revisit magnetometer
    readiness when testing yaw.

References:
  - specification/BNO080-BNO085-Tare-Function-Usage-Guide.pdf (command byte tables, pages 2-3)
  - specification/IMU BNO08x v1.17.pdf, Section 4.1.1 (tare overview)
  - https://github.com/bradcar/bno08x_i2c_spi_MicroPython/blob/main/examples/test_tare.py
"""

from micropython import const
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff, sleep_ms
from as5600 import AS5600, to_degrees
from i2c import BNO08X_I2C

# === Configuration ===
RATE_HZ = const(344)
SAMPLES_PER_PHASE = const(2000)
AXIS_CENTER = const(422)
SETTLE_SECS = const(5)

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
imu.quaternion.enable(RATE_HZ)

# === Let sensor fusion settle ===
print(f"\nWaiting {SETTLE_SECS}s for sensor fusion to settle...")
print("Keep the lever fixed at zero.\n")

settle_start = ticks_ms()
while ticks_diff(ticks_ms(), settle_start) < SETTLE_SECS * 1000:
    imu.update_sensors()
    if imu.quaternion.updated:
        yaw, pitch, roll, acc, ts_ms = imu.quaternion.euler_full
        enc = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
        print(f"  ENC: {enc:+.2f}  IMU roll: {roll:+.2f}  bias: {roll - enc:+.2f}")
        sleep_ms(500)


def collect_samples(output_file):
    """Collect SAMPLES_PER_PHASE readings into a CSV file."""
    start_ms = ticks_ms()
    count = 0

    f = open(output_file, "w")
    f.write("T,ENC,IMU,Lag\n")

    try:
        while count < SAMPLES_PER_PHASE:
            now_ms = ticks_ms()
            encoder_angle = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)

            if imu.update_sensors() > 0:
                pass

            if imu.quaternion.updated:
                yaw, pitch, roll, acc, ts_ms = imu.quaternion.euler_full
                imu_now_ms = imu.bno_start_diff(now_ms)
                lag = imu_now_ms - ts_ms
                elapsed = ticks_diff(now_ms, start_ms)
                f.write(f"{elapsed},{encoder_angle:.2f},{roll:.2f},{lag:.1f}\n")
                count += 1
    except KeyboardInterrupt:
        pass
    finally:
        f.close()

    elapsed_s = ticks_diff(ticks_ms(), start_ms) / 1000.0
    hz = count / elapsed_s if elapsed_s > 0 else 0
    print(f"  {count} samples in {elapsed_s:.1f}s ({hz:.1f} Hz) -> {output_file}")
    return count


# === Phase 1: Before tare ===
print("\n--- Phase 1: Before tare ---")
print("Collecting readings with current (untared) orientation...\n")
collect_samples("/data/tare_before.csv")

# === Apply tare ===
print("\nApplying tare (all axes, rotation vector basis)...")
imu.tare(0x07, 0)

# Let tare take effect
for _ in range(20):
    imu.update_sensors()
    sleep_ms(10)

print("Tare applied. Post-tare check:")
for _ in range(5):
    imu.update_sensors()
    if imu.quaternion.updated:
        yaw, pitch, roll, acc, ts_ms = imu.quaternion.euler_full
        enc = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
        print(f"  ENC: {enc:+.2f}  IMU roll: {roll:+.2f}  bias: {roll - enc:+.2f}")
    sleep_ms(200)

# === Phase 2: After tare ===
print("\n--- Phase 2: After tare ---")
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
