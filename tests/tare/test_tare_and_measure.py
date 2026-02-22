"""
Tare calibration — before/after bias comparison

Three-phase procedure:

  PHASE 0 — Magnetometer calibration (sensor DETACHED from bench)
    Perform figure-8 motions until mag accuracy >= 2.
    Run tests/calibration/test_calibration_mag.py first if accuracy never reaches 2.

  PHASE 1 — Attach to bench + reset + settle
    Attach sensor at zero position, aligned to magnetic North.
    Script hard-resets the sensor (GPIO 2) to clear gyro integration drift
    accumulated during Phase 0 figure-8 motion. Without reset, the GRV
    carries 45-60 deg of residual drift that 30 s of settling cannot clear.
    After reset the GRV re-initialises from the accelerometer and DCD
    reloads from flash. Wait 30 s for convergence, then collect pre-tare readings.

  PHASE 2 — Tare + verify
    Tare is applied. Post-tare readings are collected and compared.
    Optionally persist the tare offset to BNO085 flash.

Output CSVs (on Pico flash):
  /data/tare_before.csv  — pre-tare IMU vs encoder
  /data/tare_after.csv   — post-tare IMU vs encoder

Why all-axes tare requires mag accuracy >= 2:
  The BNO08X must have resolved magnetic North before taring all axes.
  Without it, the reference frame is broken (roll axis inverted in our testing).
  Spec reference: Tare Usage Guide p2, SH-2 §6.4.4.1.

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
SETTLE_SECS = const(30)
MAG_ACC_MIN = const(2)

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
# PHASE 0: Magnetometer calibration (sensor detached from bench)
# =========================================================================
print("\n=== Phase 0: Magnetometer Calibration ===")
print("DETACH the sensor from the bench before continuing.")
print("Rotate it in a figure-8 pattern on each axis until accuracy >= 2.\n")

imu.magnetic.enable(50)   # 50 Hz required by spec for ME to track mag data
imu.begin_calibration()   # enable all three ME routines so stored DCD re-engages
                          # for accel + gyro; mag needs figure-8 to re-confirm

last_print = ticks_ms()
while True:
    imu.update_sensors()

    if ticks_diff(ticks_ms(), last_print) < 300:
        continue
    last_print = ticks_ms()

    if imu.magnetic.updated:
        _, _, _, mag_acc, _ = imu.magnetic.full
        if mag_acc >= MAG_ACC_MIN:
            print(f"  Mag accuracy: {mag_acc}  — READY")
            break
        print(f"  Mag accuracy: {mag_acc}  — keep rotating (figure-8)")


wait_for_enter(
    "  Point the sensor toward magnetic North.\n"
    "  Attach it to the bench at the ZERO position.\n"
    "  Press ENTER when attached and stable.")


# =========================================================================
# PHASE 1: Sensor on bench — reset, re-enable, settle + collect pre-tare
# =========================================================================

# Hard-reset the sensor to clear gyroscope integration drift accumulated
# during Phase 0 figure-8 motion. Without this reset, the GRV carries
# 45-60 deg of residual drift that even 30 s of settle time cannot overcome.
# After reset the GRV re-initialises from the accelerometer within ~2 s,
# and DCD (mag + accel calibration) is reloaded from flash automatically.
print("\nResetting sensor to clear Phase 0 gyro drift...")
imu.reset_sensor()
print("Reset complete. Re-enabling sensors...\n")

imu.magnetic.enable(50)
imu.begin_calibration()   # re-activate ME routines so DCD re-engages for all sensors
imu.quaternion.enable(RATE_HZ)

print(f"Waiting {SETTLE_SECS}s for GRV to converge from accelerometer reference.")
print("Keep the lever fixed at zero.\n")

settle_start = ticks_ms()
last_print = ticks_ms()
while ticks_diff(ticks_ms(), settle_start) < SETTLE_SECS * 1000:
    imu.update_sensors()
    if imu.quaternion.updated and ticks_diff(ticks_ms(), last_print) >= 500:
        last_print = ticks_ms()
        yaw, pitch, roll, acc, ts_ms = imu.quaternion.euler_full
        enc = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)
        _, _, _, mag_acc, _ = imu.magnetic.full
        remaining = (SETTLE_SECS * 1000 - ticks_diff(ticks_ms(), settle_start)) / 1000
        print(f"  ENC: {enc:+.2f}  IMU roll: {roll:+.2f}  bias: {roll - enc:+.2f}  mag_acc: {mag_acc}  ({remaining:.0f}s left)")


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

            if imu.quaternion.updated:
                yaw, pitch, roll, acc, ts_ms = imu.quaternion.euler_full
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
if imu.quaternion.updated:
    _, _, roll_at_tare, _, _ = imu.quaternion.euler_full
    print(f"\nAt tare moment — ENC: {enc_at_tare:+.2f}  IMU roll: {roll_at_tare:+.2f}  bias: {roll_at_tare - enc_at_tare:+.2f}")
else:
    print(f"\nAt tare moment — ENC: {enc_at_tare:+.2f}  (IMU not updated)")

print("Applying tare (all axes, rotation vector basis)...")
imu.tare(0x07, 0)

# Drain any packets the tare command may have triggered before collecting data.
# Insufficient settling here can leave the I2C bus in a bad state.
for _ in range(50):
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