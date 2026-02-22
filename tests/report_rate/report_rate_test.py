"""
Shared test module for BNO085 rotation vector report rate experiments.

Provides hardware setup and a reusable sample collection loop used by
test_baseline_rotation_vector.py and test_game_rotation_vector.py.

Output format (CSV on flash):
  T,ENC,IMU,Lag
"""

from micropython import const
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff
from as5600 import AS5600, to_degrees
from i2c import BNO08X_I2C

# === Configuration ===
REPORT_RATE = const(344)  # 344 Hz - fastest supported on I2C (10 Hz tested, no value for flight control)
MAX_SAMPLES = const(2000) # Stop after this many samples to limit flash usage
AXIS_CENTER = const(406)  # Encoder zero position — raw=406 when lever at physical zero (precision 3D-printed jig, 2026-02-22)


def setup_hardware():
    """Initialise I2C bus, AS5600 encoder and BNO08X IMU.

    Returns:
        (imu, encoder) tuple ready for use.
    """
    i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)

    encoder = AS5600(i2c=i2c)

    reset_pin = Pin(2, Pin.OUT)
    int_pin = Pin(3, Pin.IN, Pin.PULL_UP)
    imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin, debug=False)

    return imu, encoder


def run(imu, feature, encoder, prefix="rot_vec"):
    """Run a report-rate test collecting IMU vs encoder samples to CSV.

    Args:
        imu: BNO08X_I2C instance (used for update_sensors / bno_start_diff).
        feature: SensorFeature4 instance (e.g. imu.quaternion, imu.game_quaternion).
        encoder: AS5600 encoder instance.
        prefix: filename prefix for the output CSV (e.g. "rot_vec", "game_rot_vec").
    """
    output_file = f"/data/{prefix}_{REPORT_RATE}hz_{MAX_SAMPLES}s.csv"

    # === Ensure output directory exists ===
    try:
        import os
        os.mkdir("/data")
    except OSError:
        pass  # already exists

    # === Enable the feature at max I2C rate ===
    feature.enable(REPORT_RATE)

    # === Main Loop ===
    start_ms = ticks_ms()
    sample_count = 0

    print(f"\nReport Rate Test [{prefix}] - {REPORT_RATE} Hz")
    print(f"Collecting {MAX_SAMPLES} samples -> {output_file}")
    print(f"Press Ctrl+C to abort early.\n")

    f = open(output_file, "w")
    f.write("T,ENC,IMU,Lag\n")

    try:
        while sample_count < MAX_SAMPLES:
            now_ms = ticks_ms()

            encoder_angle = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)

            if imu.update_sensors() > 0:
                pass

            if feature.updated:
                yaw, pitch, roll, acc, ts_ms = feature.euler_full

                imu_now_ms = imu.bno_start_diff(now_ms)
                lag = imu_now_ms - ts_ms
                elapsed = ticks_diff(now_ms, start_ms)

                f.write(f"{elapsed},{encoder_angle:.2f},{roll:.2f},{lag:.1f}\n")
                sample_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        f.close()

    # === Console summary ===
    end_ms = ticks_ms()
    elapsed_s = ticks_diff(end_ms, start_ms) / 1000.0
    actual_hz = sample_count / elapsed_s if elapsed_s > 0 else 0

    print(f"\nDone. {sample_count} samples in {elapsed_s:.1f}s ({actual_hz:.1f} Hz)")
    print(f"Data saved to {output_file}")
    print(f"Retrieve with:  mpremote cp :{output_file} .")
