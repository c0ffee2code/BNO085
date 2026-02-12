"""
Experiment 3: Gyro Integrated Rotation Vector (0x2A) - Channel 5

Tests BNO085 gyro integrated rotation vector against AS5600 encoder
reference on single axis.  This report arrives on dedicated SHTP Channel 5
at up to 1000 Hz and includes both quaternion orientation and angular
velocity — no accuracy field or base timestamp.

Cannot reuse report_rate_test.run() because the 8-tuple layout
(qr, qi, qj, qk, angvel_x, angvel_y, angvel_z, ts) differs from the
standard 6-tuple used by other quaternion reports.

Output CSV columns:
  T        – ms since test start
  ENC      – encoder angle (degrees)
  IMU      – roll from euler conversion (degrees)
  Lag      – ms between host clock and IMU timestamp
  AngVelX  – angular velocity X (rad/s)
  AngVelY  – angular velocity Y (rad/s)
  AngVelZ  – angular velocity Z (rad/s)

Metrics (from ADR-001):
- Latency: ms delay between physical motion and reported angle
- Angle Error: degrees deviation from encoder reference
- Update Rate: actual Hz achieved (I2C expected ~280-340 Hz, not 1000)
"""

from micropython import const
from utime import ticks_ms, ticks_diff
from report_rate_test import setup_hardware, REPORT_RATE, MAX_SAMPLES, AXIS_CENTER
from as5600 import to_degrees

RATE = const(1000)  # request max; I2C will cap around 280-340 Hz

imu, encoder = setup_hardware()
feature = imu.gyro_integrated_rotation_vector

output_file = f"/data/gyro_int_rv_{RATE}hz_{MAX_SAMPLES}s.csv"

# === Ensure output directory exists ===
try:
    import os
    os.mkdir("/data")
except OSError:
    pass

# === Enable at max rate ===
feature.enable(RATE)

# === Main loop ===
start_ms = ticks_ms()
sample_count = 0

print(f"\nReport Rate Test [gyro_int_rv] - {RATE} Hz requested")
print(f"Collecting {MAX_SAMPLES} samples -> {output_file}")
print(f"Press Ctrl+C to abort early.\n")

f = open(output_file, "w")
f.write("T,ENC,IMU,Lag,AngVelX,AngVelY,AngVelZ\n")

try:
    while sample_count < MAX_SAMPLES:
        now_ms = ticks_ms()

        encoder_angle = to_degrees(encoder.read_raw_angle(), AXIS_CENTER)

        if imu.update_sensors() > 0:
            pass

        if feature.updated:
            # .full returns (qr, qi, qj, qk, angvel_x, angvel_y, angvel_z, ts_ms)
            qr, qi, qj, qk, avx, avy, avz, ts_ms = feature.full
            yaw, pitch, roll = imu.euler_conversion(qr, qi, qj, qk)

            imu_now_ms = imu.bno_start_diff(now_ms)
            lag = imu_now_ms - ts_ms
            elapsed = ticks_diff(now_ms, start_ms)

            f.write(f"{elapsed},{encoder_angle:.2f},{roll:.2f},{lag:.1f},{avx:.4f},{avy:.4f},{avz:.4f}\n")
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
