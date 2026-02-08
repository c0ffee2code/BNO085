"""
Experiment 1: Baseline Rotation Vector (0x05) - Single Axis

Tests BNO085 rotation vector against AS5600 encoder reference on single axis.
Writes telemetry to Pico's internal flash filesystem to avoid USB serial bottleneck.
Retrieve the output file afterwards via Thonny/mpremote.

Metrics (from ADR-001):
- Latency: ms delay between physical motion and reported angle
- Angle Error: degrees deviation from encoder reference
- Update Rate: actual Hz achieved

Output format (CSV on flash):
  T,ENC,IMU,Lag
"""

from micropython import const
from machine import I2C, Pin
from utime import ticks_ms, ticks_diff
from as5600 import AS5600, to_degrees
from i2c import BNO08X_I2C

# === Configuration ===
RATE_SLOW = const(10)     # 10 Hz - slowest (driver default for rotation vector)
RATE_FAST = const(344)    # 344 Hz - fastest supported on I2C
MAX_SAMPLES = const(2000) # Stop after this many samples to limit flash usage
AXIS_CENTER = const(422)  # Encoder zero position (recalibrate if mechanical changes)
# === Rate Selection ===
print("Select report rate:")
print(f"  1) {RATE_SLOW} Hz (slowest)")
print(f"  2) {RATE_FAST} Hz (fastest on I2C)")
choice = input("Choice [1]: ").strip()
report_rate_hz = RATE_FAST if choice == "2" else RATE_SLOW
OUTPUT_FILE = f"/data/rot_vec_{report_rate_hz}hz_{MAX_SAMPLES}s.csv"

# === Ensure output directory exists ===
try:
    import os
    os.mkdir("/data")
except OSError:
    pass  # already exists

# === Hardware Setup ===
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000)

encoder = AS5600(i2c=i2c)

reset_pin = Pin(2, Pin.OUT)
int_pin = Pin(3, Pin.IN, Pin.PULL_UP)
imu = BNO08X_I2C(i2c, address=0x4a, reset_pin=reset_pin, int_pin=int_pin, debug=False)
imu.quaternion.enable(report_rate_hz)

# === Main Loop ===
start_ms = ticks_ms()
sample_count = 0

print(f"\nBaseline Rotation Vector Test - Report Rate: {report_rate_hz} Hz")
print(f"Collecting {MAX_SAMPLES} samples -> {OUTPUT_FILE}")
print(f"Press Ctrl+C to abort early.\n")

f = open(OUTPUT_FILE, "w")
f.write("T,ENC,IMU,Lag\n")

try:
    while sample_count < MAX_SAMPLES:
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
print(f"Data saved to {OUTPUT_FILE}")
print(f"Retrieve with:  mpremote cp :{OUTPUT_FILE} .")
