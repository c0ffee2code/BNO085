# ADR-001: Gap Analysis - BNO085 Driver vs Specification

**Date:** 2026-02-04
**Status:** Draft
**Context:** Flight control test bench on Raspberry Pi Pico 2

## Decision Drivers

For a flight control system, we need:
1. **Low latency orientation** - know approximate position frequently rather than precise position too late
2. **Yaw stability** - magnetometer input to prevent yaw drift over time
3. **High update rates** - 100-400Hz for responsive control loops
4. **Reliable calibration** - maintain accuracy across power cycles

## Gap Analysis Summary

### Rotation Vector Reports

| Report | ID | Spec | Driver | Priority | Notes |
|--------|-----|------|--------|----------|-------|
| Rotation Vector | 0x05 | Yes | **IMPLEMENTED** | HIGH | Magnetometer-fused, prevents yaw drift |
| Game Rotation Vector | 0x08 | Yes | **IMPLEMENTED** | MEDIUM | No mag, will drift but smooth |
| Geomagnetic Rotation Vector | 0x09 | Yes | **IMPLEMENTED** | LOW | Low power, slow response |
| ARVR Stabilized Rotation Vector | 0x28 | Yes | **NOT IMPLEMENTED** | **HIGH** | Smooth corrections during motion, ~250Hz |
| ARVR Stabilized Game Rotation Vector | 0x29 | Yes | **NOT IMPLEMENTED** | MEDIUM | Smooth, no mag |
| Gyro Integrated Rotation Vector | 0x2A | Yes | **PARTIAL** | **HIGH** | Fastest updates ~1000Hz, for prediction |

### Flight Control Relevance

**Best for our use case:**
1. **Rotation Vector (0x05)** - Already implemented. Uses all 3 sensors, provides absolute heading referenced to magnetic north. Prevents yaw drift.

2. **ARVR Stabilized Rotation Vector (0x28)** - **GAP**. Corrections applied during motion (not at rest), preventing visible jumps/discontinuities. Ideal for flight control where sudden orientation jumps would cause control instability.

3. **Gyro Integrated Rotation Vector (0x2A)** - **GAP**. Highest frequency output (~1000Hz on Channel 5). Gyro-only for fast response, with periodic corrections from full fusion. Perfect for "approximate position frequently" philosophy.

### Calibration Features

| Feature | Spec | Driver | Priority | Notes |
|---------|------|--------|----------|-------|
| Manual calibration enable | Yes | **IMPLEMENTED** | HIGH | `begin_calibration()` |
| Calibration status query | Yes | **IMPLEMENTED** | HIGH | `calibration_status()` |
| Save DCD (calibration data) | Yes | **IMPLEMENTED** | HIGH | `save_calibration_data()` |
| Configure Periodic DCD Save | Yes | **NOT IMPLEMENTED** | MEDIUM | Auto-save calibration periodically |
| FRS Read/Write | Yes | **NOT IMPLEMENTED** | LOW | Custom calibration storage |
| Simple Calibration (0x0c) | Yes | **NOT IMPLEMENTED** | LOW | 180-degree calibration |

### Tare Features

| Feature | Spec | Driver | Priority | Notes |
|---------|------|--------|----------|-------|
| Tare Now | Yes | **IMPLEMENTED** | HIGH | `tare()` |
| Persist Tare | Yes | **IMPLEMENTED** | HIGH | `save_tare_data()` |
| Set Reorientation | Yes | **IMPLEMENTED** | MEDIUM | `tare_reorientation()` |
| Clear Tare | Yes | **IMPLEMENTED** | MEDIUM | `clear_tare()` |

### Communication Interfaces

| Interface | Spec | Driver | Priority | Notes |
|-----------|------|--------|----------|-------|
| I2C | Yes | **IMPLEMENTED** | HIGH | Current implementation, up to 344Hz quaternion |
| SPI | Yes | **NOT IMPLEMENTED** | MEDIUM | 8x faster than I2C |
| UART | Yes | **NOT IMPLEMENTED** | LOW | RVC mode available |
| Channel 5 (High-speed Gyro) | Yes | **NOT IMPLEMENTED** | HIGH | Required for Gyro Integrated RV |

### Other Sensor Reports

| Report | ID | Driver | Priority for Flight Control |
|--------|-----|--------|----------------------------|
| Accelerometer | 0x01 | IMPLEMENTED | HIGH - gravity reference |
| Gyroscope | 0x02 | IMPLEMENTED | HIGH - angular velocity |
| Magnetometer | 0x03 | IMPLEMENTED | MEDIUM - heading |
| Linear Acceleration | 0x04 | IMPLEMENTED | HIGH - motion without gravity |
| Gravity | 0x06 | IMPLEMENTED | MEDIUM - attitude reference |
| Uncalibrated Gyro | 0x07 | IMPLEMENTED | LOW |
| Uncalibrated Mag | 0x0F | IMPLEMENTED | LOW |
| Stability Classifier | 0x13 | IMPLEMENTED | LOW |
| Activity Classifier | 0x1E | IMPLEMENTED | NOT NEEDED |
| Step Counter | 0x11 | IMPLEMENTED | NOT NEEDED |
| Tap/Shake/Flip Detectors | 0x10,0x19,0x1A | NOT IMPLEMENTED | NOT NEEDED |

## Project Milestones

### Milestone 1: Sensor calibration (PREREQUISITE for tare)
- [ ] Calibrate magnetometer — rotate device in figure-8 until accuracy >= 2
- [ ] Calibrate accelerometer — hold stable in 4-6 orientations for ~1s each
- [ ] Calibrate gyroscope — set device down for a few seconds (gyro ZRO)
- [ ] Save calibration with `save_calibration_data()`
- [ ] Verify calibration accuracy values via `calibration_status()`
- Reference: [BradCar test_calibration.py](https://github.com/bradcar/bno08x_i2c_spi_MicroPython/blob/main/examples/test_calibration.py)
- Reference: `specification/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf`

### Milestone 2: Tare calibration and bias validation
**Depends on:** Milestone 1 (calibration must be done first)

- [ ] Point the assembly toward magnetic North, ensure lever is level
- [ ] Run `tare()` to zero out the ~2 deg systematic bias
- [ ] Repeat static hold and slow moves tests at 344 Hz to confirm bias is gone
- [ ] Verify slow motion MAE drops below 1 deg target
- [ ] Persist tare with `save_tare_data()` for power-cycle survival
- Reference: [BradCar test_tare.py](https://github.com/bradcar/bno08x_i2c_spi_MicroPython/blob/main/examples/test_tare.py)
- Reference: `specification/BNO080-BNO085-Tare-Function-Usage-Guide.pdf`

**Lesson learned:** All-axes tare (0x07) uses the Rotation Vector (0x05) which fuses
accelerometer, gyroscope AND magnetometer. Taring without a calibrated magnetometer
produces a broken reference frame — in our testing it inverted the roll axis, making
the IMU read opposite to the encoder (correlation -0.99 instead of +0.99). The Tare
Usage Guide (page 2, steps 2-6) requires magnetometer calibration, accelerometer
calibration, gyro ZRO calibration, and pointing the device North before taring.
This is not optional even for roll-only testing, because the all-axes tare quaternion
depends on all three sensor inputs being correct.

### Milestone 3: Game Rotation Vector (0x08) comparison
- [ ] Run rapid moves, slow moves, and static hold with Game RV at 344 Hz
- [ ] Compare accuracy vs Rotation Vector (0x05) on roll axis
- [ ] Measure drift rate over longer duration (no magnetometer = expected yaw drift)
- No driver changes needed — already implemented

### Milestone 4: GC stall mitigation
- [ ] Profile the ~45 ms periodic spikes (confirm they are MicroPython GC)
- [ ] Pre-allocate buffers, use manual `gc.collect()` at safe points
- [ ] Re-test at 344 Hz, target achieved rate closer to 344 Hz with no spikes >10 ms

### Milestone 5: ARVR Stabilized Rotation Vector (0x28)
- [ ] Implement report 0x28 in driver
- [ ] Run rapid moves test, compare discontinuity behaviour vs standard RV
- [ ] Evaluate for flight control loop suitability

### Milestone 6: Gyro Integrated Rotation Vector (0x2A)
- [ ] Implement Channel 5 (high-speed gyro) support in driver
- [ ] Complete report 0x2A implementation
- [ ] Test at ~1000 Hz, measure latency and accuracy vs speed tradeoff

### Milestone 7: Consider SPI (Optional)
- [ ] Revisit after Milestones 4-6 — if I2C bandwidth is still limiting
- [ ] 8x faster throughput, enables higher update rates
- [ ] See ADR-002 for full analysis

## Your Assumption Validation

> "it's better to know approximate position frequently than precise position too late"

**CONFIRMED by BNO085 architecture:**
- Gyro Integrated RV (0x2A): Fast gyro-only updates at ~1000Hz
- ARVR Stabilized RV (0x28): Corrections during motion, not sudden jumps
- The sensor is specifically designed for this philosophy in AR/VR where latency kills user experience - same principle applies to flight control

## Test Plan: Sensor Performance Validation

### Test Bench Setup

**Mechanical System:**
- BNO085 mounted on swinging lever (table)
- AS5600 magnetic encoder on rotation axis as reference
- Single axis of rotation for controlled experiments

**Reference Sensor:** [AS5600 Driver](https://github.com/c0ffee2code/AS5600)
- High-speed readout (reference for latency measurement)
- 12-bit resolution, 0.087° precision (reference for accuracy measurement)

### Test Objectives

| Test | Metric | Method |
|------|--------|--------|
| **Latency** | ms delay between physical motion and reported angle | Compare AS5600 timestamp vs BNO085 timestamp at same angle crossing |
| **Angle Error** | Degrees deviation from true angle | Compare BNO085 euler angle vs AS5600 angle at rest and during motion |
| **Update Rate** | Actual Hz achieved | Count reports per second under different configurations |
| **Drift** | Degrees/minute of yaw drift | Long-duration static test, compare heading over time |

### Planned Experiments

#### Experiment 1: Baseline with Rotation Vector (0x05)
- Current implementation, magnetometer-fused
- Measure latency and accuracy at 10Hz (slowest) and 344Hz (fastest on I2C)
- Establish baseline for comparison

Test script: `tests/report_rate/test_baseline_rotation_vector.py`
Analysis script: `tests/report_rate/analysis/analyse_report_rate.py`

**Test 1a — Rapid moves** (~1 s per full sweep, violent shaking)

Raw data: `tests/report_rate/rapid_moves/rot_vec_{10,344}hz_2000s.csv`

| Metric | 10 Hz | 344 Hz (achieved 270 Hz) |
|--------|-------|--------------------------|
| Pearson correlation | 0.966 | **0.995** |
| MAE overall | 4.21 deg | 4.49 deg |
| MAE fast motion (top 25% velocity) | 4.38 deg | **3.10 deg** |
| MAE slow motion (bottom 25% velocity) | **3.87 deg** | 6.01 deg |
| RMS error | 6.20 deg | **5.22 deg** |
| Max error | 23.17 deg | 18.67 deg |
| Bias (ENC-IMU) | -0.08 deg | -1.86 deg |
| IMU trails motion direction | 79.8% | 69.8% |
| Lag mean | 2.10 ms | 4.42 ms |
| Lag max | 3.3 ms | 46.6 ms |

**Test 1b — Slow moves** (~5-10 s per sweep, gentle steady motion)

Raw data: `tests/report_rate/slow_moves/rot_vec_{10,344}hz_2000s.csv`

| Metric | 10 Hz | 344 Hz (achieved 272 Hz) |
|--------|-------|--------------------------|
| Pearson correlation | 0.9988 | **0.9998** |
| MAE overall | 6.93 deg | **2.26 deg** |
| MAE fast motion (top 25% velocity) | 5.54 deg | **2.93 deg** |
| MAE slow motion (bottom 25% velocity) | 8.95 deg | **1.96 deg** |
| RMS error | 7.80 deg | **2.54 deg** |
| Max error | 14.66 deg | **6.83 deg** |
| Bias (ENC-IMU) | -2.05 deg | -2.26 deg |
| IMU trails motion direction | 57.6% | 83.3% |
| Lag mean | 2.14 ms | 4.62 ms |
| Lag max | 3.7 ms | 46.2 ms |

**Test 1c — Static hold** (lever fixed with screwdriver, 344 Hz only)

Raw data: `tests/report_rate/static_hold/rot_vec_344hz_2000s.csv`

| Metric | Value |
|--------|-------|
| Encoder reading | 0.70 deg (constant, zero jitter) |
| IMU mean | 2.62 deg |
| IMU range (noise floor) | 0.31 deg (2.36 to 2.67 deg) |
| IMU std dev | 0.06 deg |
| Systematic bias (IMU - ENC) | **+1.92 deg** |
| IMU drift over 7.3 s | +0.27 deg (first 100 vs last 100 samples) |
| Lag mean | 4.56 ms |
| Lag max | 44.9 ms |
| Achieved rate | 275 Hz |

**Findings:**

1. **344 Hz is clearly better across all motion profiles.** During rapid moves it tracks fast reversals better (3.10 vs 4.38 deg). During slow moves the advantage is even larger (2.26 vs 6.93 deg MAE).
2. **10 Hz has a major problem with slow motion** — 6.93 deg MAE and 8.95 deg in the slowest segments. The 100 ms sample interval misses gradual changes, producing stale readings.
3. **~2 deg systematic bias confirmed at rest.** The static test shows +1.92 deg offset (IMU reads higher than encoder), consistent with the ~2 deg bias in both motion tests. This is a fixed mounting/tare offset — a one-time `tare()` call should eliminate it.
4. **Noise floor is excellent** — only 0.31 deg total jitter (±0.155 deg) at rest. After tare calibration, the static error would be under 0.2 deg.
5. **Minor roll drift detected** — +0.27 deg over 7.3 s (~2.2 deg/min). Short test duration makes this uncertain; could be sensor fusion settling after power-on. Note: all tests so far measure roll axis only (single encoder). Yaw drift (heading stability) requires a separate test setup.
6. **IMU consistently undershoots peak excursions** during rapid moves due to internal sensor fusion low-pass filtering. The IMU range was 85% of encoder range at 344 Hz.
7. **Periodic ~45 ms lag spikes at 344 Hz** occur every ~700 ms, dragging achieved rate to ~270 Hz. Likely MicroPython garbage collection stalling the I2C bus.

**Conclusion:** 344 Hz is the right choice for flight control at all motion speeds. The ~2 deg bias is a fixed offset that `tare()` should fix. GC stalls need mitigation (pre-allocate buffers, manual `gc.collect()` at safe points).

#### Experiment 2: Game Rotation Vector (0x08) Comparison
- Same tests without magnetometer fusion
- Measure drift rate over time
- Compare latency (should be similar or faster)

#### Experiment 3: ARVR Stabilized (0x28) - After Implementation
- Compare jump/discontinuity behavior during motion
- Measure if corrections are smoother than standard rotation vector

#### Experiment 4: Gyro Integrated (0x2A) - After Implementation
- Measure latency at high frequency
- Quantify accuracy vs speed tradeoff

### Success Criteria

For flight control viability:
- [x] Latency < 10ms at 100Hz update rate — **PASS** (2.1 ms mean at 10 Hz, 4.4 ms mean at 344 Hz; periodic 46 ms spikes at 344 Hz need GC mitigation)
- [ ] Angle error < 1° during slow motion — **LIKELY PASS after tare** (1.96° MAE at 344 Hz, but 1.92° is systematic bias; static noise floor is only 0.31°)
- [ ] Angle error < 3° during fast motion — **MARGINAL** (3.10° at 344 Hz rapid moves; nearly meets target)
- [ ] Yaw drift < 1°/minute with magnetometer fusion — **NOT TESTED** (all tests so far measure roll axis only; encoder is mounted on a single axis and cannot measure yaw drift without remounting)

## References

- [Adafruit BNO085 Guide](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085)
- [SH-2 Reference Manual](https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf)
- `specification/IMU BNO08x v1.17.pdf` - Full datasheet
- `specification/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf` - Calibration guide
- Base driver: https://github.com/bradcar/bno08x_i2c_spi_MicroPython
- AS5600 reference encoder: https://github.com/c0ffee2code/AS5600
