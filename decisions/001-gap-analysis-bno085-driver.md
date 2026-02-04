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

## Recommendations

### Phase 1: Use What's Available (Now)
- Use **Rotation Vector (0x05)** at maximum stable rate (~100Hz on I2C)
- This provides magnetometer-fused orientation preventing yaw drift
- Sufficient for initial flight control experiments

### Phase 2: Implement ARVR Stabilized (High Priority)
- Implement **ARVR Stabilized Rotation Vector (0x28)**
- Smoother corrections, no discontinuities during motion
- Better for active flight control loops

### Phase 3: High-Frequency Path (Future)
- Implement **Gyro Integrated Rotation Vector (0x2A)** on Channel 5
- Enables ~1000Hz updates for prediction/fast response
- Requires Channel 5 implementation

### Phase 4: Consider SPI (Optional)
- If I2C bandwidth becomes limiting
- 8x faster, enables higher update rates

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
- Measure latency and accuracy at 10Hz, 50Hz, 100Hz
- Establish baseline for comparison

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
- [ ] Latency < 10ms at 100Hz update rate
- [ ] Angle error < 1° during slow motion
- [ ] Angle error < 3° during fast motion
- [ ] Yaw drift < 1°/minute with magnetometer fusion

## References

- [Adafruit BNO085 Guide](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085)
- [SH-2 Reference Manual](https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf)
- `specification/IMU BNO08x v1.17.pdf` - Full datasheet
- `specification/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf` - Calibration guide
- Base driver: https://github.com/bradcar/bno08x_i2c_spi_MicroPython
- AS5600 reference encoder: https://github.com/c0ffee2code/AS5600
