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
| Game Rotation Vector | 0x08 | Yes | **IMPLEMENTED** | **HIGH** | No mag, will drift but smooth. **Best performer for roll-axis tracking without mag calibration** |
| Geomagnetic Rotation Vector | 0x09 | Yes | **IMPLEMENTED** | LOW | Low power, slow response |
| ARVR Stabilized Rotation Vector | 0x28 | Yes | **NOT IMPLEMENTED** | **HIGH** | Smooth corrections during motion, ~250Hz |
| ARVR Stabilized Game Rotation Vector | 0x29 | Yes | **NOT IMPLEMENTED** | MEDIUM | Smooth, no mag |
| Gyro Integrated Rotation Vector | 0x2A | Yes | **PARTIAL** | **HIGH** | Fastest updates ~1000Hz, for prediction |

### Flight Control Relevance

**Best for our use case:**
1. **Game Rotation Vector (0x08)** - Already implemented. **Best performer for roll-axis tracking** — 0.999 correlation, consistent ~3 deg bias (removable by tare), no range compression. No magnetometer means potential yaw drift, but irrelevant for roll control.

2. **Rotation Vector (0x05)** - Already implemented. Uses all 3 sensors, provides absolute heading referenced to magnetic north. Prevents yaw drift. **However, unusable for dynamic tracking without magnetometer calibration** — tested at 16-20 deg MAE with chaotic non-constant bias during rapid motion. Must complete Milestone 1 (calibration) before this becomes viable.

3. **ARVR Stabilized Rotation Vector (0x28)** - **GAP**. Corrections applied during motion (not at rest), preventing visible jumps/discontinuities. Ideal for flight control where sudden orientation jumps would cause control instability.

4. **Gyro Integrated Rotation Vector (0x2A)** - **GAP**. Highest frequency output (~1000Hz on Channel 5). Gyro-only for fast response, with periodic corrections from full fusion. Perfect for "approximate position frequently" philosophy.

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
| SPI | Yes | **IMPLEMENTED** | MEDIUM | 8x faster than I2C.|
| UART | Yes | NOT IMPLEMENTED | LOW | UART-SHTP (not RVC) at 3 Mbaud, but multi-packet reads are incomplete. No benefit over SPI for our use case |
| Channel 5 (High-speed Gyro) | Yes | **IMPLEMENTED** | HIGH | Required for Gyro Integrated RV |

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

### Milestone 3: Game Rotation Vector (0x08) comparison — DONE
- [x] Run rapid moves and static hold with Game RV at 344 Hz (2 iterations each)
- [x] Run rapid moves and static hold with Rotation Vector at 344 Hz (2 iterations each, corrected orientation)
- [x] Compare accuracy vs Rotation Vector (0x05) on roll axis — Game RV wins decisively (MAE 3 deg vs 16-20 deg during rapid motion)
- [ ] Measure drift rate over longer duration (no magnetometer = expected yaw drift) — deferred, requires longer test or separate setup
- No driver changes needed — already implemented

### Milestone 4: Timestamping QA — DONE
- [x] Audit driver timestamp implementation against SH-2 spec (sections 6.5.1, 7.2)
- [x] Fix signed parsing: replace unsigned byte assembly with `unpack_from("<i")` at all 5 sites
- [x] Fix rebase accumulation: 0xFA handler uses `+=` instead of `=` per SH-2 7.2.2
- [x] Regression test: static hold at 344 Hz confirms no change in normal operation
- QA report: `qa_report/timestamping.md`

### Milestone 5: GC stall mitigation
- [ ] Profile the ~45 ms periodic spikes (confirm they are MicroPython GC)
- [ ] Pre-allocate buffers, use manual `gc.collect()` at safe points
- [ ] Re-test at 344 Hz, target achieved rate closer to 344 Hz with no spikes >10 ms

### Milestone 6: ARVR Stabilized Rotation Vector (0x28)
- [ ] Implement report 0x28 in driver
- [ ] Run rapid moves test, compare discontinuity behaviour vs standard RV
- [ ] Evaluate for flight control loop suitability

### Milestone 7: Gyro Integrated Rotation Vector (0x2A) — PARTIAL
- [x] Implement Channel 5 (high-speed gyro) support in driver
- [x] Complete report 0x2A implementation and parsing
- [x] Test on I2C — achieves ~333 Hz (I2C bandwidth limit, spec max is 1000 Hz)
- [ ] Test on SPI to reach full 1000 Hz potential
- See ADR-003 for details

### Milestone 8: SPI interface
- [x] SPI driver available — copied from BradCar repo (`driver/spi.py`)
- [ ] Test SPI vs I2C at 344 Hz — measure if achieved rate improves (currently ~277-287 Hz on I2C)
- [ ] Test Gyro Integrated RV at 1000 Hz over SPI
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

#### Experiment 1: Rotation Vector (0x05) vs Game Rotation Vector (0x08)

Both vectors tested head-to-head with corrected IMU orientation (signs match encoder).
All tests at 344 Hz (max I2C rate). 10 Hz was tested early on and dropped — it has no
value for flight control (100 ms sample interval misses gradual changes, producing stale
readings with 6.93 deg MAE during slow motion). Each scenario run twice (2 iterations)
to confirm reproducibility.

Test scripts: `tests/report_rate/test_baseline_rotation_vector.py`, `tests/report_rate/test_game_rotation_vector.py`
Analysis script: `tests/report_rate/analysis/analyse_report_rate.py`
Test scenarios: `tests/report_rate/results/scenario_static_hold.txt`, `tests/report_rate/results/scenario_rapid_moves.txt`

**Test 1a — Static hold** (lever fixed with screwdriver)

Raw data:
- `tests/report_rate/results/rotation_vector/iteration_1/static_hold/rot_vec_344hz_2000s.csv`
- `tests/report_rate/results/rotation_vector/iteration_2/static_hold/rot_vec_344hz_2000s.csv`
- `tests/report_rate/results/game_rotation_vector/iteration_1/static_hold/game_rot_vec_344hz_2000s.csv`
- `tests/report_rate/results/game_rotation_vector/iteration_2/static_hold/game_rot_vec_344hz_2000s.csv`

| Metric | Rotation Vector (i1 / i2) | Game Rotation Vector (i1 / i2) |
|--------|---------------------------|-------------------------------|
| Bias (ENC-IMU) | -3.50 / -3.24 deg | -3.39 / -2.93 deg |
| IMU noise floor (range) | 0.21 / 0.10 deg | 0.07 / 0.10 deg |
| Achieved rate | 279 / 279 Hz | 286 / 287 Hz |
| Lag mean | 3.74 / 3.73 ms | 3.66 / 3.65 ms |
| Lag max | 44.2 / 43.3 ms | 44.3 / 42.9 ms |

**Test 1b — Rapid moves** (~1 s per full sweep)

Raw data:
- `tests/report_rate/results/rotation_vector/iteration_1/rapid_moves/rot_vec_344hz_2000s.csv`
- `tests/report_rate/results/rotation_vector/iteration_2/rapid_moves/rot_vec_344hz_2000s.csv`
- `tests/report_rate/results/game_rotation_vector/iteration_1/rapid_moves/game_rot_vec_344hz_2000s.csv`
- `tests/report_rate/results/game_rotation_vector/iteration_2/rapid_moves/game_rot_vec_344hz_2000s.csv`

| Metric | Rotation Vector (i1 / i2) | Game Rotation Vector (i1 / i2) |
|--------|---------------------------|-------------------------------|
| Pearson correlation | 0.963 / 0.927 | **0.999 / 0.999** |
| MAE overall | 15.82 / 20.21 deg | **3.01 / 3.74 deg** |
| MAE fast motion (top 25% vel) | 11.47 / 13.38 deg | **3.23 / 3.12 deg** |
| MAE slow motion (bottom 25% vel) | 18.86 / 25.30 deg | **3.31 / 3.78 deg** |
| RMS error | 19.67 / 24.06 deg | **3.49 / 4.19 deg** |
| Max error | 47.59 / 49.76 deg | **20.32 / 23.89 deg** |
| Bias (ENC-IMU) | -15.09 / -13.00 deg | -2.98 / -3.71 deg |
| IMU trails motion (%) | 49.6 / 51.6 | 59.6 / 54.8 |
| Encoder range | 123.1 / 123.8 deg | 124.0 / 123.3 deg |
| IMU range | **140.2 / 128.7 deg (overshoot)** | 122.4 / 121.8 deg (matches encoder) |
| Achieved rate | 277 / 277 Hz | **283 / 283 Hz** |
| Lag mean | 3.61 / 3.58 ms | 3.51 / 3.55 ms |
| Lag max | 42.4 / 43.3 ms | 44.2 / 43.0 ms |

**Findings:**

1. **At rest, both vectors are nearly identical.** ~3 deg systematic bias (fixed offset, removable by tare), excellent noise floor (0.07-0.21 deg), similar lag profiles. No meaningful difference when stationary.

2. **During rapid motion, Game Rotation Vector dramatically outperforms Rotation Vector.** Game RV maintains 0.999 correlation and ~3 deg consistent bias across both iterations. Rotation Vector drops to 0.93-0.96 correlation with 16-20 deg MAE — 5x worse.

3. **Rotation Vector bias becomes chaotic under motion.** At rest the bias is a stable -3.2 to -3.5 deg. During rapid moves it varies from ~1.5 deg to ~46 deg within a single test — the magnetometer fusion correction fights the gyro during fast direction changes. The mean bias (-13 to -15 deg) is meaningless because it is not constant.

4. **Rotation Vector overshoots encoder range.** IMU range (129-140 deg) exceeds encoder range (123 deg) — the mag corrections cause the sensor to swing past the true angle. Game RV range (122 deg) matches the encoder.

5. **Magnetometer calibration is the root cause.** Without a properly calibrated magnetometer, the rotation vector's mag fusion introduces a non-constant, motion-dependent error that makes the sensor unreliable for dynamic tracking. The game rotation vector avoids this entirely by not using the magnetometer.

6. **Results are reproducible.** Iteration 2 confirms iteration 1 patterns. The rotation vector is actually *worse* in iteration 2 (MAE 20 vs 16 deg), showing the magnetometer interference is unpredictable.

7. **Game RV achieves ~6 Hz higher rate** (283 vs 277 Hz) — less computation per sample without magnetometer fusion.

8. **Periodic ~43 ms lag spikes** present in both vectors, dragging achieved rate to ~277-287 Hz. Likely MicroPython garbage collection stalls.

**Conclusion:** Game Rotation Vector (0x08) is the clear winner for roll-axis flight control without magnetometer calibration. It tracks motion faithfully (0.999 correlation), maintains a consistent ~3 deg bias removable by tare, and matches encoder range with no overshoot. The Rotation Vector (0x05) is unusable for dynamic tracking until magnetometer calibration (Milestone 1) is completed — after which it should be re-tested to see if the mag fusion becomes an asset rather than a liability. For heading-hold modes where yaw drift matters, the Rotation Vector remains the only option that provides absolute heading reference.

#### Experiment 2: ARVR Stabilized (0x28) - After Implementation
- Compare jump/discontinuity behavior during motion
- Measure if corrections are smoother than standard rotation vector

#### Experiment 3: Gyro Integrated (0x2A) - After Implementation
- Measure latency at high frequency
- Quantify accuracy vs speed tradeoff

### Success Criteria

For flight control viability (evaluated using Game Rotation Vector, the better performer):
- [x] Latency < 10ms at 100Hz update rate — **PASS** (3.5-3.7 ms mean at 344 Hz; periodic 43 ms spikes need GC mitigation)
- [ ] Angle error < 1° during slow motion — **LIKELY PASS after tare** (static noise floor is only 0.07-0.10 deg; the ~3 deg MAE is almost entirely systematic bias removable by tare)
- [ ] Angle error < 3° during fast motion — **LIKELY PASS after tare** (3.01-3.74 deg MAE with ~3 deg being systematic bias; after tare the dynamic tracking error should be well under 1 deg)
- [ ] Yaw drift < 1°/minute with magnetometer fusion — **NOT TESTED** (all tests measure roll axis only; requires separate test setup or magnetometer calibration first)

## References

- [Adafruit BNO085 Guide](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085)
- [SH-2 Reference Manual](https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf)
- `specification/IMU BNO08x v1.17.pdf` - Full datasheet
- `specification/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf` - Calibration guide
- Base driver: https://github.com/bradcar/bno08x_i2c_spi_MicroPython
- AS5600 reference encoder: https://github.com/c0ffee2code/AS5600
