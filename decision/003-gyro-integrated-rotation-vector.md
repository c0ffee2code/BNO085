# ADR-003: Gyro Integrated Rotation Vector (0x2A) Implementation

**Date:** 2026-02-12
**Status:** Accepted
**Context:** Flight control test bench — adding highest-frequency orientation output

## Decision Drivers

1. **Lowest latency orientation** — Gyro Integrated RV provides up to 1000 Hz updates, 2.5x faster than the 400 Hz max of standard rotation vectors
2. **Dedicated transport channel** — Channel 5 bypasses the normal sensor report pipeline, allowing the host to prioritize orientation data
3. **Minimal header overhead** — no report ID, sequence, status, or delay bytes, just 14 bytes of raw sensor data per report
4. **Configurable fusion source** — can be based on Rotation Vector (with mag) or Game Rotation Vector (without mag) via FRS record

## Background

The BNO08X sensor provides a special high-speed orientation output called the Gyro Integrated Rotation Vector. Unlike standard sensor reports that arrive on Channel 3 with a 5-byte header (report ID, sequence, status/accuracy, delay), this report uses a dedicated SHTP Channel 5 with no header overhead.

The sensor's internal gyroscope drives the fast updates, with periodic corrections from the full sensor fusion algorithm. This matches our flight control philosophy: "know approximate position frequently rather than precise position too late."

### Spec References

Read these pages in `specification/IMU BNO08x v1.17.pdf`:

| Page | Section | Content |
|------|---------|---------|
| 22 | 1.3.1 SHTP | Channel 5 = gyro rotation vector (dedicated channel) |
| 25 | 1.3.3 FRS records | `0xA1A2` = Gyro-Integrated RV configuration |
| 32 | 2.2.6 Gyro Rotation Vector | How it works: optimized processing path, configurable fusion source |
| 32-33 | 2.2.7 Gyro RV Prediction | Prediction tuning (alpha, beta, gamma), default 28ms lookahead |
| 50 | 6.8-6.9 Latency & Report Rates | Gyro RV: max 1000 Hz, latency 3.7 ms at 200 Hz / 6.6 ms at 100 Hz |
| 51 | 6.10 Power Consumption | 46.17 mW at 1000 Hz, 46.05 mW at 400 Hz |

### External References

- [SparkFun BNO080 Arduino Library](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/blob/main/src/SparkFun_BNO080_Arduino_Library.cpp) — reference implementation of Channel 5 parsing
- [SparkFun Gyro Integrated RV issue #32](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/32) — implementation discussion
- [SH-2 Reference Manual](https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf) — full protocol details

## Channel 5 Report Format

Channel 5 reports have **no standard header** — no report ID, sequence number, status, or delay bytes. The 14-byte payload is pure sensor data:

| Bytes | Field | Type | Q-point | Scalar |
|-------|-------|------|---------|--------|
| 0-1 | Quaternion I | int16 | Q14 | 2^-14 = 6.1035e-05 |
| 2-3 | Quaternion J | int16 | Q14 | 2^-14 |
| 4-5 | Quaternion K | int16 | Q14 | 2^-14 |
| 6-7 | Quaternion Real | int16 | Q14 | 2^-14 |
| 8-9 | Angular Velocity X | int16 | Q10 | 2^-10 = 9.7656e-04 |
| 10-11 | Angular Velocity Y | int16 | Q10 | 2^-10 |
| 12-13 | Angular Velocity Z | int16 | Q10 | 2^-10 |

**Important:** Quaternion order on the wire is `(i, j, k, real)`. The driver convention (and standard math convention) is `(real, i, j, k)` — reordering is required when storing.

### Comparison with Standard Reports (Channel 3)

| Aspect | Standard Report (Ch 3) | Gyro Integrated RV (Ch 5) |
|--------|----------------------|--------------------------|
| Header bytes | 4 (report ID, seq, status/accuracy, delay) | 0 |
| Payload bytes | 10-14 (varies by report) | 14 (fixed) |
| Accuracy field | Yes (2 bits in status byte) | No |
| Delay field | Yes (14 bits, 100 us resolution) | No |
| Sequence number | Yes (for drop detection) | No |
| Timestamp | Derived from base timestamp + delay | Host interrupt time only |
| Max rate | 400 Hz (rotation vectors) | 1000 Hz |
| Contains angular velocity | No (separate gyro report needed) | Yes (built-in) |

## Current Driver State

The driver has 0x2A partially wired:

| Component | Status | Location |
|-----------|--------|----------|
| Constant `BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR` | Defined | `bno08x.py:197` |
| Report length (14 bytes) | Registered | `_REPORT_LENGTHS` |
| Scaling entry (Q14, 4-tuple) | Registered (Ch 3 slow path, unused) | `_SENSOR_SCALING` |
| `_Q_POINT_10_SCALAR` | Implemented | `bno08x.py:302` |
| Default frequency (1000 Hz) | Implemented | `DEFAULT_REPORT_FREQ` |
| Channel 5 inline parsing | Implemented | `update_sensors()` |
| Initial report value (8-tuple) | Implemented | `_INITIAL_REPORTS` |
| `enable_feature()` | Works — standard Set Feature Command | `bno08x.py:1479` |
| Property accessor `gyro_integrated_rotation_vector` | Implemented | `bno08x.py` |

## Implementation Plan

### 1. Add Q10 scalar constant

```python
_Q_POINT_10_SCALAR = 2 ** (10 * -1)  # for angular velocity in gyro integrated RV
```

### 2. Parse Channel 5 in `update_sensors()`

Replace the `NotImplementedError` with inline parsing. Channel 5 reports are always single-report packets (no multi-report splitting needed). Parse the 7 int16 values directly from the payload, apply Q-point scaling, reorder quaternion to `(real, i, j, k)`, and store with host timestamp.

### 3. Store data in `_report_values[0x2A]`

Tuple format: `(qr, qi, qj, qk, angvel_x, angvel_y, angvel_z, timestamp)`

- Quaternion: Q14 scaled, reordered to `(real, i, j, k)`
- Angular velocity: Q10 scaled, in rad/s
- Timestamp: `ms_at_interrupt - epoch_start_ms` (no base timestamp or delay available)

### 4. Add property accessor

```python
@property
def gyro_integrated_rotation_vector(self):
    """Gyro Integrated Rotation Vector — quaternion (real, i, j, k) + angular velocity (x, y, z)"""
    ...
```

### 5. Update default frequency

Change from 10 Hz to 1000 Hz (the whole point of this report type).

## Risks and Considerations

### I2C bandwidth saturation
Our current achieved rate is ~280 Hz at 344 Hz requested. At 1000 Hz the I2C bus will likely saturate. The theoretical I2C max for this sensor is 344 Hz for quaternion data. **Expect actual rates well below 1000 Hz on I2C** — SPI may be needed to get the full benefit. Testing will reveal the actual ceiling.

### GC stall impact at high rates
The existing ~43 ms periodic lag spikes would cause ~43 dropped samples at 1000 Hz. GC mitigation (Milestone 4) becomes more critical at higher rates. However, the Gyro Integrated RV's lack of sequence numbers means we can't even detect drops — we just get the latest value.

### FRS configuration for Game RV fusion source
By default, the Gyro Integrated RV fuses based on the standard Rotation Vector (with magnetometer). To use Game Rotation Vector as the fusion source (our best performer for roll-axis tracking — see ADR-001 Experiment 1), we would need to write FRS record `0xA1A2`. FRS write is not yet implemented in the driver. This is optional — the default configuration is functional for initial testing.

### No accuracy or sequence fields
Channel 5 reports provide no accuracy status and no sequence number. We cannot detect dropped samples or assess fusion quality from the report itself. The host must infer data quality from update rate consistency and quaternion norm checks.

## Test Results — Iteration 1

Test bench: BNO085 on I2C (400 kHz), AS5600 encoder reference, Pico 2 host.
Test script: `tests/report_rate/test_gyro_integrated_rotation_vector.py`
Results: `tests/report_rate/results/gyro_integrated_vector/iteration_1/`

### Update Rate

| Metric | Value |
|--------|-------|
| Requested rate | 1000 Hz |
| Achieved rate | ~333 Hz (2000 samples / ~7.1 s) |
| Typical sample interval | 3 ms |
| Periodic gaps | ~40 ms (I2C bus contention or report batching) |

The I2C bus caps the effective rate at ~333 Hz, consistent with the known 280–344 Hz ceiling for quaternion-class reports. SPI would be needed for the full 1000 Hz.

### Latency

| Metric | Value |
|--------|-------|
| Typical lag | 0 ms |
| Max lag (steady state) | 1 ms |
| First-sample lag | 2–3 ms (startup settling) |
| Samples with lag > 0 ms | ~5% |

Near-zero latency confirms the dedicated Channel 5 bypasses the normal sensor pipeline buffering.

### Static Hold (stability)

| Metric | Value |
|--------|-------|
| IMU drift over 7 s | 0.04 deg (62.65 → 62.61) |
| Angular velocity at rest | Settles to (0.0, 0.0, 0.0) within ~2 s |
| Encoder hold | 59.41 deg (stable) |
| ENC–IMU fixed offset | ~3.2 deg (calibration offset, expected) |

Quaternion output is stable at rest with no jitter. Angular velocity correctly zeros.

### Rapid Moves (tracking)

| Metric | Value |
|--------|-------|
| Encoder sweep range | ~120 deg |
| ENC–IMU tracking offset | ~5 deg (consistent fixed offset) |
| Peak angular velocity | ~4.6 rad/s (~264 deg/s) |
| AngVelZ during single-axis roll | Near 0 (correct axis isolation) |

IMU tracks encoder motion with a consistent fixed offset. Angular velocity responds proportionally to motion speed and zeros when motion stops.

### Conclusions

1. **Channel 5 parsing works correctly** — quaternion and angular velocity values are physically reasonable
2. **Latency is excellent** — 0–1 ms vs the typical 2–5 ms on Channel 3 reports
3. **Rate is I2C-limited** — 333 Hz achieved, consistent with bus bandwidth ceiling
4. **Static stability is good** — 0.04 deg drift in 7 s, clean angular velocity zeroing
5. **~40 ms gaps** appear periodically — likely the sensor batching when I2C can't drain fast enough; worth investigating whether these represent missed samples or delayed delivery
