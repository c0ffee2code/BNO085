# ADR-004: BNO085 Sensor Calibration Procedure

**Date:** 2026-02-16
**Status:** Accepted
**Context:** Flight control test bench — first-time sensor calibration before tare

## Decision Drivers

1. **Prerequisite for tare** — Tare (Milestone 2) requires calibrated sensors to compute a correct reference frame. Without calibration, all-axes tare produces broken coordinate systems (inverted roll axis observed in testing).
2. **Per-device requirement** — Each BNO085 has unique MEMS sensor characteristics that change after board placement. Factory calibration on the final assembled board is essential.
3. **Environment-dependent magnetometer** — The magnetometer calibrates for the local magnetic field (hard/soft iron effects). Must recalibrate when the device moves to a different environment.

## Background

The BNO085 contains three MEMS sensors (accelerometer, gyroscope, magnetometer) that require dynamic calibration. CEVA's firmware performs calibration by observing specific physical motions and computing bias, offset, and scale corrections. These corrections are stored in the Dynamic Calibration Data (DCD) file in BNO085 flash, persisting across power cycles.

### Calibration is per-sensor, not per-report

Calibration operates at the **physical sensor level**. The DCD stores corrections for the three MEMS sensors. All fusion outputs (Rotation Vector, Game Rotation Vector, Gyro Integrated RV, Geomagnetic RV, etc.) automatically benefit from the same calibration. There is no need to calibrate separately for different report types.

### Spec References

| Source | Section | Content |
|--------|---------|---------|
| `specification/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf` | 2.1 | Magnetic environment requirements |
| Same | 2.2 | Factory calibration procedure: configuration + motions |
| Same | 3.0 | User calibration (same procedure as factory) |
| `specification/SH-2-Reference-Manual-v1.2.pdf` | 6.4.6 | Configure ME Calibration command (begin/save) |
| `specification/IMU BNO08x v1.17.pdf` | Section 3 | Calibration and interpretation |

## Calibration Configuration (Spec Section 2.2)

The spec requires four things enabled before starting calibration motions:

| Requirement | Spec source | Purpose |
|-------------|-------------|---------|
| Configure ME Calibration (accel=1, gyro=1, mag=1) | Calibration Procedure p3, §2.2 | Enable dynamic calibration for all three sensors |
| Game Rotation Vector output | Calibration Procedure p3, §2.2: *"Enable Game Rotation Vector output"* | Fusion algorithm must run for calibration to converge |
| Magnetic Field at **50 Hz** | Calibration Procedure p3, §2.2: *"the magnetometer requires 50Hz report rate to get proper calibration"* | Magnetometer needs this rate for proper calibration |
| Accelerometer + Gyroscope outputs | Calibration Procedure p3, §2.2 | Monitor accuracy status bits |

### Accuracy scale (status bits)

| Value | Meaning |
|-------|---------|
| 0 | Unreliable |
| 1 | Low accuracy |
| 2 | Medium accuracy (minimum target) |
| 3 | High accuracy |

## Calibration Motions

Performed in a magnetically clean area (away from PC towers, monitors, metal furniture).

| Sensor | Motion | Duration |
|--------|--------|----------|
| Accelerometer | Hold device in 4-6 unique orientations ("cube method" — each face down) | ~1s per face |
| Gyroscope | Set device on flat surface, leave still | 2-3 seconds |
| Magnetometer | Rotate ~180 deg and back on each axis (roll, pitch, yaw) | ~2s per rotation, repeat until accuracy >= 2 |

Order: accel first, then gyro, then magnetometer. The cube method does not require all 6 faces or precise alignment — 4-5 faces in roughly distinct orientations is sufficient.

## Implementation: `tests/calibration/test_calibration.py`

### Driver methods used

| Method | SH-2 Command | What it does |
|--------|-------------|--------------|
| `begin_calibration()` | Configure ME Calibration (6.4.6.1) | Enables dynamic calibration for accel, gyro, mag |
| `calibration_status()` | Configure ME Calibration with GET_CAL flag | Confirms BNO085 accepted the calibration request |
| `save_calibration_data()` | Save DCD (6.4.7) | Writes calibration to flash; persists across power cycles |

### Script flow

1. Enable sensors: acceleration (20 Hz), gyro (20 Hz), magnetic (50 Hz), **game_quaternion (20 Hz)**
2. `begin_calibration()` — send Configure ME Calibration command
3. `calibration_status()` — wait for acknowledgment
4. Monitor loop: print accuracy every 200 ms, guide user through motions
5. When all three sensors hold accuracy >= 2 for 5 seconds, exit loop
6. `save_calibration_data()` — persist DCD to flash

### Gap found: Game Rotation Vector was missing

The spec requires Game Rotation Vector to be enabled during calibration. The bradcar reference example and our initial script both omitted it. Fix: add `imu.game_quaternion.enable(20)` after the sensor enables.

Bradcar's example also uses 20 Hz for the magnetometer — the spec requires 50 Hz. Our script already had this correct.

## Test Results

### Calibration run

All sensors reached target accuracy and held stable for 5 seconds:

| Sensor | Accuracy achieved |
|--------|------------------|
| Accelerometer | 2 (medium) |
| Gyroscope | 3 (high) |
| Magnetometer | 3 (high) |

DCD saved to flash successfully. Step-by-step prompts with non-blocking `select.select()` stdin polling kept `update_sensors()` running between steps — without this, the gyro stayed at accuracy 0 because the SHTP buffers filled during blocking `input()` calls.

### Tare run (Milestone 2)

Before/after comparison with lever fixed at zero, encoder as reference:

| Phase | Encoder | IMU roll | Bias (IMU - ENC) |
|-------|---------|----------|-----------------|
| Before tare | 0.09 deg | 0.58 deg | **0.49 deg** |
| After tare | 0.09 deg | 0.01 deg | **0.08 deg** |

Tare eliminated the systematic bias from ~0.5 deg to noise floor (~0.08 deg).

### Magnetometer environment dependency

After power cycle, the magnetometer stayed at accuracy 0 for 30+ seconds despite having DCD saved. Root cause: calibration was performed with the IMU detached (magnetically clean area), then remounted on the test bench near motors/ESCs/metal — a different magnetic environment. The DCD contains hard/soft iron corrections for the calibration environment, which don't match the test bench environment.

This confirmed the spec's warning (Calibration Procedure p2, §1.1) and validated our original hypothesis that for roll-axis testing, the magnetometer isn't critical — roll is driven by accel+gyro. The tare produced correct results even with mag accuracy 0.

For yaw-axis testing, calibration must be performed in the same magnetic environment where the device will operate.

## Post-Calibration Notes

- DCD persists across power cycles — no need to recalibrate on every boot
- Recalibrate if the device moves to a different magnetic environment
- For AR/VR use, CEVA recommends disabling gyro and mag dynamic calibration during normal operation to avoid unwanted corrections during motion. For our flight control bench, we keep defaults (accel+mag dynamic calibration enabled, gyro enabled for hand-held)
- After calibration, proceed to tare (Milestone 2) to establish the reference frame in the final mounting position