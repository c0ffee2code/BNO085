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

### Tare run (Milestone 2) — first attempt

Before/after comparison with lever fixed at zero, encoder as reference:

| Phase | Encoder | IMU roll | Bias (IMU - ENC) |
|-------|---------|----------|-----------------|
| Before tare | 0.09 deg | 0.58 deg | **0.49 deg** |
| After tare | 0.09 deg | 0.01 deg | **0.08 deg** |

Tare eliminated the systematic bias from ~0.5 deg to noise floor (~0.08 deg).

### Tare run (Milestone 2) — second attempt, after full calibration and bug fixes

Static hold, lever fixed at zero, 2000 samples per phase:

| Metric | Before tare | After tare |
|--------|-------------|------------|
| MAE | 3.45 deg | 0.96 deg |
| Bias (ENC−IMU) | +3.45 deg | −0.96 deg |
| Achieved rate | 250.7 Hz | 248.3 Hz |
| Mean lag | 4.51 ms | 4.90 ms |
| Max lag | 46.2 ms | 396.8 ms (first 2 samples only — tare response settling) |

The remaining −0.96 deg post-tare "bias" is **not a tare error** — it is the encoder center
offset. At the physical zero position, `ENC_RAW = 411` but `AXIS_CENTER` was set to 422.
Offset: `(411 − 422) × (360/4096) = −0.97 deg`, which exactly matches the residual.
After tare the IMU correctly reads 0.00 deg (zeroed to the physical position). The encoder
reads −0.97 deg because its zero was miscalibrated. Fix: `AXIS_CENTER` corrected from 422
to 411 in `test_tare_and_measure.py` and `report_rate_test.py`.

### Magnetometer environment dependency

After power cycle, the magnetometer stayed at accuracy 0 for 30+ seconds despite having DCD saved. Root cause: calibration was performed with the IMU detached (magnetically clean area), then remounted on the test bench near motors/ESCs/metal — a different magnetic environment. The DCD contains hard/soft iron corrections for the calibration environment, which don't match the test bench environment.

This confirmed the spec's warning (Calibration Procedure p2, §1.1) and validated our original hypothesis that for roll-axis testing, the magnetometer isn't critical — roll is driven by accel+gyro. The tare produced correct results even with mag accuracy 0.

For yaw-axis testing, calibration must be performed in the same magnetic environment where the device will operate.

## Post-Calibration Notes

- DCD persists across power cycles — no need to recalibrate on every boot
- Recalibrate if the device moves to a different magnetic environment
- For AR/VR use, CEVA recommends disabling gyro and mag dynamic calibration during normal operation to avoid unwanted corrections during motion. For our flight control bench, we keep defaults (accel+mag dynamic calibration enabled, gyro enabled for hand-held)
- After calibration, proceed to tare (Milestone 2) to establish the reference frame in the final mounting position

## Tare Integration Debugging (Milestone 2)

Issues discovered and fixed when integrating calibration into the tare workflow:

### Bug 1 — Mag accuracy stuck at 0 in tare script

**Symptom:** `test_tare_and_measure.py` always reported magnetometer accuracy = 0, even
immediately after running `test_calibration_mag.py` which ended at accuracy >= 2.

**Root cause:** The tare script enabled the magnetic sensor but never called `begin_calibration()`
(Configure ME Calibration, SH-2 §6.4.7.1). The ME accuracy tracking routine is **inactive by
default** after power-on. DCD is loaded from flash (giving the sensor a starting point) but the
ME does not track or report live accuracy unless explicitly told to via P0/P1/P2 enable bits.

**Fix:** Add `imu.begin_calibration()` in the tare script before the accuracy check loop.
Use `begin_calibration()` (all three enabled) rather than `begin_mag_calibration()` — see Bug 2.

### Bug 2 — `save_calibration_data()` fails with status=4

**Symptom:** After mag accuracy reached 2, calling `save_calibration_data()` raised
`RuntimeError: Unable to save calibration data, status=4`.

**Root cause:** `begin_mag_calibration()` sends Configure ME Calibration with P0=0 (accel
disabled), P1=0 (gyro disabled), P2=1 (mag enabled). This tells the ME to **stop tracking**
accel and gyro. Their in-RAM accuracy drops to 0. When `save_calibration_data()` is called, the
sensor sees accel and gyro as uncalibrated and rejects the save.

**Fix:** Use `begin_calibration()` (all three ME routines active) instead of
`begin_mag_calibration()` in the tare script. All three routines re-engage from stored DCD;
accel and gyro recover quickly without user action while mag needs the figure-8.
Additionally, `save_calibration_data()` does not belong in the tare script — that is
`test_calibration_mag.py`'s responsibility. Removed the save call from the tare script.

### Bug 3 — OSError EIO after tare, during post-tare data collection

**Symptom:** `collect_samples("/data/tare_after.csv")` crashed with `OSError: [Errno 5] EIO`
on the first `imu.update_sensors()` call after the tare was applied.

**Root cause:** SH-2 Reference Manual Figure 44 (Command Identifiers) lists Tare (ID=3) as
**"Command and Response"**. The sensor sends back a 0xF1 Command Response after processing
every tare subcommand (Tare Now, Persist Tare, Set Reorientation). The driver had no
`elif command == _ME_TARE_COMMAND` branch in `_process_control_report`. The unhandled 16-byte
response packet sat unread on the I2C bus. The next `update_sensors()` call read this stale
packet instead of a fresh sensor report → I2C frame out of sync → EIO.

This was incorrectly diagnosed earlier as "fire-and-forget" based on sections 6.4.4.1 and
6.4.4.2 which only show the request format. The command identifiers table (§6.4, Figure 44)
is the authoritative source for which commands generate responses.

**Fix:** Three driver changes:
1. Added `self._tare_completed_at: float = -1.0` flag in `__init__`
2. Added `elif command == _ME_TARE_COMMAND` handler in `_process_control_report` that sets
   `_tare_completed_at = ticks_ms()`
3. Changed `_send_tare_command` from fire-and-forget to waiting for `_tare_completed_at`,
   matching the pattern used by `save_calibration_data` for DCD responses

### Bug 4 — Rotation Vector basis corrupts tare when mag_acc=0

**Symptom:** After tare, the test bench is permanently at ~45 deg tilt despite the IMU
roll reading correctly (~0 deg) during the pre-tare settle phase.

**Root cause:** `imu.tare(0x07, 0)` uses Rotation Vector (basis=0) as the tare reference
frame. The Rotation Vector fuses accelerometer, gyroscope **and magnetometer**. When the
bench is in a different magnetic environment from where the magnetometer was last calibrated
(near motors, ESCs, or metal structures), `mag_acc=0` throughout the settle period.
The BNO085 still attempts to use the magnetometer heading — which points to an arbitrary
"North" — and bakes that arbitrary ~45 deg heading into the tare quaternion. After tare,
all Rotation Vector outputs are referenced to this corrupted frame.

**Fix:** Change tare basis from 0 (Rotation Vector) to 1 (Game Rotation Vector):

```python
imu.tare(0x07, 1)  # basis=1 = Game Rotation Vector, no magnetometer
```

Switch all data collection from `imu.quaternion` (Rotation Vector) to
`imu.game_quaternion` (Game Rotation Vector) for consistency. Game RV was already proven
as the better dynamic performer (0.999 vs 0.93–0.96 correlation, ADR-001 Experiment 1).

**Simplification:** With Game RV basis, magnetometer accuracy is no longer a prerequisite
for tare. The entire Phase 0 (mag cal figure-8, sensor detached) is eliminated from
`test_tare_and_measure.py`. The sensor reset between phases is also eliminated (it was
only needed to clear Phase 0 gyro drift). The procedure reduces to two phases:

1. **Phase 1 — sensor attached:** Enable `game_quaternion` at 344 Hz, settle 10 s, collect
   pre-tare samples.
2. **Phase 2 — tare + verify:** Apply tare (basis=1), drain response, collect post-tare
   samples, optionally persist.

For yaw accuracy (absolute heading), recalibrate the magnetometer in the operating
environment with `tests/calibration/test_calibration_mag.py` and switch back to basis=0.

### Tare procedure — physical phases

Mag calibration (figure-8 motion) is physically incompatible with the sensor being fixed to
the test bench. The original tare procedure was split into three phases in
`test_tare_and_measure.py` (Phase 0 detached, Phase 1 attached + reset, Phase 2 tare).

After discovering Bug 4, the procedure was simplified to **two phases** using Game Rotation
Vector basis (basis=1, no magnetometer):

1. **Phase 1 — sensor attached:** Enable `game_quaternion` at 344 Hz, settle 10 s,
   collect pre-tare samples. No mag cal or sensor reset needed.
2. **Phase 2 — tare + verify:** `imu.tare(0x07, 1)`, drain response packets, collect
   post-tare samples, optionally persist.

### Individual calibration scripts

Per-sensor calibration scripts added to avoid full recalibration when only one sensor degrades:

| Script | ME flags | Sensors enabled | Trigger |
|--------|----------|-----------------|---------|
| `test_calibration_accel.py` | accel=1, gyro=0, mag=0 | acceleration + game_quaternion | PCB remounted |
| `test_calibration_gyro.py` | accel=0, gyro=1, mag=0 | gyro + game_quaternion | Temperature change, power cycle |
| `test_calibration_mag.py` | accel=0, gyro=0, mag=1 | magnetic (50 Hz) + quaternion | New room, tare shows acc < 2 |

Note: individual scripts use `begin_mag/accel/gyro_calibration()`. These are safe to call
standalone because the goal IS to save DCD with only that sensor tracked — valid when the
other two sensors are already well-calibrated and the intent is a targeted update.