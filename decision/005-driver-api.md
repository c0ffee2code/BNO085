# DR-005: BNO085 Driver API Redesign

**Date:** 2026-06-27
**Status:** Implemented
**Scope:** `src/bno08x.py`, `src/i2c.py`
**Supersedes:** driver API portions of ADR-001

---

## Context

A client runs two concurrent BNO085 sensor reports at different rates — for example,
Game Rotation Vector (GRV, 0x08) at 100 Hz for an outer control loop and Calibrated Gyroscope
(0x02) at 300 Hz for an inner loop. `update_sensors()` is called at the top of every inner
iteration.

Two structural problems in the driver created latent correctness and safety issues.

**Accuracy discarded at the API boundary.** Each SHTP packet carries a 2-bit fusion accuracy
field alongside the sensor values. The driver's `__iter__` interface returned only the data
values; accuracy was accessible only via `.full` / `.meta`, both of which had a destructive
side effect on the per-feature freshness counter. In practice accuracy was never read. When
elevated motor vibration corrupted the accelerometer inputs to the BNO's internal Kalman filter
mid-run, GRV accuracy dropped to 0 (Unreliable) — but the client had no way to see it and
continued tracking a corrupted attitude reference.

**Destructive freshness state.** The `.updated` / `_unread_report_count` mechanism was consumed
by any read, including reads for accuracy or timestamps. This conflated "has new data arrived?"
with "what is the current value?", and forced the client to gate on `update_sensors() > 0`
(any packet arrived) rather than "a gyro packet specifically arrived." The inner loop could
therefore execute against a stale gyro measurement on cycles where only a GRV or control-channel
packet was received.

---

## Decision

Replace the driver's `__iter__`-based, destructive-freshness API with six specific changes (R1–R6):

**R1 — Composite `SensorReading` namedtuple.** Every feature read returns
`SensorReading(data, accuracy, sensor_ts_ms, host_ts_ms)`. Data, accuracy, and both timestamps
are decoded from the same SHTP parse pass and never separated.

| Field | Type | Description |
|-------|------|-------------|
| `data` | tuple of float | Sensor values — shape is report-specific: `(x,y,z)` for gyro/accel; `(qr,qi,qj,qk)` for GRV; `(qr,qi,qj,qk,heading_acc_rad)` for RV and Geomag RV; `(qr,qi,qj,qk,ax,ay,az)` for GIRV |
| `accuracy` | int 0–3 or None | Per-report fusion accuracy. `None` for GIRV (channel 5), whose packet has no status byte |
| `sensor_ts_ms` | float | Sensor-epoch timestamp in ms: time elapsed since first BNO interrupt, incorporating the SHTP `delay` field |
| `host_ts_ms` | int | Host `ticks_ms()` at parse time — when the driver wrote this value, not the interrupt arrival |

**R2 — Driver is a pure state store.** Reading any feature never mutates driver state.
Freshness is determined by the client comparing `sensor_ts_ms` to the last value it acted on.
Remove `.updated` and `_unread_report_count`.

**R3 — `update_sensors()` contract.** Documented and guaranteed:
1. Drains all SHTP packets pending at call time.
2. Returns integer packet count: 0 = nothing processed, >0 = at least one packet processed.
3. For bundled delivery of the same report type, only the newest survives in `_report_values`.
4. Control-channel packets (feature responses, product ID) do not advance any feature's `sensor_ts_ms`.
5. Non-blocking when no interrupt is pending: returns 0 immediately.

**R4 — `SensorResetError` on spontaneous reset.** A Product ID Response (0xF8) received after
initialization raises `SensorResetError(cause_string)` rather than silently continuing. The
`_initialized` flag gates this: False during `reset_sensor()`, True only on successful return.

**R5 — Accuracy semantics documented and stable.** The 2-bit field is surfaced as-is; no
gating or interpretation in the driver. Values: 0 = Unreliable, 1 = Low, 2 = Medium, 3 = High.

**R6 — Remove Euler conversion from driver.** `euler`, `euler_full`, and `euler_conversion`
are removed. The driver is a pure data transport; angle extraction belongs in the client, which
knows its frame convention and applies the correct general formula.

---

## Consequences

### Positive

- **Accuracy is always in reach.** Every call site that reads a sensor value gets accuracy in
  the same object at no extra cost. The client can log it every row and gate on it without a
  separate read path.
- **Per-feature freshness is correct and non-destructive.** Each loop branch runs only when its
  specific report type actually arrived. Neither branch executes on cycles where only a
  control-channel response was processed.
- **Reset detection gets the right root cause.** When the BNO resets mid-operation,
  `SensorResetError` fires immediately with the cause code — not a timeout 100 ms later
  with an ambiguous "no packet" diagnosis.
- **Client owns its geometry.** Euler conversion lives in the client, where the frame
  convention, sign inversions, and formula choice are explicit and testable.
- **Immutable snapshots.** `SensorReading` is a namedtuple — immutable, cheap to pass around,
  safe to inspect from multiple call sites in the same loop iteration without copying.

### Negative

- **Existing call sites require migration.** Code using `__iter__` unpacking, `.updated`,
  or the Euler properties must be updated to use `.data`, `.accuracy`, and timestamp comparison.
- **Timestamp-based freshness is slightly more code.** Two `!= last_ts` comparisons per
  iteration instead of one `.updated` check. On a tight loop this is negligible; the
  correctness gain outweighs the verbosity.
- **`accuracy = None` for GIRV** requires a guard at any call site that reads accuracy from
  GIRV. This is a protocol constraint (channel 5 packets carry no status byte), not a design
  choice — but it must be handled explicitly.

---

## Options considered and rejected

### Keep `__iter__` and add a separate `.accuracy` property

The simplest change: leave the existing iteration protocol, add `.accuracy` as a non-destructive
property alongside `.updated`. Rejected because it does not fix the destructive-freshness problem
(R2). Accuracy would be accessible but freshness tracking would still be consumed by any `.full`
or `.meta` read, leaving the multi-feature loop ordering problem unsolved. It also keeps two
parallel read paths (iteration vs. composite), creating ongoing maintenance ambiguity.

### Replace `.updated` with a non-destructive boolean, keep `__iter__`

Make `.updated` non-destructive (cleared only by the driver on new packet arrival) and keep
`__iter__` for data. Rejected because a boolean flag cannot express *which* report type a packet
carried — only "at least one packet since last check." With two features at different rates, a
non-destructive boolean still cannot tell the client whether the packet that arrived was a gyro
or a GRV packet. Timestamp comparison (R2) solves this for all features simultaneously without
introducing per-feature cleared-by-read flags.

### Keep Euler in the driver with a frame-convention parameter

Add a `frame` argument to `euler_conversion` so the client can specify ENU, NED, or body-frame
output. Rejected because the conversion formula itself (not just the convention) must change for
correct general-case extraction. The driver formula (`atan2(qi, qr)` scaled) is the single-axis
approximation valid only when `qj` and `qk` ≈ 0 — mount misalignment or tare residuals are
enough to invalidate it. The correct general formula belongs in the client where it is visible
and testable.

---

## Client migration pattern

Before:

```python
# Freshness: gates on any packet arriving, regardless of type
if imu.update_sensors() == 0:
    sleep_us(500)
    continue

# Data access via __iter__: accuracy and timestamps discarded
gx, _gy, _gz       = imu.gyro
qr, qi, qj, qk     = imu.game_quaternion
```

After (R1–R6):

```python
if imu.update_sensors() == 0:
    sleep_us(500)
    continue

gyro = imu.gyro                               # SensorReading — never mutates driver state
if gyro.sensor_ts_ms != last_gyro_ts:         # new gyro data specifically
    last_gyro_ts = gyro.sensor_ts_ms
    gx = gyro.data[0]
    if gyro.accuracy == 0:
        handle_degraded_sensor()              # accuracy now visible at every call site

grv = imu.game_quaternion
if grv.sensor_ts_ms != last_grv_ts:          # new GRV data specifically
    last_grv_ts = grv.sensor_ts_ms
    qr, qi, qj, qk = grv.data
    roll = client_euler(qr, qi, qj, qk)      # R6: formula lives in client
```

Absolute staleness watchdog using `host_ts_ms` — detects sensor silence in wall-clock terms
even when sensor-epoch time has stopped advancing:

```python
if ticks_diff(ticks_ms(), imu.gyro.host_ts_ms) > STALE_LIMIT_MS:
    emergency_stop("gyro silent")
```

---

## Implementation findings

During implementation, a datasheet review (SH-2 Reference Manual §6.5.18, §6.5.20) found three
additional bugs not covered by R1–R6, fixed in the same pass:

**Q-point error on Geomagnetic RV (0x09).** `_SENSOR_SCALING[0x09]` used `_Q_POINT_12_SCALAR`
for the quaternion components. The SH-2 spec defines the quaternion Q point as 14; Q12 belongs
only to the heading accuracy estimate at bytes 12–13. Pre-fix quaternion norm ≈ 4.0; post-fix ≈ 1.0.

**Heading accuracy estimate silently dropped for RV (0x05) and Geomag RV (0x09).** Both reports
carry a 16-bit heading accuracy estimate at bytes 12–13 (Q12, radians). The original developer
had scaffolded `"e1": 12 | uctypes.INT16` in `_SENSOR_REPORT_LAYOUT` and a commented
`elif count == 5` stub in `_process_report`. Both fixes were completed: `_SENSOR_SCALING[0x05]`
and `[0x09]` are now `count=5`, and `data[4]` carries heading accuracy in radians. GRV (0x08)
remains a 4-tuple (no heading accuracy field in the protocol).

**Multi-response init bug.** The BNO085 sends 4 Product ID Responses on every reset — one per
firmware module. Only the first carries `reset_cause=4` (External Reset); subsequent ones report
`cause=0`. The original check `if reset_cause != 4` was applied to all responses, causing every
init to raise `RuntimeError`. Fixed by guarding with `if not self._product_id_received`.

---

## Related

- ADR-001 — driver gap analysis and initial report inventory
- DR-004 — calibration and tare procedure (not affected by these requirements)