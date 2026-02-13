# QA Report: Timestamping Verification

**Driver file**: `driver/bno08x.py`
**Spec references**: BNO08x Datasheet v1.17 (section 1.3.5.3), SH-2 Reference Manual v1.2 (sections 6.5.1, 7.2)
**Date**: 2026-02-13

## Overview

Verified the driver's implementation of the BNO08x timestamping mechanism against the
specification. Timestamping allows the host to reconstruct accurate sensor sample times
despite communication latency between the sensor hub and the host.

## Spec Summary

The timestamp system has four components:

| Component | Report ID | Format | Description |
|---|---|---|---|
| Transport Reference | HINT pin | Host-side | Host captures `T_hint` when interrupt fires |
| Base Timestamp Reference | 0xFB | Signed 32-bit, 100us ticks | Delta from HINT to batch baseline |
| Per-report Delay | Status byte + Delay byte | Unsigned 14-bit, 100us ticks | Delay from sensor sample to timebase |
| Timestamp Rebase | 0xFA | Signed 32-bit, 100us ticks | Adjustment added to current base |

### Formula (from BNO08x datasheet 1.3.5.3, Figure 1-36)

```
base_point       = T_hint - base_delta
sensor_timestamp = base_point + report_delay
```

Spec example: Base Delta=120 (12ms), Report 1 Delay=0, Report 2 Delay=17 (1.7ms).
Result: Report 1 at T-12ms, Report 2 at T-10.3ms.

### Rebase (SH-2 7.2.2)

When reports span too far from the Base Reference for the 14-bit delay field:

```
new_base = old_base + rebase_delta
```

Spec example: HINT=5.0s, Base Delta=4.0s -> base=1.0s. Rebase Delta=1.5s -> new base=2.5s.
A report with delay=1.0s would be timestamped at t=3.5s.

## Findings

### 1. Core timestamp formula - PASS

The driver correctly implements `T_hint - base_delta + delay`.

**Fast path** (bno08x.py:894-895):
```python
packet_base_ms = ticks_diff(self.ms_at_interrupt, self._epoch_start_ms) - (
        self._last_base_timestamp_us * FP_TO_MS)
# then per report:
ts = packet_base_ms + delay_100us_ticks * FP_DIV_TEN
```

**Slow path** (bno08x.py:1303-1304):
```python
self._sensor_ms = ticks_diff(self.ms_at_interrupt,
                             self._epoch_start_ms) - self._last_base_timestamp_us * 0.001 + delay_ms
```

Both paths produce the same result. Matches the spec.

### 2. 14-bit delay extraction - PASS

Spec (SH-2 6.5.1 Figure 65): Status byte bits 7:2 = 6 MSBs of delay. Byte 3 = 8 LSBs.

**Fast path** (bno08x.py:912):
```python
(((b2 & 0xFC) << 6) | p[idx + 3]) * FP_DIV_TEN
```

**Slow path** (bno08x.py:1298):
```python
(((r.byte2 >> 2) << 8) | r.byte3) * 0.1
```

Both correctly reconstruct the 14-bit delay value. Verified with max value: 0x3FFF = 16383
ticks = 1638.3ms, matching the SH-2 spec statement of "up to 1.64 seconds".

### 3. Delay and base delta units - PASS

- Base delta: raw value * 100 = microseconds, then * 0.001 = milliseconds. Correct.
- Report delay: raw 14-bit value in 100us ticks, * 0.1 = milliseconds. Correct.

### 4. Timestamp Rebase (0xFA) semantics - FIXED

**Spec** (SH-2 7.2.2): "The Timestamp Rebase record is **added to** the batch's Base
Timestamp Reference to derive a new base."

**Bug**: The driver **replaced** `_last_base_timestamp_us` with the rebase value instead
of accumulating it. Per the spec, `new_base = old_base + rebase_delta`.

**Fix applied**: Changed `=` to `+=` at both rebase sites:
```python
self._last_base_timestamp_us += unpack_from("<i", report_bytes, 1)[0] * 100
```

Fixed in two locations:
- `_process_report()` at line 1316
- `_process_control_report()` at line 1382

### 5. Signed parsing missing for 0xFB and 0xFA - FIXED

**Spec** (SH-2 7.2.1, 7.2.2): Both Base Delta and Rebase Delta are explicitly
**"Signed. Units are 100 microsecond ticks."**

**Bug**: All 5 parsing sites used unsigned byte assembly. In MicroPython, `0xFF << 24`
produces 4278190080 (positive), not -16777216. A negative delta would be misinterpreted
as a large positive number.

**Fix applied**: Replaced manual byte assembly with `unpack_from("<i", ..., 1)[0] * 100`
(lowercase `i` = signed int32) at all 5 sites:
- Fast path base timestamp: bno08x.py:893
- `_process_report()` base timestamp: bno08x.py:1311
- `_process_report()` rebase: bno08x.py:1316
- `_process_control_report()` base timestamp: bno08x.py:1377
- `_process_control_report()` rebase: bno08x.py:1382

### 6. Channel 5 Gyro Integrated RV timestamping - PASS

Per SH-2 6.5.44.2 (Figure 109), the Gyro Integrated Rotation Vector on channel 5 has
**no report ID, status, or delay fields**. The driver correctly uses only the host
interrupt time (bno08x.py:983):

```python
ts = ticks_diff(self.ms_at_interrupt, self._epoch_start_ms)
```

This is the best available approach given no sensor-side timing metadata.

## Results Summary

| # | Check | Result | Severity |
|---|---|---|---|
| 1 | Core formula `T - base + delay` | PASS | - |
| 2 | 14-bit delay bit extraction | PASS | - |
| 3 | Units (100us ticks to ms) | PASS | - |
| 4 | Timestamp Rebase (0xFA) accumulation | **FIXED** | Medium |
| 5 | Signed parsing for 0xFB/0xFA | **FIXED** | Low |
| 6 | Channel 5 (no delay fields) | PASS | - |

**Overall**: 6/6 checks pass. Two bugs were found and fixed.

## Regression Test

**Test**: Game Rotation Vector static hold, 344 Hz, 2000 samples.
**Baseline**: `tests/report_rate/results/game_rotation_vector/iteration_2/static_hold/game_rot_vec_344hz_2000s.csv`

| Metric | Pre-fix (iter 2) | Post-fix (iter 3) | Verdict |
|---|---|---|---|
| Samples | 2000 | 2000 | identical |
| Duration | 6.97s | 7.10s | normal variance |
| Avg rate | 286.7 Hz | 281.5 Hz | within run-to-run noise |
| Median delta | 3.0 ms | 3.0 ms | identical |
| Timestamp gaps (>10ms) | 18 | 18 | identical count and pattern |
| Negative deltas | 0 | 0 | clean |
| Mean lag | 3.65 ms | 3.73 ms | +74us (negligible) |
| P95 lag | 4.9 ms | 4.9 ms | identical |
| Max lag | 42.9 ms | 42.6 ms | slightly improved |

**No regressions detected.** The 18 periodic ~40ms gaps in both runs occur at matching
row indices, confirming they are firmware-level pauses inherent to the BNO085, not
driver-related. Both bugs only trigger under conditions not present in normal bench
operation (host stalls >1.6s or negative base deltas).
