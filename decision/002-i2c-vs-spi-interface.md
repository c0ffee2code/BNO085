# ADR-002: I2C vs SPI Interface for BNO085

**Date:** 2026-02-06
**Status:** Backlog (pending ADR-001 experiment completion)
**Context:** Flight control test bench on Raspberry Pi Pico 2

## Motivation

Baseline testing (ADR-001, Experiment 1) measured a consistent **~3.3ms transport delay** on I2C, producing ~1.2-2.1° position error during fast oscillations (~1.6Hz, ±39° amplitude). The error is almost entirely explained by I2C transport lag, not sensor inaccuracy.

This raises the question: would switching to SPI meaningfully reduce latency and improve flight control performance?

## What We Know So Far

### From the BNO085 Datasheet (v1.17)
- **I2C**: Up to 400kHz clock, max ~344Hz quaternion updates
- **SPI**: Up to 3MHz clock, max ~400Hz quaternion updates
- SPI is listed as "8x faster" than I2C in raw throughput

### From Our Baseline Measurements (I2C, Rotation Vector 0x05 at 100Hz)
- Transport lag: **3.1-4.5ms** (typical 3.3ms)
- Angle error at peak velocity: **1.2-2.1°**
- Angle error at turning points: **< 0.3°**
- The error pattern is pure phase lag (not noise or drift)

## Questions to Investigate

### Feasibility on Pico 2
- [ ] Does the Pico 2 hardware SPI support the BNO085's SPI mode? (CPOL/CPHA settings)
- [ ] Pin availability — do we have free SPI pins given the current wiring (I2C + AS5600 + INT)?
- [x] Does the existing base driver (BradCar) have SPI support? **YES** — full `BNO08X_SPI` class exists in [`lib/spi.py`](https://github.com/bradcar/bno08x_i2c_spi_MicroPython/blob/main/lib/spi.py)

### Performance
- [ ] How much of the 3.3ms lag is I2C bus transfer vs BNO085 internal processing?
  - If most lag is internal sensor processing, SPI won't help much
  - If most lag is bus transfer, SPI could cut it significantly
- [ ] What is the theoretical minimum transfer time for a rotation vector report (~23 bytes)?
  - I2C at 400kHz: ~0.5ms for 23 bytes (plus overhead)
  - SPI at 3MHz: ~0.06ms for 23 bytes
- [ ] Would SPI enable higher update rates (200Hz+) that are currently bus-limited?

### Practical Considerations
- [ ] Wiring complexity — SPI needs 4+2 wires (MOSI, MISO, SCK, CS, INT, WAKE) vs I2C's 2+1 (SDA, SCL, INT)
- [ ] Can we still use the AS5600 encoder on I2C while BNO085 is on SPI? (different buses, should be fine)
- [x] Driver implementation effort — **Low.** BradCar driver already has a complete `BNO08X_SPI` class with SHTP packet read/write, multipart assembly, and `@micropython.native` on hot paths. Requires: CS pin, INT pin, WAKE pin (must be high for SPI mode), and `spi.init(polarity=1, phase=1)`

## Possible Outcomes

1. **Stay on I2C** — if the lag is dominated by sensor-internal processing, SPI won't help enough to justify the effort. Predictive compensation (using gyro rate to extrapolate) might be simpler.

2. **Switch to SPI** — if bus transfer is a significant portion of the lag, SPI could reduce it substantially. Worth it if we need <2ms total latency for the control loop.

3. **Hybrid approach** — use SPI for the BNO085 (latency-critical) and keep I2C for the AS5600 encoder (not latency-critical). The Pico 2 has both buses available.

## Decision

**Deferred.** Complete ADR-001 experiments first. Revisit after we have:
- ARVR Stabilized RV (0x28) test results
- Gyro Integrated RV (0x2A) test results — these may reduce effective lag enough to make SPI unnecessary
- A clearer picture of what latency the flight control loop actually needs

## References

- ADR-001: Gap Analysis and Baseline Testing
- BNO085 Datasheet v1.17, Section 1.3 (Interface Comparison)
- Pico 2 Datasheet — SPI peripheral capabilities
- Base driver SPI implementation: https://github.com/bradcar/bno08x_i2c_spi_MicroPython/blob/main/lib/spi.py
