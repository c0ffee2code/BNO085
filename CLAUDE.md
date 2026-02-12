# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Flight control system test bench using BNO08X IMU sensor on Raspberry Pi Pico 2.

**Goal**: Build and test a flight control system with the Pico 2 as the brain.

**Target Platform**: MicroPython on Raspberry Pi Pico 2

**Base Driver**: Adapted from https://github.com/bradcar/bno08x_i2c_spi_MicroPython by BradCar

## Project Focus

This project performs **gap analysis between the BNO08X specification and the existing driver implementation**, adding/tuning features specifically needed for flight control applications:
- High-frequency attitude estimation (quaternions, rotation vectors)
- Sensor fusion for stable orientation
- Low-latency interrupt-driven updates
- Calibration management

## Architecture

### Core Files

- `driver/bno08x.py` - Main BNO08X driver with SHTP protocol implementation, sensor feature managers, and data parsing
- `driver/i2c.py` - I2C-specific subclass (`BNO08X_I2C`) with interrupt-driven packet reading
- `driver/spi.py` - SPI-specific subclass (`BNO08X_SPI`) with interrupt and wake pin support
- `specification/IMU BNO08x v1.17.pdf` - Official sensor datasheet

### Key Classes

- `BNO08X` - Base class handling SHTP protocol, report parsing, and sensor configuration
- `BNO08X_I2C` - I2C interface implementation (address 0x4B default, 0x4A alternate)
- `SensorFeature1/2/3/4` - Feature managers for different sensor data tuple sizes
- `RawSensorFeature` - Manager for raw accelerometer/gyroscope/magnetometer data

### Communication Protocol

Uses SHTP (Sensor Hub Transport Protocol) with 6 channels:
- Channel 0: Command/Advertisement
- Channel 1: Executable (reset)
- Channel 2: Control reports
- Channel 3: Sensor input data
- Channel 4: Wake-up sensors
- Channel 5: High-speed gyro (not implemented)

### Data Flow

1. INT pin triggers interrupt on new data
2. `update_sensors()` reads SHTP packets and parses reports
3. Report values stored in `_report_values` dictionary with timestamps
4. Feature manager classes provide property access to sensor data

## MicroPython Dependencies

```python
from machine import Pin, I2C
from struct import pack_into, unpack_from, pack
from utime import ticks_ms, ticks_us, ticks_diff, sleep_ms
from micropython import const
import uctypes
```

## Hardware Requirements

- I2C bus with `readfrom`, `writeto`, `readfrom_mem`, `writeto_mem` methods
- INT pin (mandatory) for interrupt-driven data reading
- RESET pin (optional) for hardware reset capability

## Performance Notes

Sensor update rates vary by interface:
- I2C: Up to 344 Hz for quaternion data
- SPI/UART: Up to 400 Hz for quaternion data (not yet implemented in this repo)
- Acceleration data: Up to 500 Hz on all interfaces

## Code Style

- Uses `micropython.const()` for memory-efficient constants
- Q-point fixed-point scaling for efficient embedded math
- `@micropython.native` decorators on time-critical paths
- Extensive docstrings document sensor report formats and protocol details

## Reference Documents

- `specification/IMU BNO08x v1.17.pdf` - Official BNO08X datasheet for gap analysis
