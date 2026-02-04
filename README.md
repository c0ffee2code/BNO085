# BNO085 Flight Control Test Bench

Flight control system test bench using the BNO085 9-axis IMU on Raspberry Pi Pico 2.

## Acknowledgments

This project is built upon the excellent MicroPython BNO08X driver by **BradCar**:

**https://github.com/bradcar/bno08x_i2c_spi_MicroPython**

Huge thanks to BradCar for making this driver available - it made this pet project possible!

The original driver was adapted from:
- Adafruit CircuitPython library by Bryan Siepert
- Additional contributions by dobodu

## Hardware

- **MCU**: Raspberry Pi Pico 2 (RP2350)
- **IMU**: [Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085)
- **Interface**: I2C with interrupt pin

### About the BNO085

The Adafruit BNO085 breakout features a Hillcrest Laboratories BNO085 chip with an integrated ARM Cortex M0 processor running proprietary SH-2 firmware for on-chip sensor fusion.

**Base Sensors:**
- 3-axis accelerometer (gravity + linear motion) in m/s²
- 3-axis gyroscope (rotation speed) in rad/s
- 3-axis magnetometer in µT

**Fusion Outputs:**
- Absolute orientation as quaternion
- Rotation vectors (standard, game, geomagnetic, AR/VR optimized)
- Linear acceleration (gravity removed)
- Gravity vector
- Euler angles (derived from quaternion)

**Additional Features:**
- Stability detection and classification
- Activity classification (walking, running, etc.)
- Step counter and detector
- Tap and shake detection
- Significant motion detection

**Compatibility Note:** The BNO085 I2C implementation has known quirks, but works well with RP2040/RP2350 (Pico 2).

## Project Goal

A learning project to understand flight control systems by building, breaking, and iterating. Practice, practice, practice!
