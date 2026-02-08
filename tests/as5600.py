"""
AS5600 Magnetic Rotary Encoder Driver for MicroPython
https://github.com/c0ffee2code/AS5600/blob/master/driver/as5600.py

A minimalistic driver for the AS5600 12-bit magnetic rotary position sensor,
designed for low-latency angle reading in flight control applications (testing benches with 1, 2 and more degrees of freedom).

Library Constraints:
    - Assumes 3.3V operation (Adafruit boards)
    - AGC range: 0-128 (optimal ~64 for best performance)
    - I2C address: 0x36 (fixed)

See README.md for AGC interpretation and troubleshooting guidance.
"""

from machine import I2C, Pin
from micropython import const
import time

# Encoder raw output is in steps
# Range is [0 ... 4096)
STEPS = const(4096)
DEG_PER_STEP = 360.0 / 4096.0   # ≈ 0.087890625°

DEFAULT_ADDR = const(0x36)
RAW_ANGLE_REG = const(0x0C)

# Configuration register
CONF_REG = const(0x07)

# Sensor status registers
STATUS_REG = const(0x0B)
AGC_REG = const(0x1A)
MAGNITUDE_REG = const(0x1B)

# CONF register bit masks (no const() - these are exported for external use)
CONF_PM_MASK = 0x0003      # bits 1:0 - Power Mode
CONF_HYST_MASK = 0x000C    # bits 3:2 - Hysteresis
CONF_SF_MASK = 0x0300      # bits 9:8 - Slow Filter
CONF_FTH_MASK = 0x1C00     # bits 12:10 - Fast Filter Threshold
CONF_WD_MASK = 0x2000      # bit 13 - Watchdog

# Recommended configuration for low-latency flight control
# SF=11 (0.286ms),
# FTH=001 (6 LSB),
# PM=00 (always on),
# HYST=00 (this feature does not affect ANGLE_RAW anyway),
# WD=0 (no need to put sensor asleep)
CONF_LOW_LATENCY = 0x0700

# AGC range for 3.3V operation (library constraint)
# For 5V operation this would be 255
AGC_MAX_3V = 128
AGC_MAX_5V = 255

def to_degrees(raw_angle, axis_center):
    steps_error = wrap_error(raw_angle - axis_center)
    return steps_error * DEG_PER_STEP

def wrap_error(err):
    """
    Normalize an angular error from a modulo encoder into the shortest signed distance.

    The AS5600 encoder reports angles modulo 4096 steps (0 and 4095 are adjacent).
    A naive subtraction of two angles can therefore produce large jumps near the
    wrap-around boundary (e.g. 4090 → 5 gives an error of -4085).

    This helper folds such values into the range [-2048, +2047], ensuring:
      - continuity across the 0/360° boundary
      - correct sign (direction of rotation)
      - a small, linear error suitable for control loops (PID)

    Input:
        err : int
            Raw difference between two encoder readings (e.g. current - reference)

    Returns:
        int
            Wrapped error representing the shortest angular distance in encoder steps.
    """
    if err > STEPS // 2:
        err -= STEPS
    elif err < -STEPS // 2:
        err += STEPS
    return err

class AS5600:
    """
    Minimalistic driver for AS5600 magnetic rotary encoder.

    Provides low-level I2C communication with the AS5600 sensor for:
    - Raw angle reading (12-bit, 0-4095 steps)
    - Sensor health monitoring (magnet detection, field strength)
    - Diagnostic telemetry for debugging and analysis
    """

    def __init__(self, i2c, address=DEFAULT_ADDR):
        """
        Initialize AS5600 driver.

        Args:
            i2c: MicroPython I2C instance (e.g., I2C(0, scl=Pin(1), sda=Pin(0), freq=400_000))
            address: I2C address (default 0x36)
        """
        self._i2c = i2c
        self._i2c_addr = address

    def read_raw_angle(self):
        """
        Read RAW_ANGLE register.

        Returns:
            int: Magnet position in steps, range [0, 4095] representing 0-360°
        """
        return self._read_12bit_register(RAW_ANGLE_REG)

    def diagnose(self, axis_center=None):
        """
        Generate single-line diagnostic telemetry string.

        This method provides a grep-friendly, pipe-delimited output for:
        - Hardware debugging: verify magnet positioning and field strength
        - Telemetry logging: capture sensor health over time for analysis
        - Correlation analysis: relate angle reading quality to AGC/magnitude
        - Performance tuning: monitor sensor state while adjusting filter settings

        The AGC (Automatic Gain Control) percentage indicates magnetic field strength.
        See README.md for interpretation guidance and recommended actions.

        The magnitude value from CORDIC provides additional field strength data.
        Consistent magnitude readings indicate stable magnet positioning.

        Args:
            axis_center: Optional mechanical center point in steps (0-4095).
                         If provided, includes DEG field with relative angle.

        Returns:
            str: Pipe-delimited diagnostic string with format:
                 AS5600|TS=<ms>|MAGNET=<bool>|WEAK=<bool>|STRONG=<bool>|
                 AGC=<val>|AGC_PCT=<pct>|MAG=<val>|RAW=<steps>|DEG=<degrees>

                 Where:
                 - TS: timestamp in milliseconds since boot (time.ticks_ms)
                 - MAGNET: magnet detected flag
                 - WEAK: magnet too weak flag (ML status bit)
                 - STRONG: magnet too strong flag (MH status bit)
                 - AGC: raw AGC value (0-128 for 3.3V operation)
                 - AGC_PCT: AGC as percentage (0-100)
                 - MAG: CORDIC magnitude value
                 - RAW: raw angle in steps (0-4095)
                 - DEG: relative angle in degrees (only if axis_center provided)

        Example:
            >>> encoder = AS5600(i2c)
            >>> print(encoder.diagnose(axis_center=422))
            AS5600|TS=123456|MAGNET=true|WEAK=false|STRONG=false|AGC=85|AGC_PCT=66|MAG=1847|RAW=422|DEG=0.0
        """
        ts = time.ticks_ms()

        # Read status register
        reg_status = self._read_8bit_register(STATUS_REG)
        magnet_detected = bool(reg_status & (1 << 5))
        magnet_too_weak = bool(reg_status & (1 << 4))
        magnet_too_strong = bool(reg_status & (1 << 3))

        # Read AGC and magnitude
        agc = self._read_8bit_register(AGC_REG)
        agc_pct = (agc * 100) // AGC_MAX_3V
        magnitude = self._read_12bit_register(MAGNITUDE_REG)

        # Read angle
        raw_angle = self.read_raw_angle()

        # Build diagnostic string
        parts = [
            "AS5600",
            f"TS={ts}",
            f"MAGNET={'true' if magnet_detected else 'false'}",
            f"WEAK={'true' if magnet_too_weak else 'false'}",
            f"STRONG={'true' if magnet_too_strong else 'false'}",
            f"AGC={agc}",
            f"AGC_PCT={agc_pct}",
            f"MAG={magnitude}",
            f"RAW={raw_angle}",
        ]

        # Add degrees if axis_center provided
        if axis_center is not None:
            deg = to_degrees(raw_angle, axis_center)
            parts.append(f"DEG={deg:.1f}")

        return "|".join(parts)

    def _read_12bit_register(self, register):
        """Read a 12-bit value from two consecutive registers."""
        hi, lo = self._i2c.readfrom_mem(self._i2c_addr, register, 2)
        return ((hi & 0x0F) << 8) | lo

    def _read_8bit_register(self, register):
        """Read an 8-bit value from a single register."""
        return self._i2c.readfrom_mem(self._i2c_addr, register, 1)[0]

    def _write_16bit_register(self, register, value):
        """Write a 16-bit value to two consecutive registers (big-endian)."""
        hi = (value >> 8) & 0xFF
        lo = value & 0xFF
        self._i2c.writeto_mem(self._i2c_addr, register, bytes([hi, lo]))

    def read_conf(self):
        """
        Read CONF register (0x07-0x08).

        Returns:
            int: 16-bit configuration value
        """
        hi, lo = self._i2c.readfrom_mem(self._i2c_addr, CONF_REG, 2)
        return (hi << 8) | lo

    def write_conf(self, value):
        """
        Write CONF register (0x07-0x08).

        Args:
            value: 16-bit configuration value

        Note:
            Changes to CONF register take effect immediately but are not
            persistent across power cycles (volatile memory).
        """
        self._write_16bit_register(CONF_REG, value)

    def configure_low_latency_mode(self):
        """
        Configure sensor for low-latency flight control applications.

        Sets:
        - SF=11: Fastest slow filter (0.286ms settling, 0.043° RMS noise)
        - FTH=001: Fast filter threshold at 6 LSB for quick step response
        - PM=00: Normal power mode (always on, no polling delay)
        - HYST=00: No hysteresis
        - WD=0: Watchdog off

        Expected performance:
        - Total latency: ~486 μs (150 μs ADC + 286 μs filter + 50 μs I2C)
        - Noise: 0.043° RMS (±0.129° 3σ)

        See decision/LATENCY_PRECISION_TRADEOFF.md for rationale.
        """
        self.write_conf(CONF_LOW_LATENCY)