"""
Experiment 2: Game Rotation Vector (0x08) - Single Axis

Tests BNO085 game rotation vector (accel + gyro only, no magnetometer)
against AS5600 encoder reference on single axis.

Game RV avoids magnetometer-induced heading jumps but will drift in yaw over time.
Uses the shared report_rate_test module for hardware setup and sample collection.

Metrics (from ADR-001):
- Latency: ms delay between physical motion and reported angle
- Angle Error: degrees deviation from encoder reference
- Update Rate: actual Hz achieved
"""

from report_rate_test import setup_hardware, run

imu, encoder = setup_hardware()
run(imu, imu.game_quaternion, encoder, prefix="game_rot_vec")
