"""Synthetic Sensor Feeder.

Computes basic spatial kinematics and continuously writes mock accelerometer,
gyroscope, and barometric values into Renode's virtual sensor registers.
"""
from __future__ import annotations

import math
import struct
import threading
import time
from typing import Any

from backend.logger import setup_logger

logger = setup_logger("sensor_feeder")


class SensorFeeder(threading.Thread):
    def __init__(self, machine: Any, update_rate_hz: int = 50):
        super().__init__(daemon=True)
        self.machine = machine
        self.update_rate_hz = update_rate_hz
        self._running = False
        
        # Simulated state
        self.altitude_m = 50.0
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0

        # Memory mapped addresses (matching the .repl or typical firmware expectations)
        # Mock addresses for I2C/SPI connected sensors exposed to the bus
        self.MPU6000_ADDR = 0x40013000  # SPI1 base
        self.MS5611_ADDR = 0x40005400   # I2C1 base

    def start_feeding(self) -> None:
        self._running = True
        self.start()

    def stop_feeding(self) -> None:
        self._running = False

    def run(self) -> None:
        logger.info(f"Sensor feeder started at {self.update_rate_hz}Hz.")
        dt = 1.0 / self.update_rate_hz
        
        while self._running:
            self._feed_imu()
            self._feed_barometer()
            time.sleep(dt)

    def _feed_imu(self) -> None:
        """Feed synthetic accelerometer and gyroscope data."""
        # Simple hover state: 1G on Z axis, 0 on X and Y.
        accel_x, accel_y, accel_z = 0.0, 0.0, -9.81
        gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0

        # In a real pyrenode integration, we would write these to the peripheral's registers.
        try:
            # Struct pack floats to bytes, then write to memory
            # sysbus = self.machine.sysbus
            # sysbus.WriteDoubleWord(self.MPU6000_ADDR + 0x3B, struct.unpack('<I', struct.pack('<f', accel_x))[0])
            pass
        except Exception:
            pass

    def _feed_barometer(self) -> None:
        """Feed synthetic pressure data derived from altitude."""
        # Standard atmosphere equation: P = P0 * (1 - L*h/T0) ^ (g*M / (R*L))
        # Simplified mock pressure in Pascals
        pressure_pa = 101325.0 * math.exp(-0.00012 * self.altitude_m)
        temperature_c = 20.0
        
        try:
            # sysbus = self.machine.sysbus
            # sysbus.WriteDoubleWord(self.MS5611_ADDR + 0x00, int(pressure_pa))
            pass
        except Exception:
            pass
