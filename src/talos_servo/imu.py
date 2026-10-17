"""Driver for the MPU6050 6-axis IMU (accelerometer + gyroscope) on an I2C bus."""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from types import TracebackType
from typing import Any

log = logging.getLogger(__name__)

DEFAULT_I2C_BUS = 1
DEFAULT_ADDRESS = 0x68
# With the AD0 pin tied high the sensor answers here instead.
ALT_ADDRESS = 0x69

# Register map (MPU-6000/6050 register map, rev 4.2).
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
PWR_MGMT_1 = 0x6B
WHO_AM_I = 0x75

# Accel X/Y/Z, temperature, gyro X/Y/Z: seven big-endian 16-bit words.
SAMPLE_BLOCK_LEN = 14

WHO_AM_I_MPU6050 = 0x68
# Values reported by compatible chips and common clones sold on MPU6050 breakout boards.
WHO_AM_I_COMPATIBLE = frozenset({0x70, 0x71, 0x72, 0x73, 0x98})

# PWR_MGMT_1: clear the sleep bit and clock from the X gyro PLL, which drifts less than
# the internal oscillator.
PWR_CLOCK_PLL_GYRO_X = 0x01
PWR_SLEEP = 0x40

# Digital low-pass filter setting 3 is about 44 Hz, which damps servo vibration.
DEFAULT_DLPF = 3
MAX_DLPF = 6
# With the DLPF on, the gyro runs at 1 kHz; divider 9 gives 100 Hz output.
DEFAULT_SAMPLE_RATE_DIV = 9

# Full-scale range -> (config register value, LSB per unit).
ACCEL_RANGES: dict[int, tuple[int, float]] = {
    2: (0x00, 16384.0),
    4: (0x08, 8192.0),
    8: (0x10, 4096.0),
    16: (0x18, 2048.0),
}
GYRO_RANGES: dict[int, tuple[int, float]] = {
    250: (0x00, 131.0),
    500: (0x08, 65.5),
    1000: (0x10, 32.8),
    2000: (0x18, 16.4),
}
DEFAULT_ACCEL_RANGE = 2
DEFAULT_GYRO_RANGE = 250

TEMP_LSB_PER_C = 340.0
TEMP_OFFSET_C = 36.53

# Time for the PLL and filters to settle after waking.
SETTLE_SECONDS = 0.1

DEFAULT_CALIBRATION_SAMPLES = 200
DEFAULT_CALIBRATION_INTERVAL = 0.005

Vector = tuple[float, float, float]


@dataclass(frozen=True)
class ImuSample:
    accel: Vector  # g
    gyro: Vector  # degrees per second
    temperature: float  # C


class ImuError(RuntimeError):
    """An IMU operation failed."""


def _default_bus_factory(bus_number: int) -> Any:
    try:
        from smbus2 import SMBus
    except ImportError as exc:
        raise ImuError("the imu needs the imu extra (smbus2 missing): uv sync --extra imu") from exc
    return SMBus(bus_number)


def to_signed16(high: int, low: int) -> int:
    value = (high << 8) | low
    return value - 0x10000 if value & 0x8000 else value


class Mpu6050:
    """One MPU6050 on an I2C bus.

    Use as a context manager. On exit the sensor is put back to sleep and the bus is closed,
    even if the body raised.
    """

    def __init__(
        self,
        bus_number: int = DEFAULT_I2C_BUS,
        address: int = DEFAULT_ADDRESS,
        *,
        accel_range: int = DEFAULT_ACCEL_RANGE,
        gyro_range: int = DEFAULT_GYRO_RANGE,
        dlpf: int = DEFAULT_DLPF,
        bus_factory: Callable[[int], Any] = _default_bus_factory,
    ) -> None:
        if accel_range not in ACCEL_RANGES:
            raise ValueError(
                f"accel range must be one of {sorted(ACCEL_RANGES)}, got {accel_range}"
            )
        if gyro_range not in GYRO_RANGES:
            raise ValueError(f"gyro range must be one of {sorted(GYRO_RANGES)}, got {gyro_range}")
        if not 0 <= dlpf <= MAX_DLPF:
            raise ValueError(f"dlpf must be within 0..{MAX_DLPF}, got {dlpf}")
        self.bus_number = bus_number
        self.address = address
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self.dlpf = dlpf
        self.gyro_offset: Vector = (0.0, 0.0, 0.0)
        self._bus_factory = bus_factory
        self._bus: Any = None

    def open(self) -> None:
        try:
            self._bus = self._bus_factory(self.bus_number)
        except OSError as exc:
            raise ImuError(
                f"could not open i2c bus {self.bus_number}: {exc} (is I2C enabled in raspi-config?)"
            ) from exc
        try:
            self._check_identity()
            self._write(PWR_MGMT_1, PWR_CLOCK_PLL_GYRO_X, "wake sensor")
            self._write(CONFIG, self.dlpf, "set low-pass filter")
            self._write(SMPLRT_DIV, DEFAULT_SAMPLE_RATE_DIV, "set sample rate")
            self._write(GYRO_CONFIG, GYRO_RANGES[self.gyro_range][0], "set gyro range")
            self._write(ACCEL_CONFIG, ACCEL_RANGES[self.accel_range][0], "set accel range")
        except BaseException:
            self._close_bus()
            raise
        time.sleep(SETTLE_SECONDS)

    def close(self) -> None:
        if self._bus is None:
            return
        try:
            self._write(PWR_MGMT_1, PWR_SLEEP, "put sensor to sleep")
        except ImuError as exc:
            log.warning("%s", exc)
        finally:
            self._close_bus()

    def __enter__(self) -> Mpu6050:
        self.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def read_raw(self) -> tuple[int, int, int, int, int, int, int]:
        """Return raw signed (ax, ay, az, temp, gx, gy, gz) from one burst read."""
        data = self._read_block(ACCEL_XOUT_H, SAMPLE_BLOCK_LEN, "read sample")
        if len(data) != SAMPLE_BLOCK_LEN:
            raise ImuError(
                f"read sample failed: got {len(data)} bytes, expected {SAMPLE_BLOCK_LEN}"
            )
        ax, ay, az, temp, gx, gy, gz = (
            to_signed16(data[i], data[i + 1]) for i in range(0, SAMPLE_BLOCK_LEN, 2)
        )
        return ax, ay, az, temp, gx, gy, gz

    def read(self) -> ImuSample:
        ax, ay, az, temp, gx, gy, gz = self.read_raw()
        accel_scale = ACCEL_RANGES[self.accel_range][1]
        gyro_scale = GYRO_RANGES[self.gyro_range][1]
        ox, oy, oz = self.gyro_offset
        return ImuSample(
            accel=(ax / accel_scale, ay / accel_scale, az / accel_scale),
            gyro=(gx / gyro_scale - ox, gy / gyro_scale - oy, gz / gyro_scale - oz),
            temperature=temp / TEMP_LSB_PER_C + TEMP_OFFSET_C,
        )

    def calibrate_gyro(
        self,
        samples: int = DEFAULT_CALIBRATION_SAMPLES,
        interval: float = DEFAULT_CALIBRATION_INTERVAL,
    ) -> Vector:
        """Average the gyro while the sensor is still and use it as the zero offset."""
        if samples < 1:
            raise ValueError(f"samples must be >= 1, got {samples}")
        gyro_scale = GYRO_RANGES[self.gyro_range][1]
        sums = [0.0, 0.0, 0.0]
        for _ in range(samples):
            _ax, _ay, _az, _temp, gx, gy, gz = self.read_raw()
            sums[0] += gx
            sums[1] += gy
            sums[2] += gz
            time.sleep(interval)
        self.gyro_offset = (
            sums[0] / samples / gyro_scale,
            sums[1] / samples / gyro_scale,
            sums[2] / samples / gyro_scale,
        )
        return self.gyro_offset

    def _check_identity(self) -> None:
        who = self._read_byte(WHO_AM_I, "read WHO_AM_I")
        if who == WHO_AM_I_MPU6050:
            return
        if who in WHO_AM_I_COMPATIBLE:
            log.warning("WHO_AM_I is 0x%02x, not 0x68: treating it as an MPU6050 compatible", who)
            return
        raise ImuError(f"device at 0x{self.address:02x} is not an MPU6050 (WHO_AM_I = 0x{who:02x})")

    def _require_open(self) -> Any:
        if self._bus is None:
            raise ImuError("imu is not open")
        return self._bus

    def _read_byte(self, register: int, operation: str) -> int:
        bus = self._require_open()
        try:
            return bus.read_byte_data(self.address, register)
        except OSError as exc:
            raise ImuError(f"{operation} failed at 0x{self.address:02x}: {exc}") from exc

    def _read_block(self, register: int, length: int, operation: str) -> list[int]:
        bus = self._require_open()
        try:
            return bus.read_i2c_block_data(self.address, register, length)
        except OSError as exc:
            raise ImuError(f"{operation} failed at 0x{self.address:02x}: {exc}") from exc

    def _write(self, register: int, value: int, operation: str) -> None:
        bus = self._require_open()
        try:
            bus.write_byte_data(self.address, register, value)
        except OSError as exc:
            raise ImuError(f"{operation} failed at 0x{self.address:02x}: {exc}") from exc

    def _close_bus(self) -> None:
        bus, self._bus = self._bus, None
        if bus is not None:
            bus.close()
