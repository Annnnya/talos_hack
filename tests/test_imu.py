from __future__ import annotations

import logging
from functools import partial

import pytest

from talos_servo import cli
from talos_servo.imu import (
    ACCEL_CONFIG,
    ACCEL_RANGES,
    ACCEL_XOUT_H,
    CONFIG,
    DEFAULT_DLPF,
    GYRO_CONFIG,
    GYRO_RANGES,
    PWR_CLOCK_PLL_GYRO_X,
    PWR_MGMT_1,
    PWR_SLEEP,
    WHO_AM_I,
    ImuError,
    Mpu6050,
    to_signed16,
)


def make(i2c, **kwargs) -> Mpu6050:
    return Mpu6050(bus_factory=i2c, **kwargs)


def test_to_signed16():
    assert to_signed16(0x40, 0x00) == 16384
    assert to_signed16(0xFF, 0xFF) == -1
    assert to_signed16(0x80, 0x00) == -32768


def test_open_wakes_and_configures(i2c):
    with make(i2c, accel_range=8, gyro_range=1000) as imu:
        assert imu is not None
        writes = list(i2c.bus.writes)
    assert writes[0] == (PWR_MGMT_1, PWR_CLOCK_PLL_GYRO_X)
    assert (CONFIG, DEFAULT_DLPF) in writes
    assert (ACCEL_CONFIG, ACCEL_RANGES[8][0]) in writes
    assert (GYRO_CONFIG, GYRO_RANGES[1000][0]) in writes


def test_close_puts_sensor_to_sleep_and_closes_bus(i2c):
    with make(i2c):
        pass
    assert i2c.bus.writes[-1] == (PWR_MGMT_1, PWR_SLEEP)
    assert i2c.bus.closed


def test_bus_closed_when_body_raises(i2c):
    with pytest.raises(RuntimeError, match="boom"), make(i2c):
        raise RuntimeError("boom")
    assert i2c.bus.closed


def test_wrong_device_raises_and_closes_bus(i2c):
    i2c.registers = {WHO_AM_I: 0x12}
    with pytest.raises(ImuError, match="not an MPU6050"), make(i2c):
        pass
    assert i2c.bus.closed


def test_compatible_clone_warns_but_works(i2c, caplog):
    i2c.registers = {WHO_AM_I: 0x70}
    with caplog.at_level(logging.WARNING), make(i2c) as imu:
        imu.read()
    assert "0x70" in caplog.text


def test_wrong_address_raises(i2c):
    with (
        pytest.raises(ImuError, match="read WHO_AM_I failed at 0x69"),
        make(i2c, address=0x69),
    ):
        pass


def test_bus_open_failure_mentions_raspi_config(i2c):
    i2c.open_error = FileNotFoundError(
        2, "No such file or directory: '/dev/i2c-1'"
    )
    with pytest.raises(ImuError, match="raspi-config"), make(i2c):
        pass


def test_read_error_becomes_imu_error(i2c):
    with make(i2c) as imu:
        i2c.bus.fail["read_block"] = OSError(121, "Remote I/O error")
        with pytest.raises(ImuError, match="read sample failed"):
            imu.read()


def test_read_before_open_raises(i2c):
    with pytest.raises(ImuError, match="not open"):
        make(i2c).read()


@pytest.mark.parametrize("accel_range", sorted(ACCEL_RANGES))
def test_accel_scaling(i2c, accel_range):
    lsb = ACCEL_RANGES[accel_range][1]
    with make(i2c, accel_range=accel_range) as imu:
        i2c.bus.set_words(
            ACCEL_XOUT_H, [0, int(-lsb / 2), int(lsb), 0, 0, 0, 0]
        )
        sample = imu.read()
    assert sample.accel == pytest.approx((0.0, -0.5, 1.0))


@pytest.mark.parametrize("gyro_range", sorted(GYRO_RANGES))
def test_gyro_scaling(i2c, gyro_range):
    lsb = GYRO_RANGES[gyro_range][1]
    with make(i2c, gyro_range=gyro_range) as imu:
        i2c.bus.set_words(
            ACCEL_XOUT_H, [0, 0, 0, 0, round(lsb * 10), round(-lsb * 2), 0]
        )
        sample = imu.read()
    assert sample.gyro == pytest.approx((10.0, -2.0, 0.0), abs=0.02)


def test_temperature(i2c):
    with make(i2c) as imu:
        i2c.bus.set_words(ACCEL_XOUT_H, [0, 0, 0, -340, 0, 0, 0])
        assert imu.read().temperature == pytest.approx(35.53)


def test_calibrate_gyro_removes_offset(i2c):
    with make(i2c) as imu:
        i2c.bus.set_words(ACCEL_XOUT_H, [0, 0, 16384, 0, 131, -262, 0])
        assert imu.calibrate_gyro(samples=10) == pytest.approx((1.0, -2.0, 0.0))
        assert imu.read().gyro == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.parametrize(
    "kwargs",
    [{"accel_range": 3}, {"gyro_range": 300}, {"dlpf": 7}, {"dlpf": -1}],
)
def test_bad_settings_raise(i2c, kwargs):
    with pytest.raises(ValueError):
        make(i2c, **kwargs)


def test_bad_calibration_samples_raise(i2c):
    with make(i2c) as imu, pytest.raises(ValueError):
        imu.calibrate_gyro(samples=0)


# --- cli
# ---------------------------------------------------------------------------


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


@pytest.fixture
def run_imu(i2c, monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(cli, "Mpu6050", partial(Mpu6050, bus_factory=i2c))
    monkeypatch.setattr(cli.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(cli.time, "sleep", clock.sleep)
    monkeypatch.delenv(cli.PORT_ENV_VAR, raising=False)
    monkeypatch.delenv(cli.IMU_BUS_ENV_VAR, raising=False)
    return cli.main


def test_cli_imu_streams_without_servo_port(run_imu, i2c, capsys):
    assert run_imu(["imu", "--seconds", "0.3", "--interval", "0.1"]) == 0
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 3
    assert lines[0].startswith("acc[g] +0.00 +0.00 +0.00")
    assert i2c.bus.bus_number == 1
    assert i2c.bus.closed


def test_cli_imu_options(run_imu, i2c, capsys):
    args = [
        "imu",
        "--i2c-bus",
        "3",
        "--address",
        "0x68",
        "--seconds",
        "0.1",
        "--calibrate",
    ]
    assert run_imu(args) == 0
    assert i2c.bus.bus_number == 3
    assert "gyro offset" in capsys.readouterr().out


def test_cli_imu_bus_from_env(run_imu, i2c, monkeypatch):
    monkeypatch.setenv(cli.IMU_BUS_ENV_VAR, "5")
    assert run_imu(["imu", "--seconds", "0"]) == 0
    assert i2c.bus.bus_number == 5


def test_cli_imu_error_exits_1(run_imu, i2c, capsys):
    assert run_imu(["imu", "--address", "0x69", "--seconds", "0"]) == 1
    assert "WHO_AM_I failed at 0x69" in capsys.readouterr().err
