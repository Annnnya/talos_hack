"""IMU node against the fake i2c bus. Skipped where ROS messages are missing."""

from __future__ import annotations

import math

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("geometry_msgs")
pytest.importorskip("sensor_msgs")

from rclpy.parameter import Parameter  # noqa: E402

from talos_servo.imu import ACCEL_XOUT_H, ImuError, Mpu6050  # noqa: E402
from talos_servo.ros.imu_convert import (  # noqa: E402
    DEFAULT_ANGULAR_VELOCITY_VARIANCE,
    STANDARD_GRAVITY,
)
from talos_servo.ros.imu_node import (  # noqa: E402
    DEFAULT_FRAME_ID,
    DEFAULT_MAX_READ_FAILURES,
    ImuNode,
    parse_args,
)

# Raw counts at the default ranges: 16384 per g, 131 per deg/s.
ONE_G = 16384
ONE_DPS = 131
TEMP_RAW_25C = round((25.0 - 36.53) * 340)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def imu(i2c):
    sensor = Mpu6050(bus_factory=i2c)
    sensor.open()
    # Flat and still: +1 g on z, rotating +1 deg/s about x and -2 deg/s about z.
    i2c.bus.set_words(
        ACCEL_XOUT_H, [0, 0, ONE_G, TEMP_RAW_25C, ONE_DPS, 0, -2 * ONE_DPS]
    )
    yield sensor
    sensor.close()


def make_node(imu, **params) -> ImuNode:
    params.setdefault("calibrate", False)
    overrides = [Parameter(name, value=value) for name, value in params.items()]
    return ImuNode(imu, parameter_overrides=overrides)


@pytest.fixture
def node(imu):
    n = make_node(imu)
    yield n
    n.destroy_node()


def capture(monkeypatch, publisher) -> list:
    sent: list = []
    monkeypatch.setattr(publisher, "publish", sent.append)
    return sent


def test_tick_publishes_si_units(node, monkeypatch):
    sent = capture(monkeypatch, node.imu_pub)
    node.tick()
    (msg,) = sent
    assert msg.header.frame_id == DEFAULT_FRAME_ID
    assert msg.linear_acceleration.z == pytest.approx(STANDARD_GRAVITY)
    assert msg.linear_acceleration.x == 0.0
    assert msg.angular_velocity.x == pytest.approx(math.radians(1.0))
    assert msg.angular_velocity.z == pytest.approx(math.radians(-2.0))


def test_imu_msg_has_no_orientation_and_diagonal_covariances(node, monkeypatch):
    sent = capture(monkeypatch, node.imu_pub)
    node.tick()
    (msg,) = sent
    assert msg.orientation_covariance[0] == -1.0
    assert (
        msg.angular_velocity_covariance[0] == DEFAULT_ANGULAR_VELOCITY_VARIANCE
    )
    assert msg.angular_velocity_covariance[1] == 0.0
    assert (
        msg.angular_velocity_covariance[4] == DEFAULT_ANGULAR_VELOCITY_VARIANCE
    )


def test_topics_are_relative(node):
    topics = {name for name, _types in node.get_topic_names_and_types()}
    assert {"/imu/data_raw", "/imu/temperature"} <= topics


def test_frame_id_parameter(imu, monkeypatch):
    n = make_node(imu, frame_id="base_imu")
    try:
        sent = capture(monkeypatch, n.imu_pub)
        n.tick()
        assert sent[0].header.frame_id == "base_imu"
    finally:
        n.destroy_node()


def test_calibration_zeroes_the_gyro_at_startup(imu, monkeypatch):
    n = make_node(imu, calibrate=True)
    try:
        sent = capture(monkeypatch, n.imu_pub)
        n.tick()
        assert sent[0].angular_velocity.x == pytest.approx(0.0, abs=1e-9)
        assert sent[0].angular_velocity.z == pytest.approx(0.0, abs=1e-9)
    finally:
        n.destroy_node()


def test_temperature_waits_for_a_sample_then_publishes(node, monkeypatch):
    sent = capture(monkeypatch, node.temperature_pub)
    node.publish_temperature()
    assert sent == []
    node.tick()
    node.publish_temperature()
    assert sent[0].temperature == pytest.approx(25.0, abs=0.01)


def test_single_read_failure_is_skipped(node, imu, i2c, monkeypatch):
    sent = capture(monkeypatch, node.imu_pub)
    i2c.bus.fail["read_block"] = OSError(121, "Remote I/O error")
    node.tick()
    assert sent == []
    del i2c.bus.fail["read_block"]
    node.tick()
    assert len(sent) == 1
    assert node.consecutive_failures == 0


def test_repeated_read_failures_raise(node, i2c):
    i2c.bus.fail["read_block"] = OSError(121, "Remote I/O error")
    for _ in range(DEFAULT_MAX_READ_FAILURES - 1):
        node.tick()
    with pytest.raises(ImuError, match="failed in a row"):
        node.tick()


def test_bad_variance_parameter_is_rejected(imu):
    with pytest.raises(ValueError, match="variance"):
        make_node(imu, angular_velocity_variance=-1.0)


def test_parse_args_passes_ros_args_through(monkeypatch):
    monkeypatch.setenv("TALOS_IMU_BUS", "3")
    args, ros_args = parse_args(
        ["--address", "0x69", "--ros-args", "-p", "rate_hz:=50.0"]
    )
    assert args.i2c_bus == 3
    assert args.address == 0x69
    assert ros_args == ["--ros-args", "-p", "rate_hz:=50.0"]
