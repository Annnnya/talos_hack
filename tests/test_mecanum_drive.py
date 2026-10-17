from __future__ import annotations

import pytest
from scservo_sdk import COMM_RX_TIMEOUT

from talos_servo.mecanum_drive import DriveFault, Kinematics, MecanumDrive

IDS = (1, 2, 3, 4)
KINEMATICS = Kinematics(
    wheelbase_m=0.3,
    track_width_m=0.3,
    steps_per_meter=1000.0,
    max_speed=100,
    wheel_directions=(1, 1, 1, 1),
)


def make_drive(bus, **kwargs) -> MecanumDrive:
    return MecanumDrive(bus, IDS, KINEMATICS, 20, **kwargs)


def packet_speeds(packet: dict[int, list[int]]) -> dict[int, int]:
    def decode(data: list[int]) -> int:
        raw = data[5] | (data[6] << 8)
        return -(raw & 0x7FFF) if raw & 0x8000 else raw

    return {servo_id: decode(data) for servo_id, data in packet.items()}


def test_tick_sends_all_wheels_in_one_packet_before_any_read(sdk):
    with sdk.bus() as bus:
        drive = make_drive(bus)
        drive.set_command(0.1, 0.0, 0.0, now=0.0)
        drive.request_reads(1)
        drive.tick(now=0.0)
        kinds = [c[0] for c in sdk.handler.calls]
        assert kinds == ["sync_write", "read_block"]
        assert packet_speeds(sdk.handler.sync_writes()[0]) == {1: 100, 2: 100, 3: 100, 4: 100}
        assert sdk.handler.speed_calls() == []


def test_command_is_only_sent_on_tick(sdk):
    with sdk.bus() as bus:
        drive = make_drive(bus)
        drive.set_command(0.1, 0.0, 0.0, now=0.0)
        assert sdk.handler.sync_writes() == []


def test_mixed_motion_uses_proportional_accelerations(sdk):
    with sdk.bus() as bus:
        drive = make_drive(bus)
        # Strafe plus turn gives different wheel speeds.
        drive.set_command(0.1, 0.05, 0.0, now=0.0)
        drive.tick(now=0.0)
        packet = sdk.handler.sync_writes()[-1]
        speeds = packet_speeds(packet)
        accs = {servo_id: data[0] for servo_id, data in packet.items()}
        fastest = max(speeds, key=lambda i: abs(speeds[i]))
        slowest = min(speeds, key=lambda i: abs(speeds[i]))
        assert accs[fastest] == 20
        assert accs[slowest] < 20


def test_unchanged_target_keeps_ramp_rates(sdk):
    with sdk.bus() as bus:
        drive = make_drive(bus)
        drive.set_command(0.1, 0.05, 0.0, now=0.0)
        drive.tick(now=0.0)
        drive.tick(now=0.02)
        first, second = sdk.handler.sync_writes()[-2:]
        assert first == second


def test_watchdog_stops_wheels_after_timeout(sdk):
    with sdk.bus() as bus:
        drive = make_drive(bus, command_timeout_s=0.5)
        drive.set_command(0.1, 0.0, 0.0, now=0.0)
        assert not drive.tick(now=0.4).timed_out
        result = drive.tick(now=0.6)
        assert result.timed_out
        assert packet_speeds(sdk.handler.sync_writes()[-1]) == dict.fromkeys(IDS, 0)
        assert not drive.moving
        assert not drive.tick(now=0.7).timed_out


def test_reads_go_round_robin_one_per_tick(sdk):
    with sdk.bus() as bus:
        drive = make_drive(bus)
        drive.request_reads(5)
        indexes = [drive.tick(now=0.0).read_index for _ in range(6)]
        assert indexes == [0, 1, 2, 3, 0, None]


def test_single_read_failure_is_reported_not_fatal(sdk):
    sdk.fail[("read_block", 2)] = (COMM_RX_TIMEOUT, 0)
    with sdk.bus() as bus:
        drive = make_drive(bus, max_read_failures=3)
        drive.request_reads(2)
        drive.tick(now=0.0)
        result = drive.tick(now=0.0)
        assert result.read_index == 1
        assert result.telemetry is None
        assert "read telemetry of servo 2" in str(result.read_error)


def test_repeated_read_failures_raise_drive_fault(sdk):
    sdk.fail[("read_block", 1)] = (COMM_RX_TIMEOUT, 0)
    with sdk.bus() as bus:
        drive = make_drive(bus, max_read_failures=2)
        drive.request_reads(8)
        with pytest.raises(DriveFault, match="servo 1 failed 2 reads"):
            for _ in range(8):
                drive.tick(now=0.0)


def test_successful_read_resets_failure_count(sdk):
    with sdk.bus() as bus:
        drive = make_drive(bus, max_read_failures=2)
        drive.request_reads(12)
        sdk.handler.fail[("read_block", 1)] = (COMM_RX_TIMEOUT, 0)
        for _ in range(4):
            drive.tick(now=0.0)
        del sdk.handler.fail[("read_block", 1)]
        for _ in range(4):
            drive.tick(now=0.0)
        sdk.handler.fail[("read_block", 1)] = (COMM_RX_TIMEOUT, 0)
        for _ in range(4):
            drive.tick(now=0.0)


def test_write_failure_raises_drive_fault(sdk):
    sdk.fail[("sync_write", 254)] = (COMM_RX_TIMEOUT, 0)
    with sdk.bus() as bus:
        drive = make_drive(bus)
        drive.set_command(0.1, 0.0, 0.0, now=0.0)
        with pytest.raises(DriveFault, match="wheel command failed"):
            drive.tick(now=0.0)


def test_stop_uses_one_packet_and_clears_command(sdk):
    with sdk.bus() as bus:
        drive = make_drive(bus)
        drive.set_command(0.1, 0.0, 0.0, now=0.0)
        drive.tick(now=0.0)
        assert drive.stop() == []
        assert packet_speeds(sdk.handler.sync_writes()[-1]) == dict.fromkeys(IDS, 0)
        drive.tick(now=0.01)
        assert packet_speeds(sdk.handler.sync_writes()[-1]) == dict.fromkeys(IDS, 0)


def test_stop_falls_back_to_per_servo_stops(sdk):
    with sdk.bus() as bus:
        drive = make_drive(bus)
        sdk.handler.fail[("sync_write", 254)] = (COMM_RX_TIMEOUT, 0)
        errors = drive.stop()
        assert errors[0].startswith("sync stop failed")
        assert [c[1] for c in sdk.handler.speed_calls()] == list(IDS)
