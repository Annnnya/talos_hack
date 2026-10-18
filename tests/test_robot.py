"""Robot against the fake SDK: config, limits, direction, watchdog, stopping."""

from __future__ import annotations

import pytest
from conftest import BASE, SMALL, FakeClock, config
from scservo_sdk import (
    COMM_RX_TIMEOUT,
    SMS_STS_MODE,
    SMS_STS_MOVING,
    SMS_STS_TORQUE_ENABLE,
)

from talos_servo.bus import (
    MAX_POSITION,
    MAX_SPEED,
    MODE_WHEEL,
    ServoError,
    _speed_packet,
)
from talos_servo.robot import (
    FAULT_CLEAR_S,
    RETRY_OFFLINE_EVERY,
    SOURCE_ROBOT,
    CommandError,
    Robot,
)


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def robot(sdk, clock) -> Robot:
    r = Robot(sdk.bus(), config(SMALL), clock=clock)
    r.open()
    return r


@pytest.fixture
def base(sdk, clock) -> Robot:
    r = Robot(sdk.bus(), config(BASE), clock=clock)
    r.open()
    return r


def last_speeds(sdk) -> dict[int, list[int]]:
    return sdk.handler.sync_writes()[-1]


def speed_of(packet: list[int]) -> int:
    raw = packet[5] | packet[6] << 8
    return -(raw & 0x7FFF) if raw & 0x8000 else raw


# --- open and close ----------------------------------------------------------


def test_open_sets_each_mode_once_from_config(sdk, robot):
    mode_writes = [
        c
        for c in sdk.handler.calls
        if c[0] == "write1" and c[2] == SMS_STS_MODE
    ]
    assert mode_writes == [("write1", 2, SMS_STS_MODE, MODE_WHEEL)]
    assert all(j.online for j in robot.joints.values())


def test_open_starts_from_stopped_wheels(sdk, robot):
    assert speed_of(last_speeds(sdk)[2]) == 0


def test_disabled_joints_are_never_touched(sdk, base):
    assert "bend" not in base.joints
    assert not [c for c in sdk.handler.calls if len(c) > 1 and c[1] == 5]
    assert [s["name"] for s in base.snapshot()] == ["fl", "fr", "rl", "rr"]


def test_close_stops_wheels_and_closes_port(sdk, robot):
    robot.spin("wheel", 300)
    robot.tick()
    robot.close()
    assert speed_of(last_speeds(sdk)[2]) == 0
    assert sdk.port.closed_count == 1


def test_missing_servo_is_offline_and_retried_later(sdk, clock):
    sdk.fail[("ping", 2)] = (COMM_RX_TIMEOUT, 0)
    robot = Robot(sdk.bus(), config(SMALL), clock=clock)
    robot.open()
    assert robot.joints["bend"].online
    assert not robot.joints["wheel"].online
    del sdk.fail[("ping", 2)]
    for _ in range(RETRY_OFFLINE_EVERY):
        robot.tick()
    assert robot.joints["wheel"].online


# --- commands ----------------------------------------------------------------


def test_move_is_clamped_to_joint_and_speed_limits(sdk, robot):
    robot.move("bend", 4000, 9999, 20)
    assert ("move", 1, 3000, 500, 20) in sdk.handler.calls
    assert robot.joints["bend"].target == 3000


def test_spin_is_sent_on_the_next_tick_and_clamped(sdk, robot):
    robot.spin("wheel", 9999)
    robot.tick()
    assert speed_of(last_speeds(sdk)[2]) == 500


def test_uncapped_spin_is_clamped_to_the_servo_range_only(sdk, robot):
    robot.spin("wheel", 9999, capped=False)
    robot.tick()
    assert speed_of(last_speeds(sdk)[2]) == 9999
    robot.spin("wheel", MAX_SPEED + 1, capped=False)
    assert robot.joints["wheel"].target_speed == MAX_SPEED


def test_control_mode_is_fixed(robot):
    with pytest.raises(CommandError, match="speed joint"):
        robot.move("wheel", 2000, 100)
    with pytest.raises(CommandError, match="position joint"):
        robot.spin("bend", 100)


def test_unknown_joint_is_an_error(robot):
    with pytest.raises(CommandError, match="unknown joint 'nope'"):
        robot.spin("nope", 100)


def test_stop_writes_right_away(sdk, robot):
    robot.spin("wheel", 300)
    robot.tick()
    robot.stop("wheel")
    assert sdk.handler.speed_calls()[-1] == ("speed", 2, 0, 20)
    assert robot.joints["wheel"].target_speed == 0


def test_stop_all_never_raises(sdk, robot):
    robot.spin("wheel", 300)
    sdk.fail[("sync_write", 254)] = (COMM_RX_TIMEOUT, 0)
    sdk.fail[("speed", 2)] = (COMM_RX_TIMEOUT, 0)
    robot.stop_all()
    assert robot.joints["wheel"].target_speed == 0
    assert "timeout" in robot.joints["wheel"].error


def test_stuck_wheel_write_error_is_logged_once(sdk, robot, caplog):
    sdk.fail[("sync_write", 254)] = (COMM_RX_TIMEOUT, 0)
    with caplog.at_level("INFO", logger="talos_servo.robot"):
        for _ in range(5):
            robot.tick()
        del sdk.fail[("sync_write", 254)]
        robot.tick()
    messages = [r.getMessage() for r in caplog.records]
    assert sum("wheel speed write failed" in m for m in messages) == 1
    assert "wheel speed writes work again" in messages


def test_torque_off_stops_a_speed_joint_first(sdk, robot):
    robot.spin("wheel", 300)
    robot.set_torque("wheel", False)
    assert sdk.handler.calls[-2:] == [
        ("speed", 2, 0, 20),
        ("write1", 2, 40, 0),
    ]


# --- direction
# -----------------------------------------------------------------


def test_inverted_speed_joint_gets_the_opposite_sign(sdk, base):
    base.spin("rr", 100)
    base.spin("fl", 100)
    base.tick()
    sent = last_speeds(sdk)
    assert (speed_of(sent[1]), speed_of(sent[4])) == (100, -100)


def test_inverted_joint_reads_back_in_joint_units(sdk, base):
    sdk.handler.speed = -250
    for _ in base.joints:
        base.tick()
    assert base.joints["rr"].speed == 250
    assert base.joints["fl"].speed == -250


def test_inverted_position_joint_is_mirrored(sdk, clock):
    data = {
        "joints": {"bend": {"id": 1, "control": "position", "inverted": True}}
    }
    robot = Robot(sdk.bus(), config(data), clock=clock)
    robot.open()
    robot.move("bend", 1000, 100)
    assert sdk.handler.calls[-1][:3] == ("move", 1, MAX_POSITION - 1000)


# --- watchdog and drive
# --------------------------------------------------------


def test_expiring_spin_stops_without_new_commands(sdk, robot, clock):
    robot.spin("wheel", 300, expires=True)
    clock.now += 0.3
    robot.tick()
    assert robot.joints["wheel"].target_speed == 300
    clock.now += 0.3
    robot.tick()
    assert robot.joints["wheel"].target_speed == 0
    assert robot.joints["wheel"].command_source == SOURCE_ROBOT


def test_plain_spin_keeps_going(robot, clock):
    robot.spin("wheel", 300)
    clock.now += 10
    robot.tick()
    assert robot.joints["wheel"].target_speed == 300


def test_drive_forward_turns_every_wheel_forward(sdk, base):
    base.drive(0.1, 0.0, 0.0)
    base.tick()
    sent = last_speeds(sdk)
    assert [speed_of(sent[i]) for i in (1, 2, 3)] == [100, 100, 100]
    assert speed_of(sent[4]) == -100  # inverted motor, same wheel direction


def test_drive_is_clamped_to_base_limits(base):
    base.drive(5.0, 0.0, 0.0)
    assert base.joints["fl"].target_speed == 200  # 0.2 m/s * 1000 steps/m


def test_drive_ignores_non_finite_input(base):
    base.drive(float("nan"), 0.0, float("inf"))
    assert all(j.target_speed == 0 for j in base.joints.values())


def test_drive_stops_without_new_commands(base, clock):
    base.drive(0.1, 0.0, 0.0)
    clock.now += 1.0
    base.tick()
    assert all(j.target_speed == 0 for j in base.joints.values())


def test_drive_without_base_is_an_error(robot):
    with pytest.raises(CommandError, match="no base"):
        robot.drive(0.1, 0.0, 0.0)


def test_wheels_ramp_together(sdk, base):
    base.drive(0.0, 0.0, 0.0)
    base.drive(0.1, 0.05, 0.0)
    base.tick()
    sent = last_speeds(sdk)
    # FL and RR change less than FR and RL, so they ramp slower.
    assert sent[1][0] < sent[2][0]
    assert sent[2] == _speed_packet(150, 20)


def test_base_velocity_from_wheel_speeds(sdk, base):
    sdk.handler.speed = 100
    for _ in base.joints:
        base.tick()
    base.joints["rr"].speed = 100  # as if the inverted motor read -100
    vx, vy, wz = base.base_velocity()
    assert vx == pytest.approx(0.1)
    assert vy == pytest.approx(0.0)
    assert wz == pytest.approx(0.0)


# --- last command
# ----------------------------------------------------------------


def test_last_command_age_grows(robot, clock):
    robot.move("bend", 2100, 100, source="cli")
    clock.now += 12.34
    bend = next(s for s in robot.snapshot() if s["name"] == "bend")
    assert bend["last_command"] == {
        "source": "cli",
        "client": None,
        "age": 12.3,
    }


def test_tick_reads_every_wheel_in_one_packet(sdk, base):
    sdk.handler.calls.clear()
    base.tick()
    ids = sorted(base.joints[w].id for w in ("fl", "fr", "rl", "rr"))
    assert [sorted(r) for r in sdk.handler.sync_reads()] == [ids]


def test_tick_updates_wheel_position_and_speed(sdk, base):
    sdk.handler.position = 1000
    sdk.handler.speed = 300
    base.tick()
    # rr is inverted: its speed reads the other way round.
    assert base.joints["fl"].speed == 300
    assert base.joints["rr"].speed == -300
    assert base.joints["fl"].position == 1000


def test_failed_wheel_read_keeps_ticking(sdk, base, caplog):
    for servo_id in (1, 2, 3, 4):
        sdk.fail[("sync_read", servo_id)] = (COMM_RX_TIMEOUT, 0)
    base.tick()
    base.tick()
    assert caplog.text.count("wheel sync read failed") == 1
    # Speeds are still written.
    assert last_speeds(sdk)
    # One lost sync-read packet must not brake the robot: the round robin
    # owns online state and the safe state.
    assert base.fault is None
    base.spin("fl", 300)
    base.tick()
    assert speed_of(last_speeds(sdk)[1]) == 300


def test_lost_wheel_latches_through_the_round_robin(sdk, base):
    base.spin("fl", 300)
    sdk.fail[("sync_read", 1)] = (COMM_RX_TIMEOUT, 0)
    sdk.fail[("read_block", 1)] = (COMM_RX_TIMEOUT, 0)
    for _ in base.joints:
        base.tick()
    assert not base.joints["fl"].online
    assert "fl stopped answering" in base.fault
    assert all(j.target_speed == 0 for j in base.joints.values())


# --- safe state --------------------------------------------------------------


def torque_writes(sdk) -> list[tuple]:
    return [
        c
        for c in sdk.handler.calls
        if c[0] == "write1" and c[2] == SMS_STS_TORQUE_ENABLE
    ]


def move_calls(sdk) -> list[tuple]:
    return [c for c in sdk.handler.calls if c[0] == "move"]


def lose_bend(sdk, robot) -> None:
    """Make bend stop answering and tick until the round robin reads it."""
    sdk.fail[("read_block", 1)] = (COMM_RX_TIMEOUT, 0)
    for _ in robot.joints:
        robot.tick()
    assert not robot.joints["bend"].online


def test_wheel_write_error_brakes_and_latches(sdk, robot):
    robot.spin("wheel", 300)
    robot.tick()
    sdk.fail[("sync_write", 254)] = (COMM_RX_TIMEOUT, 0)
    robot.tick()
    assert "wheel write error" in robot.fault
    assert robot.joints["wheel"].target_speed == 0
    del sdk.fail[("sync_write", 254)]
    robot.tick()
    assert speed_of(last_speeds(sdk)[2]) == 0
    with pytest.raises(CommandError, match="safe state"):
        robot.spin("wheel", 300)
    assert torque_writes(sdk) == []


def test_stops_are_accepted_while_latched(sdk, base):
    base.hold_safe("test")
    base.drive(0.0, 0.0, 0.0)
    base.spin("fl", 0)
    base.stop("fl")
    base.stop_all()
    with pytest.raises(CommandError, match="safe state"):
        base.drive(0.1, 0.0, 0.0)


def test_lost_hinge_brakes_the_wheels_and_holds_its_goal(sdk, robot):
    robot.move("bend", 2500, 100)
    robot.spin("wheel", 300)
    robot.tick()
    moves = len(move_calls(sdk))
    lose_bend(sdk, robot)
    assert "bend stopped answering" in robot.fault
    assert robot.joints["wheel"].target_speed == 0
    assert speed_of(last_speeds(sdk)[2]) == 0
    with pytest.raises(CommandError, match="safe state"):
        robot.move("bend", 2600, 100)
    assert len(move_calls(sdk)) == moves
    assert robot.joints["bend"].target == 2500
    assert torque_writes(sdk) == []


def test_fault_holds_while_the_lost_joint_stays_offline(sdk, robot, clock):
    lose_bend(sdk, robot)
    for _ in range(3 * RETRY_OFFLINE_EVERY):
        clock.now += FAULT_CLEAR_S
        robot.tick()
    assert robot.fault is not None


def test_fault_clears_once_the_bus_is_healthy_for_a_while(sdk, robot, clock):
    lose_bend(sdk, robot)
    del sdk.fail[("read_block", 1)]
    for _ in range(RETRY_OFFLINE_EVERY):
        robot.tick()
    assert robot.joints["bend"].online
    assert robot.fault is not None
    clock.now += FAULT_CLEAR_S
    robot.tick()
    assert robot.fault is None
    robot.spin("wheel", 300)
    robot.tick()
    assert speed_of(last_speeds(sdk)[2]) == 300


def test_servo_missing_at_open_does_not_latch(sdk, clock):
    sdk.fail[("ping", 2)] = (COMM_RX_TIMEOUT, 0)
    robot = Robot(sdk.bus(), config(SMALL), clock=clock)
    robot.open()
    for _ in range(RETRY_OFFLINE_EVERY):
        robot.tick()
    assert robot.fault is None
    robot.move("bend", 2500, 100)


def test_failed_command_latches_the_safe_state(sdk, robot):
    robot.spin("wheel", 300)
    sdk.fail[("move", 1)] = (COMM_RX_TIMEOUT, 0)
    with pytest.raises(ServoError):
        robot.move("bend", 2500, 100)
    assert robot.fault.startswith("bend")
    assert robot.joints["wheel"].target_speed == 0


def test_hold_safe_never_raises(sdk, robot):
    robot.spin("wheel", 300)
    sdk.fail[("sync_write", 254)] = (COMM_RX_TIMEOUT, 0)
    sdk.fail[("speed", 2)] = (COMM_RX_TIMEOUT, 0)
    robot.hold_safe("imu says falling")
    assert robot.fault == "imu says falling"
    assert robot.joints["wheel"].target_speed == 0


def test_close_brakes_and_never_turns_torque_off(sdk, robot):
    robot.move("bend", 2500, 100)
    robot.spin("wheel", 300)
    robot.tick()
    moves = len(move_calls(sdk))
    robot.close()
    assert speed_of(last_speeds(sdk)[2]) == 0
    assert len(move_calls(sdk)) == moves
    assert torque_writes(sdk) == []


def test_open_starts_without_a_fault(sdk, robot):
    robot.close()
    robot.open()
    assert robot.fault is None


def test_wider_limits_let_a_position_joint_past_its_config(robot, sdk):
    robot.set_position_limits("bend", 0, MAX_POSITION)
    robot.move("bend", 100, 50)
    assert robot.joints["bend"].target == 100
    robot.reset_position_limits("bend")
    robot.move("bend", 100, 50)
    assert robot.joints["bend"].target == 1000


@pytest.mark.parametrize(("low", "high"), [(-1, 10), (10, 5), (0, 4096)])
def test_bad_limits_are_refused(robot, low, high):
    with pytest.raises(CommandError):
        robot.set_position_limits("bend", low, high)


def test_speed_joints_have_no_limits(robot):
    with pytest.raises(CommandError, match="speed joint"):
        robot.set_position_limits("wheel", 0, 10)


def test_read_position_reads_now(robot, sdk):
    sdk.handler.position = 1234
    sdk.handler.registers[SMS_STS_MOVING] = 1
    assert robot.read_position("bend") == (1234, True)


def test_read_position_fails_loudly(robot, sdk):
    sdk.handler.fail[("read_block", 1)] = (COMM_RX_TIMEOUT, 0)
    with pytest.raises(ServoError):
        robot.read_position("bend")
