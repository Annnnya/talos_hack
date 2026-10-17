"""Servo bus wrappers: real Feetech STS hardware and a simulated mock.

Both classes expose the same interface so main.py doesn't care which one it uses.
The Feetech SDK is not thread-safe, so every bus access goes through one lock.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field, asdict

# STS register addresses (see scservo_sdk/sms_sts.py)
REG_MODE = 33
REG_TORQUE_ENABLE = 40
REG_PRESENT_LOAD = 60
REG_PRESENT_VOLTAGE = 62
REG_PRESENT_TEMPERATURE = 63

MODE_POSITION = "position"
MODE_WHEEL = "wheel"

POS_MAX = 4095      # 12-bit encoder, one full turn
SPEED_MAX = 3400    # steps/s, roughly the STS3215 limit
ACC_MAX = 254


@dataclass
class ServoState:
    id: int
    name: str
    min_pos: int = 0
    max_pos: int = POS_MAX
    online: bool = False
    mode: str = MODE_POSITION
    torque: bool = False
    position: int = 0
    speed: int = 0
    load: int = 0
    voltage: float = 0.0
    temp: int = 0
    target: int | None = None       # target position (position mode)
    target_speed: int = 0           # commanded speed (wheel mode)
    error: str | None = None

    def to_dict(self) -> dict:
        return asdict(self)


def _clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(v)))


class BaseBus:
    def __init__(self, servos: list[dict]):
        self.lock = threading.Lock()
        self.servos: dict[int, ServoState] = {
            s["id"]: ServoState(
                id=s["id"],
                name=s.get("name", f"Servo {s['id']}"),
                min_pos=s.get("min_pos", 0),
                max_pos=s.get("max_pos", POS_MAX),
            )
            for s in servos
        }
        self._tick = 0

    def snapshot(self) -> list[dict]:
        with self.lock:
            return [s.to_dict() for s in self.servos.values()]

    def _get(self, servo_id: int) -> ServoState:
        if servo_id not in self.servos:
            raise KeyError(f"Unknown servo id {servo_id}")
        return self.servos[servo_id]

    # Overridden below
    def open(self): ...
    def close(self): ...
    def poll(self): ...
    def set_mode(self, servo_id: int, mode: str): ...
    def set_torque(self, servo_id: int, enabled: bool): ...
    def move(self, servo_id: int, position: int, speed: int, acc: int): ...
    def spin(self, servo_id: int, speed: int, acc: int): ...

    def stop(self, servo_id: int, acc: int = 50):
        s = self._get(servo_id)
        if s.mode == MODE_WHEEL:
            self.spin(servo_id, 0, acc)

    def stop_all(self, acc: int = 50):
        for sid in list(self.servos):
            try:
                self.stop(sid, acc)
            except Exception as exc:  # keep stopping the others
                print(f"Warning: could not stop servo {sid}: {exc}")


class ServoBus(BaseBus):
    """Real hardware on a Feetech serial bus."""

    def __init__(self, port: str, baudrate: int, servos: list[dict]):
        super().__init__(servos)
        from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS  # imported lazily so --mock works without the SDK

        self._COMM_SUCCESS = COMM_SUCCESS
        self.port_name = port
        self.baudrate = baudrate
        self.port = PortHandler(port)
        self.sdk = sms_sts(self.port)

    # --- helpers -------------------------------------------------------
    def _ok(self, s: ServoState, comm: int, err: int, op: str) -> bool:
        if comm != self._COMM_SUCCESS:
            s.online = False
            s.error = f"{op}: {self.sdk.getTxRxResult(comm)}"
            return False
        s.online = True
        s.error = f"{op}: {self.sdk.getRxPacketError(err)}" if err else None
        return True

    def _write(self, s: ServoState, result, op: str) -> bool:
        comm, err = result
        return self._ok(s, comm, err, op)

    # --- lifecycle ------------------------------------------------------
    def open(self):
        if not self.port.openPort():
            raise RuntimeError(f"Could not open {self.port_name}")
        if not self.port.setBaudRate(self.baudrate):
            self.port.closePort()
            raise RuntimeError(f"Could not set baud rate {self.baudrate}")
        with self.lock:
            for s in self.servos.values():
                _, comm, err = self.sdk.ping(s.id)
                if not self._ok(s, comm, err, "ping"):
                    continue
                mode, comm, err = self.sdk.read1ByteTxRx(s.id, REG_MODE)
                if self._ok(s, comm, err, "read mode"):
                    s.mode = MODE_WHEEL if mode == 1 else MODE_POSITION
                torque, comm, err = self.sdk.read1ByteTxRx(s.id, REG_TORQUE_ENABLE)
                if self._ok(s, comm, err, "read torque"):
                    s.torque = bool(torque)
                print(f"Servo {s.id} ({s.name}): online, mode={s.mode}, torque={s.torque}")

    def close(self):
        self.stop_all()
        self.port.closePort()

    # --- telemetry ------------------------------------------------------
    def poll(self):
        """Read position/speed every tick; load/voltage/temp every 10th tick."""
        self._tick += 1
        slow = self._tick % 10 == 0
        with self.lock:
            for s in self.servos.values():
                # Offline servos are retried only on slow ticks so they don't stall the loop.
                if not s.online and not slow:
                    continue
                pos, spd, comm, err = self.sdk.ReadPosSpeed(s.id)
                if not self._ok(s, comm, err, "read pos/speed"):
                    continue
                s.position, s.speed = pos, spd
                if slow:
                    load, comm, err = self.sdk.read2ByteTxRx(s.id, REG_PRESENT_LOAD)
                    if self._ok(s, comm, err, "read load"):
                        s.load = self.sdk.scs_tohost(load, 10)
                    volt, comm, err = self.sdk.read1ByteTxRx(s.id, REG_PRESENT_VOLTAGE)
                    if self._ok(s, comm, err, "read voltage"):
                        s.voltage = volt / 10.0
                    temp, comm, err = self.sdk.read1ByteTxRx(s.id, REG_PRESENT_TEMPERATURE)
                    if self._ok(s, comm, err, "read temp"):
                        s.temp = temp

    # --- commands -------------------------------------------------------
    def set_mode(self, servo_id: int, mode: str):
        with self.lock:
            s = self._get(servo_id)
            if s.mode == MODE_WHEEL:
                self.sdk.WriteSpec(s.id, 0, 50)  # stop before switching
            value = 1 if mode == MODE_WHEEL else 0
            if self._write(s, self.sdk.write1ByteTxRx(s.id, REG_MODE, value), "set mode"):
                s.mode = mode
                s.target_speed = 0
                s.target = s.position if mode == MODE_POSITION else None

    def set_torque(self, servo_id: int, enabled: bool):
        with self.lock:
            s = self._get(servo_id)
            if self._write(s, self.sdk.write1ByteTxRx(s.id, REG_TORQUE_ENABLE, int(enabled)), "torque"):
                s.torque = enabled

    def move(self, servo_id: int, position: int, speed: int, acc: int):
        with self.lock:
            s = self._get(servo_id)
            if s.mode != MODE_POSITION:
                raise ValueError(f"Servo {servo_id} is in wheel mode; switch to position mode first")
            pos = _clamp(position, s.min_pos, s.max_pos)
            if self._write(s, self.sdk.WritePosEx(s.id, pos, _clamp(speed, 0, SPEED_MAX), _clamp(acc, 0, ACC_MAX)), "move"):
                s.target = pos
                s.torque = True  # writing a goal enables torque on STS servos

    def spin(self, servo_id: int, speed: int, acc: int):
        with self.lock:
            s = self._get(servo_id)
            if s.mode != MODE_WHEEL:
                raise ValueError(f"Servo {servo_id} is in position mode; switch to speed mode first")
            spd = _clamp(speed, -SPEED_MAX, SPEED_MAX)
            if self._write(s, self.sdk.WriteSpec(s.id, spd, _clamp(acc, 0, ACC_MAX)), "spin"):
                s.target_speed = spd
                if spd:
                    s.torque = True


class MockServoBus(BaseBus):
    """Simulated servos for developing the UI without hardware."""

    def __init__(self, servos: list[dict]):
        super().__init__(servos)
        self._last = time.monotonic()
        self._move_speed: dict[int, int] = {}
        self._pos_f: dict[int, float] = {}

    def open(self):
        for s in self.servos.values():
            s.online = True
            s.torque = True
            s.position = (s.min_pos + s.max_pos) // 2
            s.target = s.position
            s.voltage = 12.0
            s.temp = 30
            self._pos_f[s.id] = float(s.position)
        print("Mock bus: simulating", len(self.servos), "servos")

    def close(self):
        self.stop_all()

    def poll(self):
        now = time.monotonic()
        dt, self._last = now - self._last, now
        with self.lock:
            for s in self.servos.values():
                p = self._pos_f[s.id]
                if not s.torque:
                    s.speed = 0
                elif s.mode == MODE_WHEEL:
                    s.speed = s.target_speed
                    p = (p + s.target_speed * dt) % (POS_MAX + 1)
                elif s.target is not None:
                    vmax = self._move_speed.get(s.id, 1000) or SPEED_MAX
                    diff = s.target - p
                    step = math.copysign(min(abs(diff), vmax * dt), diff)
                    p += step
                    s.speed = int(step / dt) if dt else 0
                self._pos_f[s.id] = p
                s.position = int(p)
                s.load = int(abs(s.speed) / SPEED_MAX * 300)

    def set_mode(self, servo_id: int, mode: str):
        with self.lock:
            s = self._get(servo_id)
            s.mode = mode
            s.target_speed = 0
            s.target = s.position if mode == MODE_POSITION else None

    def set_torque(self, servo_id: int, enabled: bool):
        with self.lock:
            self._get(servo_id).torque = enabled

    def move(self, servo_id: int, position: int, speed: int, acc: int):
        with self.lock:
            s = self._get(servo_id)
            if s.mode != MODE_POSITION:
                raise ValueError(f"Servo {servo_id} is in wheel mode; switch to position mode first")
            s.target = _clamp(position, s.min_pos, s.max_pos)
            s.torque = True
            self._move_speed[servo_id] = _clamp(speed, 0, SPEED_MAX)

    def spin(self, servo_id: int, speed: int, acc: int):
        with self.lock:
            s = self._get(servo_id)
            if s.mode != MODE_WHEEL:
                raise ValueError(f"Servo {servo_id} is in position mode; switch to speed mode first")
            s.target_speed = _clamp(speed, -SPEED_MAX, SPEED_MAX)
            if s.target_speed:
                s.torque = True
