"""Which twist_mux input is driving the base, without ROS imports.

Mirrors the rules of twist_mux so the UIs can show who is in control:

- An input or lock with a timeout > 0 expires that long after its last
  message. Timeout 0 never expires
- A lock is on when it has expired or its last message was true. The lock
  priority is the highest priority of the locks that are on
- An input counts when it has not expired and its priority is not below the
  lock priority. The highest priority one drives

Taken from twist_mux 4.5.0 (`TwistMux::hasPriority` and the topic handles
in `twist_mux.cpp`); re-check these rules when twist_mux is upgraded. Its
diagnostics only say whether each input is masked and the lock priority,
not which input drives, so they cannot replace this copy.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Any

TOPICS_PREFIX = "topics"
LOCKS_PREFIX = "locks"
# twist_mux clamps priorities to this range.
MIN_PRIORITY = 0
MAX_PRIORITY = 255
NONE = "none"
LOCKED = "locked"


@dataclass(frozen=True)
class MuxEntry:
    """One twist_mux input or lock, as configured."""

    name: str
    topic: str
    timeout_s: float
    priority: int


def parse_entries(params: Mapping[str, Any], prefix: str) -> list[MuxEntry]:
    """Build entries from flat ROS params like `topics.web.topic`.

    Raises ValueError on a missing or invalid field.
    """
    fields: dict[str, dict[str, Any]] = {}
    for key, value in params.items():
        parts = key.split(".")
        if len(parts) != 3 or parts[0] != prefix:
            continue
        fields.setdefault(parts[1], {})[parts[2]] = value
    return [_entry(prefix, name, f) for name, f in sorted(fields.items())]


def _entry(prefix: str, name: str, fields: dict[str, Any]) -> MuxEntry:
    where = f"{prefix}.{name}"
    missing = {"topic", "timeout", "priority"} - fields.keys()
    if missing:
        raise ValueError(f"{where} is missing {', '.join(sorted(missing))}")
    topic = fields["topic"]
    if not isinstance(topic, str) or not topic:
        raise ValueError(f"{where}.topic must be a non-empty string")
    timeout = fields["timeout"]
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
        raise ValueError(f"{where}.timeout must be a number")
    if timeout < 0:
        raise ValueError(f"{where}.timeout must be >= 0, got {timeout}")
    priority = fields["priority"]
    if isinstance(priority, bool) or not isinstance(priority, int):
        raise ValueError(f"{where}.priority must be an integer")
    if not MIN_PRIORITY <= priority <= MAX_PRIORITY:
        raise ValueError(
            f"{where}.priority must be in {MIN_PRIORITY}..{MAX_PRIORITY}, "
            f"got {priority}"
        )
    return MuxEntry(name, topic, float(timeout), priority)


class MuxState:
    """Last message times of every input and lock, and who drives."""

    def __init__(
        self, inputs: Iterable[MuxEntry], locks: Iterable[MuxEntry]
    ) -> None:
        self.inputs = {entry.name: entry for entry in inputs}
        self.locks = {entry.name: entry for entry in locks}
        self._input_seen: dict[str, float] = {}
        self._lock_seen: dict[str, float] = {}
        self._lock_data: dict[str, bool] = {}

    def on_input(self, name: str, now: float) -> None:
        self._input_seen[name] = now

    def on_lock(self, name: str, data: bool, now: float) -> None:
        self._lock_seen[name] = now
        self._lock_data[name] = data

    def active(self, now: float) -> str:
        """Name of the input driving now, `locked` or `none`."""
        lock_priority = self.lock_priority(now)
        best: MuxEntry | None = None
        for entry in self.inputs.values():
            # Unlike twist_mux, an input that never sent anything is not
            # active, even with timeout 0.
            seen = self._input_seen.get(entry.name)
            if seen is None or _expired(entry, seen, now):
                continue
            if entry.priority < lock_priority:
                continue
            if best is None or entry.priority > best.priority:
                best = entry
        if best is not None:
            return best.name
        return LOCKED if lock_priority > MIN_PRIORITY else NONE

    def lock_priority(self, now: float) -> int:
        priority = MIN_PRIORITY
        for entry in self.locks.values():
            # A lock with a timeout starts expired, so on, until its first
            # message, like in twist_mux.
            seen = self._lock_seen.get(entry.name)
            if _expired(entry, seen, now) or self._lock_data.get(entry.name):
                priority = max(priority, entry.priority)
        return priority


def _expired(entry: MuxEntry, seen: float | None, now: float) -> bool:
    if entry.timeout_s == 0:
        return False
    return seen is None or now - seen > entry.timeout_s
