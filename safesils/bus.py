"""Virtual CAN bus with E2E protection (CRC + alive counter) and fault injection."""
from __future__ import annotations

import json
import zlib
from dataclasses import dataclass

from .scenario import Scenario

COUNTER_MOD = 15


def crc8(payload: dict, counter: int) -> int:
    data = json.dumps(payload, sort_keys=True).encode() + bytes([counter])
    return zlib.crc32(data) & 0xFF


@dataclass
class Message:
    sender: str
    counter: int
    payload: dict
    crc: int
    sent_ms: int
    deliver_ms: int
    injected_bad: bool = False  # ground truth for the judge, invisible to the controller


class Channel:
    """One sensor-to-controller channel. Faults are applied at send time."""

    def __init__(self, name: str, scn: Scenario) -> None:
        self.name = name
        self.scn = scn
        self.counter = 0
        self._pending: list[Message] = []

    def send(self, t_ms: int, payload: dict) -> None:
        faults = {f.type: f for f in self.scn.active_faults(t_ms, target=self.name)}
        if "bus_drop" in faults:
            return
        frozen = "bus_freeze_counter" in faults
        if not frozen:
            self.counter = (self.counter + 1) % COUNTER_MOD
        msg = Message(self.name, self.counter, dict(payload), crc8(payload, self.counter), t_ms, t_ms, injected_bad=frozen)
        if "bus_corrupt" in faults:
            msg.crc ^= 0xFF
            msg.injected_bad = True
        if "bus_delay" in faults:
            msg.deliver_ms = t_ms + int(faults["bus_delay"].params["delay_ms"])
        self._pending.append(msg)

    def receive(self, t_ms: int) -> list[Message]:
        due = sorted((m for m in self._pending if m.deliver_ms <= t_ms), key=lambda m: m.deliver_ms)
        self._pending = [m for m in self._pending if m.deliver_ms > t_ms]
        return due
