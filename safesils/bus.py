"""Virtual CAN bus with E2E protection (CRC + alive counter) and fault injection."""
from __future__ import annotations

from dataclasses import dataclass

from .scenario import Scenario

COUNTER_MOD = 15


def encode(payload: dict) -> tuple[int, int, int, int]:
    """Pack a sensor object into CAN signals: range 0.01 m (uint16), rel. velocity 0.01 m/s (int16)."""
    range_raw = min(max(round(payload["range_m"] * 100), 0), 0xFFFF)
    vel_raw = min(max(round(payload["rel_vel_mps"] * 100), -0x8000), 0x7FFF)
    return int(bool(payload["obj"])), int(bool(payload["status_ok"])), range_raw, vel_raw


def decode(obj: int, status_ok: int, range_raw: int, vel_raw: int) -> dict:
    return {"obj": bool(obj), "status_ok": bool(status_ok), "range_m": range_raw / 100, "rel_vel_mps": vel_raw / 100}


def frame_bytes(payload: dict, counter: int) -> bytes:
    obj, ok, range_raw, vel_raw = encode(payload)
    vel_u16 = vel_raw & 0xFFFF
    return bytes([range_raw >> 8, range_raw & 0xFF, vel_u16 >> 8, vel_u16 & 0xFF, obj | (ok << 1), counter & 0x0F])


def crc8(payload: dict, counter: int) -> int:
    """CRC-8 SAE J1850 (poly 0x1D, init 0xFF, final xor 0xFF), as in AUTOSAR E2E profile 1."""
    crc = 0xFF
    for byte in frame_bytes(payload, counter):
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


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
        payload = decode(*encode(payload))  # quantised exactly as on the wire
        msg = Message(self.name, self.counter, payload, crc8(payload, self.counter), t_ms, t_ms, injected_bad=frozen)
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
