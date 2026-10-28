"""Scenario and fault definitions.

A scenario is a plain JSON document so that people, the generator and the
LLM agent can all write it. One scenario is one test case (TC) and lists the
requirement IDs it verifies, which is what the traceability matrix is built on.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

# type -> (needs a sensor target, parameter defaults)
FAULT_TYPES: dict[str, tuple[bool, dict[str, Any]]] = {
    "bus_delay": (True, {"delay_ms": 100}),
    "bus_drop": (True, {}),
    "bus_corrupt": (True, {}),
    "bus_freeze_counter": (True, {}),
    "sensor_offset": (True, {"offset_m": 5.0}),
    "sensor_stuck": (True, {}),
    "sensor_noise": (True, {"sigma_m": 2.0}),
    "sensor_dropout": (True, {}),
    "ghost_object": (True, {"range_m": 20.0, "rel_vel_mps": -10.0}),
    "task_overrun": (False, {"period_ms": 25}),
    "brake_cmd_corrupt": (False, {"value_mps2": 9.0}),
    "actuator_limit": (False, {"factor": 0.5}),
}
SENSORS = ("radar", "camera")


@dataclass
class Fault:
    type: str
    start_s: float
    end_s: float | None = None  # None: active until the end of the run
    target: str | None = None
    params: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.type not in FAULT_TYPES:
            raise ValueError(f"unknown fault type {self.type!r}")
        needs_target, defaults = FAULT_TYPES[self.type]
        if needs_target and self.target not in SENSORS:
            raise ValueError(f"fault {self.type} needs target in {SENSORS}")
        self.params = {**defaults, **self.params}

    def active(self, t_ms: int) -> bool:
        t_s = t_ms / 1000.0
        return t_s >= self.start_s and (self.end_s is None or t_s < self.end_s)

    @property
    def start_ms(self) -> int:
        return int(round(self.start_s * 1000))


@dataclass
class Lead:
    present: bool = True
    gap_m: float = 40.0
    speed_kph: float = 30.0
    decel_at_s: float | None = None
    decel_mps2: float = 0.0


@dataclass
class Driver:
    brake_at_s: float | None = None
    brake_mps2: float = 4.0
    steer_at_s: float | None = None
    steer_rate_dps: float = 150.0
    steer_duration_s: float = 0.5


@dataclass
class Scenario:
    tc_id: str
    requirements: list[str]
    description: str = ""
    duration_s: float = 8.0
    seed: int = 0
    ego_speed_kph: float = 50.0
    lead: Lead = field(default_factory=Lead)
    driver: Driver = field(default_factory=Driver)
    faults: list[Fault] = field(default_factory=list)
    exploratory: bool = False  # A-SPICE: knowledge-based TC, not tied 1:1 to a requirement

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Scenario":
        d = dict(d)
        d["lead"] = Lead(**d.get("lead", {}))
        d["driver"] = Driver(**d.get("driver", {}))
        d["faults"] = [Fault(**f) for f in d.get("faults", [])]
        return cls(**d)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def load(cls, path: str | Path) -> "Scenario":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def save(self, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(self.to_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def active_faults(self, t_ms: int, type_: str | None = None, target: str | None = None) -> list[Fault]:
        return [
            f for f in self.faults
            if f.active(t_ms) and (type_ is None or f.type == type_) and (target is None or f.target == target)
        ]
