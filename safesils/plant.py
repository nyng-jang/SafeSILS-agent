"""Plant model: ego and lead vehicle longitudinal dynamics, sensors, brake actuator."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .scenario import Scenario

KPH = 1 / 3.6


@dataclass
class Truth:
    range_m: float
    rel_vel_mps: float  # lead speed - ego speed (negative: closing)
    ttc_s: float
    lead_present: bool
    collided: bool


class VehiclePlant:
    """Point-mass longitudinal model of ego and lead vehicle."""

    def __init__(self, scn: Scenario) -> None:
        self.scn = scn
        self.ego_x = 0.0
        self.ego_v = scn.ego_speed_kph * KPH
        self.lead_x = scn.lead.gap_m
        self.lead_v = scn.lead.speed_kph * KPH
        self.collided = False
        self.impact_speed_mps = 0.0

    def step(self, t_ms: int, dt_s: float, ego_decel: float) -> None:
        if self.collided:
            return
        self.ego_v = max(0.0, self.ego_v - ego_decel * dt_s)
        self.ego_x += self.ego_v * dt_s
        lead = self.scn.lead
        if lead.present:
            if lead.decel_at_s is not None and t_ms / 1000.0 >= lead.decel_at_s:
                self.lead_v = max(0.0, self.lead_v - lead.decel_mps2 * dt_s)
            self.lead_x += self.lead_v * dt_s
            if self.lead_x - self.ego_x <= 0.0:
                self.collided = True
                self.impact_speed_mps = self.ego_v - self.lead_v

    def truth(self) -> Truth:
        if not self.scn.lead.present:
            return Truth(math.inf, 0.0, math.inf, False, False)
        rng = self.lead_x - self.ego_x
        rel = self.lead_v - self.ego_v
        ttc = rng / -rel if rel < -0.1 else math.inf
        return Truth(rng, rel, ttc, True, self.collided)


class Sensor:
    """Radar or camera object sensor with fault hooks applied before transmission."""

    SPEC = {
        "radar": {"max_range": 150.0, "sigma": 0.2},
        "camera": {"max_range": 80.0, "sigma": 0.5},
    }

    def __init__(self, name: str, scn: Scenario, rng: np.random.Generator) -> None:
        self.name = name
        self.scn = scn
        self.rng = rng
        self.spec = self.SPEC[name]
        self._stuck: dict | None = None

    def measure(self, t_ms: int, truth: Truth) -> dict:
        sigma = self.spec["sigma"]
        obj = truth.lead_present and truth.range_m <= self.spec["max_range"]
        m = {
            "obj": bool(obj),
            "range_m": float(truth.range_m + self.rng.normal(0, sigma)) if obj else 0.0,
            "rel_vel_mps": float(truth.rel_vel_mps + self.rng.normal(0, 0.05)) if obj else 0.0,
            "status_ok": True,
        }
        for f in self.scn.active_faults(t_ms, target=self.name):
            p = f.params
            if f.type == "sensor_offset" and m["obj"]:
                m["range_m"] += p["offset_m"]
            elif f.type == "sensor_noise" and m["obj"]:
                m["range_m"] += float(self.rng.normal(0, p["sigma_m"]))
            elif f.type == "sensor_dropout":
                m = {"obj": False, "range_m": 0.0, "rel_vel_mps": 0.0, "status_ok": False}
            elif f.type == "ghost_object":
                if not m["obj"] or p["range_m"] < m["range_m"]:
                    m.update(obj=True, range_m=float(p["range_m"]), rel_vel_mps=float(p["rel_vel_mps"]))
            elif f.type == "sensor_stuck":
                if self._stuck is None:
                    self._stuck = dict(m)
                m = dict(self._stuck)
        if not self.scn.active_faults(t_ms, "sensor_stuck", self.name):
            self._stuck = None
        return m


class BrakeActuator:
    """First-order lag brake actuator (tau 50 ms, max 10 m/s^2)."""

    TAU_S = 0.05
    MAX_DECEL = 10.0

    def __init__(self, scn: Scenario) -> None:
        self.scn = scn
        self.decel = 0.0

    def step(self, t_ms: int, dt_s: float, request: float) -> float:
        target = min(max(request, 0.0), self.MAX_DECEL)
        for f in self.scn.active_faults(t_ms, "actuator_limit"):
            target *= f.params["factor"]
        self.decel += (target - self.decel) * dt_s / self.TAU_S
        return self.decel
