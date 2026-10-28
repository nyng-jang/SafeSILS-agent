"""Scenario generation: requirement baselines, boundary sweeps and fault combinations.

The LLM agent (safesils.agent) calls these as tools and adds its own scenarios; this
module is the deterministic part so that every generated TC is reproducible
from (kind, requirement, seed).
"""
from __future__ import annotations

import numpy as np

from .scenario import Driver, Fault, Lead, Scenario

# which requirements a fault type exercises (used for traceability of generated TCs)
FAULT_TO_REQS = {
    "bus_corrupt": ["TSR-01"],
    "bus_freeze_counter": ["TSR-01"],
    "bus_drop": ["TSR-02", "TSR-07"],
    "bus_delay": ["TSR-02"],
    "sensor_offset": ["TSR-03"],
    "sensor_noise": ["TSR-03"],
    "sensor_stuck": ["TSR-03"],
    "ghost_object": ["TSR-03"],
    "sensor_dropout": ["TSR-07"],
    "task_overrun": ["TSR-08", "TSR-05"],
    "brake_cmd_corrupt": ["TSR-04", "TSR-05"],
    "actuator_limit": ["SG-03"],
}

APPROACH = dict(ego_speed_kph=50.0, lead=Lead(gap_m=60.0, speed_kph=20.0))
STOPPED_LEAD = dict(ego_speed_kph=50.0, lead=Lead(gap_m=40.0, speed_kph=0.0))
FREE_ROAD = dict(ego_speed_kph=70.0, lead=Lead(present=False))


def baseline() -> list[Scenario]:
    """One or two hand-written TCs per requirement: the regression set."""
    S = Scenario
    return [
        S("TC-SG03-001", ["SG-03"], "Lead vehicle stopped 40 m ahead at 50 km/h", **STOPPED_LEAD),
        S("TC-SG03-002", ["SG-03"], "Lead brakes at 6 m/s^2 from 50 km/h, ego 70 km/h, gap 30 m",
          ego_speed_kph=70, lead=Lead(gap_m=30, speed_kph=50, decel_at_s=2.0, decel_mps2=6.0)),
        S("TC-SG01-001", ["SG-01"], "Free road at 70 km/h: no braking", duration_s=6, **FREE_ROAD),
        S("TC-TSR01-001", ["TSR-01", "SG-03"], "Radar CRC corruption for 300 ms while approaching",
          faults=[Fault("bus_corrupt", 3.0, 3.3, "radar")], **APPROACH),
        S("TC-TSR01-002", ["TSR-01"], "Camera alive counter frozen for 200 ms",
          faults=[Fault("bus_freeze_counter", 3.0, 3.2, "camera")], **APPROACH),
        S("TC-TSR02-001", ["TSR-02"], "Radar frames lost for 500 ms",
          faults=[Fault("bus_drop", 2.0, 2.5, "radar")], **APPROACH),
        S("TC-TSR02-002", ["TSR-02"], "Radar frames delayed 140 ms (just under timeout)",
          faults=[Fault("bus_delay", 2.0, 4.0, "radar", {"delay_ms": 140})], **APPROACH),
        S("TC-FSR01-001", ["FSR-01", "SG-01"], "Radar ghost object 20 m ahead on a free road at 80 km/h",
          ego_speed_kph=80, lead=Lead(present=False), faults=[Fault("ghost_object", 2.0, 4.0, "radar")]),
        S("TC-TSR03-002", ["TSR-03"], "Camera range offset +8 m while approaching",
          faults=[Fault("sensor_offset", 1.0, None, "camera", {"offset_m": 8.0})], **APPROACH),
        S("TC-TSR04-001", ["TSR-04", "SG-01"], "Internal brake command corrupted to 9 m/s^2 on a free road",
          faults=[Fault("brake_cmd_corrupt", 3.0, 3.5)], **FREE_ROAD),
        S("TC-TSR05-001", ["TSR-05", "TSR-08"], "Task overrun (30 ms period) during AEB braking",
          faults=[Fault("task_overrun", 2.6, None, params={"period_ms": 30})], **STOPPED_LEAD),
        S("TC-TSR06-001", ["TSR-06"], "Driver steers away at 150 deg/s during AEB braking",
          driver=Driver(steer_at_s=2.6, steer_rate_dps=150), **STOPPED_LEAD),
        S("TC-TSR06-002", ["TSR-06"], "Driver presses the brake during AEB braking",
          driver=Driver(brake_at_s=2.6, brake_mps2=6.0), **STOPPED_LEAD),
        S("TC-TSR07-001", ["TSR-07", "SG-03"], "Camera reports sensor failure while approaching",
          faults=[Fault("sensor_dropout", 2.0, None, "camera")], **STOPPED_LEAD),
        S("TC-TSR07-002", ["TSR-07"], "Radar frames permanently lost",
          faults=[Fault("bus_drop", 2.0, None, "radar")], **APPROACH),
        S("TC-TSR08-001", ["TSR-08"], "Control task period stretched to 25 ms",
          faults=[Fault("task_overrun", 3.0, None, params={"period_ms": 25})], **FREE_ROAD),
        S("TC-TSR08-002", ["TSR-08"], "Control task period 14 ms (within 1.5x tolerance)",
          faults=[Fault("task_overrun", 3.0, None, params={"period_ms": 14})], **FREE_ROAD),
    ]


def sweep(req_id: str, n: int = 5) -> list[Scenario]:
    """Boundary-value sweep of the parameter that decides a requirement."""
    out: list[Scenario] = []
    if req_id == "TSR-02":
        for i, d in enumerate(np.linspace(120, 180, n)):
            out.append(Scenario(f"TC-TSR02-1{i:02d}", ["TSR-02"], f"Radar delay {d:.0f} ms",
                                faults=[Fault("bus_delay", 2.0, 4.0, "radar", {"delay_ms": int(d)})], **APPROACH))
    elif req_id == "TSR-03":
        for i, off in enumerate(np.linspace(2.0, 4.0, n)):
            out.append(Scenario(f"TC-TSR03-1{i:02d}", ["TSR-03"], f"Camera offset {off:.2f} m",
                                faults=[Fault("sensor_offset", 1.0, None, "camera", {"offset_m": round(float(off), 2)})],
                                **APPROACH))
    elif req_id == "TSR-08":
        for i, per in enumerate(np.linspace(13, 17, n)):
            out.append(Scenario(f"TC-TSR08-1{i:02d}", ["TSR-08"], f"Task period {per:.0f} ms",
                                faults=[Fault("task_overrun", 3.0, None, params={"period_ms": int(round(per))})],
                                **FREE_ROAD))
    elif req_id == "TSR-06":
        for i, rate in enumerate(np.linspace(80, 120, n)):
            out.append(Scenario(f"TC-TSR06-1{i:02d}", ["TSR-06"], f"Driver steer rate {rate:.0f} deg/s",
                                driver=Driver(steer_at_s=2.6, steer_rate_dps=float(rate)), **STOPPED_LEAD))
    else:
        raise ValueError(f"no sweep defined for {req_id}")
    return out


_COMBO_CATALOG = [
    ("bus_delay", "radar", lambda r: {"delay_ms": int(r.integers(100, 150))}),
    ("bus_delay", "camera", lambda r: {"delay_ms": int(r.integers(100, 150))}),
    ("sensor_offset", "camera", lambda r: {"offset_m": round(float(r.uniform(2.0, 3.5)), 2)}),
    ("sensor_offset", "radar", lambda r: {"offset_m": round(float(r.uniform(-3.5, -2.0)), 2)}),
    ("ghost_object", "radar", lambda r: {"range_m": round(float(r.uniform(10, 40)), 1), "rel_vel_mps": -10.0}),
    ("sensor_noise", "camera", lambda r: {"sigma_m": round(float(r.uniform(1.0, 3.0)), 2)}),
    ("bus_corrupt", "radar", lambda r: {}),
    ("bus_freeze_counter", "camera", lambda r: {}),
    ("task_overrun", None, lambda r: {"period_ms": int(r.integers(14, 16))}),
    ("sensor_stuck", "radar", lambda r: {}),
]


def combinations(n: int = 20, seed: int = 0) -> list[Scenario]:
    """Exploratory TCs: two near-threshold faults overlapping in time."""
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        picks = rng.choice(len(_COMBO_CATALOG), size=2, replace=False)
        start = round(float(rng.uniform(1.0, 2.5)), 2)
        faults, req_ids = [], []
        for k in picks:
            ftype, target, gen = _COMBO_CATALOG[k]
            s = round(start + float(rng.uniform(0, 0.3)), 2)
            faults.append(Fault(ftype, s, round(s + float(rng.uniform(0.3, 2.0)), 2), target, gen(rng)))
            req_ids += FAULT_TO_REQS[ftype]
        setup = [APPROACH, STOPPED_LEAD, FREE_ROAD][int(rng.integers(0, 3))]
        desc = " + ".join(f"{f.type}({f.target or 'ecu'})" for f in faults)
        out.append(Scenario(f"TC-EXP-{seed:03d}-{i:03d}", sorted(set(req_ids)) + ["SG-01", "SG-03"], desc,
                            seed=int(rng.integers(0, 2**31)), faults=faults, exploratory=True, **setup))
    return out
