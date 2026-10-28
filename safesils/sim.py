"""SILS loop: plant, sensors, bus and controller co-simulated on a 1 ms base tick.

The controller is reached only through `ControllerUnderTest`, so a Simulink or
OpenModelica controller exported as an FMU can replace the Python reference
controller without touching scenarios, logging or the judge.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
import pandas as pd

from .bus import Channel, Message
from .controller import AEBController
from .plant import BrakeActuator, Sensor, VehiclePlant
from .scenario import Scenario

DT_MS = 1
SENSOR_PERIOD_MS = 50
CAMERA_PHASE_MS = 20


class ControllerUnderTest(Protocol):
    def step(self, t_ms: int, radar_msgs: list[Message], camera_msgs: list[Message],
             ego_speed_mps: float, driver_brake: bool, steer_rate_dps: float,
             brake_cmd_fault: float | None = None) -> dict: ...


@dataclass
class RunResult:
    scenario: Scenario
    signals: pd.DataFrame
    collided: bool
    impact_speed_mps: float


def _driver_inputs(scn: Scenario, t_ms: int) -> tuple[bool, float]:
    d, t_s = scn.driver, t_ms / 1000.0
    braking = d.brake_at_s is not None and t_s >= d.brake_at_s
    steering = d.steer_at_s is not None and d.steer_at_s <= t_s < d.steer_at_s + d.steer_duration_s
    return braking, d.steer_rate_dps if steering else 0.0


def run(scn: Scenario, controller: ControllerUnderTest | None = None) -> RunResult:
    rng = np.random.default_rng(scn.seed)
    ctrl = controller or AEBController()
    plant = VehiclePlant(scn)
    sensors = {name: Sensor(name, scn, rng) for name in ("radar", "camera")}
    channels = {name: Channel(name, scn) for name in ("radar", "camera")}
    actuator = BrakeActuator(scn)

    rows: list[dict] = []
    out: dict = {}
    next_ctrl_ms = 0
    bad_rx = {"radar": False, "camera": False}
    good_rx = {"radar": False, "camera": False}
    last_sent = {"radar": -1, "camera": -1}
    bus_view: dict[str, dict | None] = {"radar": None, "camera": None}
    aeb_decel = 0.0
    duration_ms = int(scn.duration_s * 1000)

    for t in range(0, duration_ms, DT_MS):
        truth = plant.truth()
        if t % SENSOR_PERIOD_MS == 0:
            channels["radar"].send(t, sensors["radar"].measure(t, truth))
        if t % SENSOR_PERIOD_MS == CAMERA_PHASE_MS:
            channels["camera"].send(t, sensors["camera"].measure(t, truth))

        driver_brake, steer_rate = _driver_inputs(scn, t)
        if t >= next_ctrl_ms:
            inbox = {n: ch.receive(t) for n, ch in channels.items()}
            for n, msgs in inbox.items():
                bad_rx[n] = any(m.injected_bad for m in msgs)
                good_rx[n] = False
                for m in msgs:  # ground truth: authentic, in-order frames
                    if not m.injected_bad and m.sent_ms > last_sent[n]:
                        good_rx[n] = True
                        last_sent[n] = m.sent_ms
                        bus_view[n] = m.payload
            corrupt = scn.active_faults(t, "brake_cmd_corrupt")
            out = ctrl.step(t, inbox["radar"], inbox["camera"], plant.ego_v, driver_brake, steer_rate,
                            corrupt[0].params["value_mps2"] if corrupt else None)
            overrun = scn.active_faults(t, "task_overrun")
            next_ctrl_ms = t + (int(overrun[0].params["period_ms"]) if overrun else 10)
            ctrl_tick = True
        else:
            bad_rx = {"radar": False, "camera": False}
            good_rx = {"radar": False, "camera": False}
            ctrl_tick = False

        aeb_decel = actuator.step(t, DT_MS / 1000.0, out["brake_req_mps2"])
        driver_decel = scn.driver.brake_mps2 if driver_brake else 0.0
        plant.step(t, DT_MS / 1000.0, max(aeb_decel, driver_decel))

        rows.append({
            "t_ms": t,
            "ctrl_tick": ctrl_tick,
            "ego_speed_mps": plant.ego_v,
            "true_range_m": truth.range_m,
            "true_rel_vel_mps": truth.rel_vel_mps,
            "true_ttc_s": truth.ttc_s,
            "aeb_decel_mps2": aeb_decel,
            "driver_brake": driver_brake,
            "steer_rate_dps": steer_rate,
            "bad_rx_radar": bad_rx["radar"],
            "bad_rx_camera": bad_rx["camera"],
            "good_rx_radar": good_rx["radar"],
            "good_rx_camera": good_rx["camera"],
            **{f"bus_{n}_range_m": (v["range_m"] if v and v["obj"] and v["status_ok"] else float("nan"))
               for n, v in bus_view.items()},
            "faults_active": ",".join(f"{f.type}:{f.target or '-'}" for f in scn.faults if f.active(t)),
            **out,
        })
        if plant.collided:
            break

    df = pd.DataFrame(rows)
    return RunResult(scn, df, plant.collided, plant.impact_speed_mps)
