"""Controllers that live outside Python: Simulink FMU and the MATLAB/Octave reference.

Both receive exactly the signals a real ECU would see: up to FRAME_SLOTS raw CAN
frames per sensor and activation, plus vehicle and driver signals. The output
vector layout is fixed by matlab/aeb_controller_step.m.
"""
from __future__ import annotations

import math

import numpy as np

from .bus import Message, encode

FRAME_SLOTS = 4
FRAME_FIELDS = ("present", "counter", "crc", "obj", "status_ok", "range_raw", "vel_raw")
STATES = ("MONITOR", "STANDBY", "WARNING", "PARTIAL_BRAKE", "FULL_BRAKE", "DEGRADED", "FAIL_SAFE")
OUTPUT_NAMES = (
    "aeb_state", "brake_req_mps2", "brake_cmd_internal", "fcw_warning", "fault_lamp", "ttc_s",
    "confirmed", "override", "e2e_radar", "e2e_camera", "timeout_radar", "timeout_camera",
    "last_valid_radar", "last_valid_camera", "mismatch", "monitor_block", "wd_violations",
    "radar_range_rx", "camera_range_rx",
)
BOOL_OUTPUTS = {"fcw_warning", "fault_lamp", "confirmed", "override", "e2e_radar", "e2e_camera",
                "timeout_radar", "timeout_camera", "mismatch", "monitor_block"}


class FrameOverflow(RuntimeError):
    pass


def frames_to_slots(msgs: list[Message]) -> np.ndarray:
    """Encode received messages as a FRAME_SLOTS x 7 matrix of raw CAN signals."""
    if len(msgs) > FRAME_SLOTS:
        raise FrameOverflow(f"{len(msgs)} frames in one activation, interface carries {FRAME_SLOTS}")
    slots = np.zeros((FRAME_SLOTS, len(FRAME_FIELDS)))
    for i, m in enumerate(msgs):
        obj, ok, range_raw, vel_raw = encode(m.payload)
        slots[i] = (1, m.counter, m.crc, obj, ok, range_raw, vel_raw)
    return slots


def decode_outputs(y: np.ndarray | list[float]) -> dict:
    out: dict = {}
    for name, v in zip(OUTPUT_NAMES, y):
        v = float(v)
        if name == "aeb_state":
            out[name] = STATES[int(round(v))]
        elif name in BOOL_OUTPUTS:
            out[name] = v >= 0.5
        elif name in ("wd_violations", "last_valid_radar", "last_valid_camera"):
            out[name] = int(round(v))
        else:
            out[name] = v if not math.isnan(v) else math.nan
    return out


def input_vector(t_ms: int, radar_msgs: list[Message], camera_msgs: list[Message], ego_speed_mps: float,
                 driver_brake: bool, steer_rate_dps: float, brake_cmd_fault: float | None) -> np.ndarray:
    """Flat input row: t, 28 radar slot values, 28 camera slot values, vehicle/driver/fault signals."""
    return np.concatenate([
        [t_ms],
        frames_to_slots(radar_msgs).ravel(),
        frames_to_slots(camera_msgs).ravel(),
        [ego_speed_mps, float(driver_brake), steer_rate_dps,
         float(brake_cmd_fault is not None), brake_cmd_fault or 0.0],
    ])


INPUT_PORTS = (("reset", 1), ("t_ms", 1), ("radar_frames", FRAME_SLOTS * len(FRAME_FIELDS)),
               ("camera_frames", FRAME_SLOTS * len(FRAME_FIELDS)), ("ego_speed_mps", 1), ("driver_brake", 1),
               ("steer_rate_dps", 1), ("fault_active", 1), ("fault_value", 1))


def _order_by_ports(variables, ports: tuple[str, ...]) -> list:
    """Sort FMU scalar variables by port order, then by element index (e.g. radar_frames[12])."""
    def key(v):
        base, _, idx = v.name.partition("[")
        nums = [int(n) for n in idx.rstrip("]").split(",") if n.strip().isdigit()] if idx else [0]
        return ports.index(base), nums
    return sorted(variables, key=key)


class FMUController:
    """Simulink controller exported as an FMI 2.0 co-simulation FMU (needs `pip install fmpy`).

    Built by matlab/build_aeb_model.m: one input port per entry of INPUT_PORTS
    (frame ports are 28-element vectors, row-major 4x7) and one scalar output
    port per entry of OUTPUT_NAMES. Each activation is one 10 ms doStep.
    """

    def __init__(self, fmu_path: str) -> None:
        from fmpy import extract, read_model_description
        from fmpy.fmi2 import FMU2Slave

        md = read_model_description(fmu_path)
        ins = [v for v in md.modelVariables if v.causality == "input"]
        outs = {v.name: v for v in md.modelVariables if v.causality == "output"}
        ports = tuple(name for name, _ in INPUT_PORTS)
        n_in = sum(width for _, width in INPUT_PORTS)
        if len(ins) != n_in or any(v.name.partition("[")[0] not in ports for v in ins):
            raise ValueError(f"FMU inputs {[v.name for v in ins][:5]}... do not match {ports} ({n_in} scalars)")
        missing = [n for n in OUTPUT_NAMES if n not in outs]
        if missing:
            raise ValueError(f"FMU lacks outputs {missing}")
        self._inputs = [v.valueReference for v in _order_by_ports(ins, ports)]
        self._outputs = [outs[n].valueReference for n in OUTPUT_NAMES]
        self._fmu = FMU2Slave(guid=md.guid, unzipDirectory=extract(fmu_path),
                              modelIdentifier=md.coSimulation.modelIdentifier, instanceName="aeb")
        self._fmu.instantiate()
        self._fmu.setupExperiment(startTime=0.0)
        self._fmu.enterInitializationMode()
        self._fmu.exitInitializationMode()
        self._time = 0.0
        self._first = True

    def step(self, t_ms, radar_msgs, camera_msgs, ego_speed_mps, driver_brake, steer_rate_dps,
             brake_cmd_fault=None) -> dict:
        u = input_vector(t_ms, radar_msgs, camera_msgs, ego_speed_mps, driver_brake, steer_rate_dps, brake_cmd_fault)
        self._fmu.setReal(self._inputs, [float(self._first), *u.tolist()])
        self._fmu.doStep(currentCommunicationPoint=self._time, communicationStepSize=0.01)
        self._time += 0.01
        self._first = False
        return decode_outputs(self._fmu.getReal(self._outputs))

    def close(self) -> None:
        self._fmu.terminate()
        self._fmu.freeInstance()
