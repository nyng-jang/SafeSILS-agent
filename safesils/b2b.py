"""Back-to-back comparison of the Python reference controller with the MATLAB/Simulink one.

Two levels:
  * open loop: record every controller activation of a Python closed-loop run and
    replay the identical inputs through matlab/aeb_controller_step.m in GNU Octave
    or MATLAB; every output of every activation must match.
  * closed loop: run the same scenario with the exported FMU in the loop and
    compare the judge's verdicts and key signals with the Python run.

export_vectors writes the recorded inputs and the Python outputs as CSV test
vectors, so matlab/b2b_simulink.m can run the open-loop comparison against the
Simulink model inside MATLAB, without Python on that machine.
"""
from __future__ import annotations

import math
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .controller import AEBController
from .external import OUTPUT_NAMES, decode_outputs, encode_outputs, input_vector
from .judge import evaluate
from .scenario import Scenario
from .sim import ControllerUnderTest, run

MATLAB_DIR = Path(__file__).resolve().parent.parent / "matlab"
COMPARED = [n for n in OUTPUT_NAMES if n != "brake_cmd_internal"] + ["brake_cmd_internal"]


class Recorder:
    """Wraps a controller and records its inputs and outputs per activation."""

    def __init__(self, inner: ControllerUnderTest) -> None:
        self.inner = inner
        self.inputs: list[np.ndarray] = []
        self.outputs: list[dict] = []

    def step(self, t_ms, radar_msgs, camera_msgs, ego_speed_mps, driver_brake, steer_rate_dps, brake_cmd_fault=None):
        self.inputs.append(input_vector(t_ms, radar_msgs, camera_msgs, ego_speed_mps, driver_brake,
                                        steer_rate_dps, brake_cmd_fault))
        out = self.inner.step(t_ms, radar_msgs, camera_msgs, ego_speed_mps, driver_brake, steer_rate_dps,
                              brake_cmd_fault)
        self.outputs.append(out)
        return out


@dataclass
class Mismatch:
    step: int
    t_ms: int
    signal: str
    python: object
    matlab: object


def _same(a, b) -> bool:
    if isinstance(a, float) or isinstance(b, float):
        a, b = float(a), float(b)
        if math.isnan(a) and math.isnan(b):
            return True
        return a == b or abs(a - b) <= 1e-9 * max(1.0, abs(a))
    return a == b


def replay_engine() -> list[str]:
    if shutil.which("octave"):
        return ["octave", "--no-gui", "--quiet", "--eval"]
    if shutil.which("matlab"):
        return ["matlab", "-batch"]
    raise RuntimeError("needs GNU Octave or MATLAB on PATH for the open-loop replay")


def record(scn: Scenario) -> Recorder:
    rec = Recorder(AEBController())
    run(scn, rec)
    return rec


def _write_csv(path: Path, rows) -> None:
    """CSV with full double precision and MATLAB spellings of Inf/NaN."""
    def fmt(v: float) -> str:
        if math.isnan(v):
            return "NaN"
        if math.isinf(v):
            return "Inf" if v > 0 else "-Inf"
        return f"{v:.17g}"
    path.write_text("".join(",".join(fmt(float(v)) for v in row) + "\n" for row in rows))


def export_vectors(scn: Scenario, out_dir: Path) -> tuple[Path, Path]:
    """Write <tc_id>.in.csv (one activation per row, layout of replay_inputs.m) and
    <tc_id>.out.csv (the Python reference outputs, layout of aeb_controller_step.m)."""
    rec = record(scn)
    out_dir.mkdir(parents=True, exist_ok=True)
    fin, fout = out_dir / f"{scn.tc_id}.in.csv", out_dir / f"{scn.tc_id}.out.csv"
    _write_csv(fin, rec.inputs)
    _write_csv(fout, (encode_outputs(o) for o in rec.outputs))
    return fin, fout


def open_loop(scn: Scenario) -> list[Mismatch]:
    rec = record(scn)
    with tempfile.TemporaryDirectory() as d:
        fin, fout = Path(d) / "in.csv", Path(d) / "out.csv"
        np.savetxt(fin, np.vstack(rec.inputs), delimiter=",", fmt="%.17g")
        cmd = f"addpath('{MATLAB_DIR.as_posix()}'); replay_inputs('{fin.as_posix()}', '{fout.as_posix()}');"
        subprocess.run([*replay_engine(), cmd], check=True, capture_output=True, text=True)
        Y = np.genfromtxt(fout, delimiter=",")
    Y = np.atleast_2d(Y)
    diffs = []
    for k, (u, py, y) in enumerate(zip(rec.inputs, rec.outputs, Y)):
        ml = decode_outputs(y)
        for name in COMPARED:
            if not _same(py[name], ml[name]):
                diffs.append(Mismatch(k, int(u[0]), name, py[name], ml[name]))
    return diffs


def closed_loop(scn: Scenario, controller: ControllerUnderTest) -> list[str]:
    """Differences in verdicts and key signals between the Python and the given controller."""
    ref, other = run(scn), run(scn, controller)
    vr = {v.req_id: v.verdict for v in evaluate(ref)}
    vo = {v.req_id: v.verdict for v in evaluate(other)}
    diffs = [f"{k}: python {vr[k]} vs fmu {vo.get(k)}" for k in vr if vr[k] != vo.get(k)]
    n = min(len(ref.signals), len(other.signals))
    for col in ("aeb_state", "brake_req_mps2", "fault_lamp"):
        a, b = ref.signals[col].iloc[:n].to_numpy(), other.signals[col].iloc[:n].to_numpy()
        bad = np.nonzero(a != b)[0]
        if len(bad):
            i = int(bad[0])
            diffs.append(f"{col} differs first at t={int(ref.signals.t_ms.iloc[i])} ms: {a[i]} vs {b[i]}")
    return diffs
