"""Rule-based pass/fail judge.

Verdicts are deterministic functions of the logged signals and the scenario.
The LLM agent never decides PASS/FAIL (tool confidence, ISO 26262-8 clause 11).
Each rule checks one requirement from requirements/aeb_safety.yaml.

Controller outputs the judge relies on (the contract for an FMU controller):
aeb_state, brake_req_mps2, fault_lamp, fcw_warning, e2e_radar, e2e_camera,
timeout_radar, timeout_camera, mismatch, monitor_block.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np
import pandas as pd

from . import requirements as reqs
from .scenario import SENSORS, Scenario
from .sim import RunResult

PASS, FAIL, NA = "PASS", "FAIL", "N/A"


@dataclass
class Verdict:
    req_id: str
    verdict: str
    measured: float | None
    limit: float | None
    detail: str

    def to_dict(self) -> dict:
        return asdict(self)


def _first(df: pd.DataFrame, mask: pd.Series, after_ms: int = -1) -> int | None:
    hit = df.loc[mask & (df.t_ms >= after_ms), "t_ms"]
    return int(hit.iloc[0]) if len(hit) else None


def _run_lengths(t: pd.Series, mask: pd.Series) -> list[tuple[int, int]]:
    """(start_ms, length_ms) for each contiguous True run of mask on a 1 ms log."""
    runs, start, prev = [], None, None
    for ti, m in zip(t.to_numpy(), mask.to_numpy()):
        if m and start is None:
            start = ti
        elif not m and start is not None:
            runs.append((int(start), int(prev - start + 1)))
            start = None
        prev = ti
    if start is not None:
        runs.append((int(start), int(prev - start + 1)))
    return runs


def _ctrl(df: pd.DataFrame) -> pd.DataFrame:
    """Controller ticks only, with sensor ranges blanked once the data is older than the timeout."""
    out = df[df.ctrl_tick].copy()
    limit = reqs.params("TSR-02")["timeout_ms"]
    for s in SENSORS:
        last_good = out.t_ms.where(out[f"good_rx_{s}"]).ffill().fillna(0)
        out.loc[out.t_ms - last_good > limit, f"bus_{s}_range_m"] = np.nan
    return out


# -- safety goals ------------------------------------------------------------------

def check_sg01(res: RunResult) -> Verdict:
    """No AEB deceleration > 1.5 m/s^2 without collision risk for longer than FTTI."""
    df = res.signals
    limit = reqs.get("SG-01")["ftti_ms"]
    # A threat episode starts when true TTC <= 3 s and lasts until closing stops.
    threat = np.zeros(len(df), dtype=bool)
    active = False
    for i, (ttc, rel) in enumerate(zip(df.true_ttc_s.to_numpy(), df.true_rel_vel_mps.to_numpy())):
        if ttc <= 3.0:
            active = True
        elif rel >= -0.1:
            active = False
        threat[i] = active
    unintended = (df.aeb_decel_mps2 > 1.5) & ~pd.Series(threat, index=df.index)
    runs = _run_lengths(df.t_ms, unintended)
    worst = max((r[1] for r in runs), default=0)
    detail = f"longest unintended braking {worst} ms" + (f" from t={runs[0][0]} ms" if runs else "")
    return Verdict("SG-01", PASS if worst <= limit else FAIL, worst, limit, detail)


def check_sg03(res: RunResult) -> Verdict:
    """Collision avoided, or the driver was warned before impact."""
    df = res.signals
    if not res.collided:
        return Verdict("SG-03", PASS, 0.0, None, "no collision")
    warned = bool(df.fcw_warning.any() or df.fault_lamp.any())
    how = "FCW" if df.fcw_warning.any() else "fault lamp" if warned else "none"
    return Verdict("SG-03", PASS if warned else FAIL, round(res.impact_speed_mps, 2), None,
                   f"collision at {res.impact_speed_mps * 3.6:.1f} km/h relative, driver informed by: {how}")


# -- functional safety requirements ---------------------------------------------------

def check_fsr01(res: RunResult) -> Verdict:
    """Automatic braking only while radar and camera confirm the same object."""
    df = _ctrl(res.signals)
    p = reqs.params("TSR-03")
    r, c = df.bus_radar_range_m, df.bus_camera_range_m
    agree = r.notna() & c.notna() & ((r - c).abs() <= np.maximum(p["abs_m"], p["rel"] * r))
    bad = df[(df.brake_req_mps2 > 0) & ~agree]
    braking_ms = int((df.brake_req_mps2 > 0).sum() * 10)
    if len(bad):
        return Verdict("FSR-01", FAIL, float(len(bad)), 0, f"brake request without confirmed object at {int(bad.t_ms.iloc[0])} ms")
    return Verdict("FSR-01", PASS, 0.0, 0, f"all braking ({braking_ms} ms of requests) backed by both sensors")


# -- technical safety requirements --------------------------------------------------------

def check_tsr01(res: RunResult) -> Verdict:
    df, scn = res.signals, res.scenario
    limit = reqs.params("TSR-01")["detect_ms"]
    worst, notes = None, []
    for s in SENSORS:
        if not [f for f in scn.faults if f.target == s and f.type in ("bus_corrupt", "bus_freeze_counter")]:
            continue
        t_bad = _first(df, df[f"bad_rx_{s}"])
        if t_bad is None:
            notes.append(f"{s}: no faulty frame reached the controller")
            continue
        t_det = _first(df, df[f"e2e_{s}"].astype(bool), t_bad)
        lat = math.inf if t_det is None else t_det - t_bad
        worst = lat if worst is None else max(worst, lat)
        notes.append(f"{s}: faulty frame at {t_bad} ms, flagged {'never' if t_det is None else f'at {t_det} ms'}")
    if worst is None:
        return Verdict("TSR-01", NA, None, limit, "; ".join(notes) or "no E2E fault injected")
    return Verdict("TSR-01", PASS if worst <= limit else FAIL, worst, limit, "; ".join(notes))


def check_tsr02(res: RunResult) -> Verdict:
    """Timeout must be flagged once no authentic in-order frame arrived for 150 ms.

    A timeout raised earlier than that is conservative (the E2E check may reject
    a frame after a counter jump), so it is reported but does not fail the rule.
    """
    df = _ctrl(res.signals)
    limit = reqs.params("TSR-02")["timeout_ms"]
    problems, notes, max_gap = [], [], 0.0
    for s in SENSORS:
        last_good = df.t_ms.where(df[f"good_rx_{s}"]).ffill().fillna(0)
        gap = df.t_ms - last_good
        max_gap = max(max_gap, float(gap.max()))
        flag = df[f"timeout_{s}"].astype(bool)
        late = df[~flag & (gap > limit + 10)]
        early = int((flag & (gap <= limit)).sum())
        if len(late):
            problems.append(f"{s}: no timeout at {int(late.t_ms.iloc[0])} ms after {int(gap[late.index[0]])} ms gap")
        if early:
            notes.append(f"{s}: conservative timeout on {early} cycles")
    detail = "; ".join(problems + notes) or f"timeout flags consistent, longest gap {int(max_gap)} ms"
    return Verdict("TSR-02", FAIL if problems else PASS, max_gap, limit, detail)


def check_tsr03(res: RunResult) -> Verdict:
    df = _ctrl(res.signals).copy()
    p = reqs.params("TSR-03")
    r, c = df.bus_radar_range_m, df.bus_camera_range_m
    disagree = (r.notna() & c.notna() & ((r - c).abs() > np.maximum(p["abs_m"], p["rel"] * r))).fillna(False)
    start = df.t_ms.where(disagree & ~disagree.shift(fill_value=False)).ffill()
    age = (df.t_ms - start).where(disagree, -1)
    must_inhibit = age >= p["debounce_ms"] + 10
    violations = df[must_inhibit & (df.brake_req_mps2 > 0)]
    if not disagree.any():
        return Verdict("TSR-03", NA, None, p["debounce_ms"], "radar and camera never disagreed")
    if len(violations):
        t0 = int(violations.t_ms.iloc[0])
        return Verdict("TSR-03", FAIL, float(age.max()), p["debounce_ms"], f"disagreement not inhibited at {t0} ms")
    return Verdict("TSR-03", PASS, float(age.max()), p["debounce_ms"], f"disagreement up to {int(age.max())} ms, braking inhibited")


def check_tsr04(res: RunResult) -> Verdict:
    df, scn = res.signals, res.scenario
    limit = reqs.params("TSR-04")["debounce_ms"] + 20  # debounce + two control cycles
    faults = [f for f in scn.faults if f.type == "brake_cmd_corrupt" and f.params["value_mps2"] > 0.1]
    if not faults:
        return Verdict("TSR-04", NA, None, limit, "no brake command corruption injected")
    f = faults[0]
    t_fs = _first(df, df.aeb_state == "FAIL_SAFE", f.start_ms)
    lat = math.inf if t_fs is None else t_fs - f.start_ms
    return Verdict("TSR-04", PASS if lat <= limit else FAIL, lat, limit,
                   f"corruption at {f.start_ms} ms, FAIL_SAFE {'never' if t_fs is None else f'at {t_fs} ms'}")


def check_tsr05(res: RunResult) -> Verdict:
    df = _ctrl(res.signals)
    limit = reqs.params("TSR-05")["react_ms"]
    fs = df.aeb_state == "FAIL_SAFE"
    if not fs.any():
        return Verdict("TSR-05", NA, None, limit, "FAIL_SAFE never entered")
    t_fs = int(df.t_ms[fs].iloc[0])
    late = df[(df.t_ms >= t_fs + limit) & (df.brake_req_mps2 > 0)]
    if len(late):
        return Verdict("TSR-05", FAIL, float(int(late.t_ms.iloc[0]) - t_fs), limit, "brake request still active in FAIL_SAFE")
    released = df[(df.t_ms >= t_fs) & (df.brake_req_mps2 <= 0)]
    lat = float(int(released.t_ms.iloc[0]) - t_fs) if len(released) else math.inf
    return Verdict("TSR-05", PASS if lat <= limit else FAIL, lat, limit, f"FAIL_SAFE at {t_fs} ms, brake request 0 after {lat:.0f} ms")


def check_tsr06(res: RunResult) -> Verdict:
    df = _ctrl(res.signals)
    p = reqs.params("TSR-06")
    intervene = df.driver_brake.astype(bool) | (df.steer_rate_dps.abs() > p["steer_rate_dps"])
    if not intervene.any():
        return Verdict("TSR-06", NA, None, p["release_ms"], "no driver intervention")
    t_i = int(df.t_ms[intervene].iloc[0])
    window = df[(df.t_ms >= t_i + p["release_ms"]) & (df.t_ms <= t_i + p["release_ms"] + 300)]
    still = window[window.brake_req_mps2 > 0]
    released = df[(df.t_ms >= t_i) & (df.brake_req_mps2 <= 0)]
    lat = float(int(released.t_ms.iloc[0]) - t_i) if len(released) else math.inf
    ok = not len(still)
    return Verdict("TSR-06", PASS if ok else FAIL, lat, p["release_ms"], f"intervention at {t_i} ms, released after {lat:.0f} ms")


def check_tsr07(res: RunResult) -> Verdict:
    df, scn = res.signals, res.scenario
    limit = reqs.params("TSR-07")["react_ms"]
    failing = {f.target for f in scn.faults if f.type in ("bus_drop", "sensor_dropout")}
    if len(failing) != 1:
        return Verdict("TSR-07", NA, None, limit, "needs exactly one failed sensor")
    f = next(f for f in scn.faults if f.type in ("bus_drop", "sensor_dropout"))
    t_d = _first(df, (df.aeb_state == "DEGRADED") & df.fault_lamp.astype(bool), f.start_ms)
    lat = math.inf if t_d is None else t_d - f.start_ms
    return Verdict("TSR-07", PASS if lat <= limit else FAIL, lat, limit,
                   f"{f.target} failed at {f.start_ms} ms, DEGRADED {'never' if t_d is None else f'at {t_d} ms'}")


def check_tsr08(res: RunResult) -> Verdict:
    df, scn = res.signals, res.scenario
    p = reqs.params("TSR-08")
    faults = [f for f in scn.faults if f.type == "task_overrun"]
    if not faults:
        return Verdict("TSR-08", NA, None, None, "no task overrun injected")
    f = faults[0]
    period = f.params["period_ms"]
    t_fs = _first(df, df.aeb_state == "FAIL_SAFE", f.start_ms)
    if period <= p["period_ms"] * p["tolerance"]:
        return Verdict("TSR-08", PASS if t_fs is None else FAIL, period, p["period_ms"] * p["tolerance"],
                       f"period {period} ms within tolerance, FAIL_SAFE {'not entered' if t_fs is None else f'wrongly entered at {t_fs} ms'}")
    limit = (p["max_violations"] + 1) * period + 10
    lat = math.inf if t_fs is None else t_fs - f.start_ms
    return Verdict("TSR-08", PASS if lat <= limit else FAIL, lat, limit,
                   f"period {period} ms from {f.start_ms} ms, FAIL_SAFE {'never' if t_fs is None else f'at {t_fs} ms'}")


CHECKS = {
    "SG-01": check_sg01, "SG-03": check_sg03, "FSR-01": check_fsr01,
    "TSR-01": check_tsr01, "TSR-02": check_tsr02, "TSR-03": check_tsr03, "TSR-04": check_tsr04,
    "TSR-05": check_tsr05, "TSR-06": check_tsr06, "TSR-07": check_tsr07, "TSR-08": check_tsr08,
}
ALWAYS = ("SG-01", "TSR-02", "TSR-05")  # invariants checked on every run


def evaluate(res: RunResult) -> list[Verdict]:
    ids = list(dict.fromkeys([*res.scenario.requirements, *ALWAYS]))
    return [CHECKS[i](res) for i in ids if i in CHECKS]


def overall(verdicts: list[Verdict], scn: Scenario) -> str:
    """Run verdict: FAIL if any rule fails, PASS if every targeted rule passed."""
    if any(v.verdict == FAIL for v in verdicts):
        return FAIL
    targeted = [v for v in verdicts if v.req_id in scn.requirements]
    if targeted and all(v.verdict == NA for v in targeted):
        return NA
    return PASS
