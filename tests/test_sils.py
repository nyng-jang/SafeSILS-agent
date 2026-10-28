"""Regression and mutation tests.

The baseline scenarios must pass on the reference controller. Each mutation
weakens one safety mechanism; the judge has to catch it, which shows the
verification harness itself can detect the defect it is meant to find.
"""
from __future__ import annotations

import pytest

from safesils import generator
from safesils.controller import AEBController, Params
from safesils.judge import FAIL, PASS, evaluate, overall
from safesils.scenario import Fault, Lead, Scenario
from safesils.sim import run

BASELINE = {s.tc_id: s for s in generator.baseline()}


@pytest.mark.parametrize("tc_id", sorted(BASELINE))
def test_baseline_passes(tc_id: str) -> None:
    scn = BASELINE[tc_id]
    verdicts = evaluate(run(scn))
    assert overall(verdicts, scn) == PASS, [v for v in verdicts if v.verdict == FAIL]


def test_nominal_aeb_avoids_collision() -> None:
    res = run(BASELINE["TC-SG03-001"])
    assert not res.collided
    assert res.signals.aeb_state.isin(["FULL_BRAKE"]).any()


def test_free_road_never_brakes() -> None:
    res = run(BASELINE["TC-SG01-001"])
    assert res.signals.brake_req_mps2.max() == 0.0


def test_counter_wrap_after_long_loss_recovers() -> None:
    """Exploratory combinations showed long corruption windows wrap the 4-bit alive
    counter; the receiver must resynchronise and leave timeout instead of
    rejecting every frame as a repeat."""
    scn = Scenario("TC-REG-001", ["TSR-02"], ego_speed_kph=50, lead=Lead(gap_m=60, speed_kph=20),
                   faults=[Fault("bus_corrupt", 2.0, 2.75, "radar")])  # exactly 15 frames
    df = run(scn).signals
    recovered = df[(df.t_ms > 2900) & ~df.timeout_radar.astype(bool)]
    assert len(recovered), "radar never recovered after the corruption window"


MUTANTS = [
    ("TC-TSR02-001", "TSR-02", Params(timeout_ms=400)),
    ("TC-TSR03-002", "TSR-03", Params(mismatch_abs_m=1000.0, mismatch_rel=10.0)),
    ("TC-TSR04-001", "TSR-04", Params(monitor_debounce_ms=500)),
    ("TC-TSR06-001", "TSR-06", Params(steer_rate_dps=500.0)),
    ("TC-TSR07-001", "TSR-07", Params(mismatch_degraded_ms=10**6, timeout_ms=10**6)),
    ("TC-TSR08-001", "TSR-08", Params(wd_max_violations=1000)),
]


@pytest.mark.parametrize("tc_id,req_id,params", MUTANTS, ids=[m[1] for m in MUTANTS])
def test_judge_catches_weakened_mechanism(tc_id: str, req_id: str, params: Params) -> None:
    res = run(BASELINE[tc_id], AEBController(params))
    verdict = {v.req_id: v.verdict for v in evaluate(res)}
    assert verdict[req_id] == FAIL


def test_judge_catches_missing_monitor() -> None:
    """Without the monitor gate the corrupted brake command reaches the actuator (SG-01)."""

    class NoMonitor(AEBController):
        def _monitor(self, t_ms: int, brake_cmd: float) -> tuple[float, bool]:
            return brake_cmd, False

    verdict = {v.req_id: v.verdict for v in evaluate(run(BASELINE["TC-TSR04-001"], NoMonitor()))}
    assert verdict["SG-01"] == FAIL and verdict["TSR-04"] == FAIL
