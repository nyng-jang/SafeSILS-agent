"""Regression and mutation tests.

The baseline scenarios must pass on the reference controller. Each mutation
weakens one safety mechanism; the judge has to catch it, which shows the
verification harness itself can detect the defect it is meant to find.
"""
from __future__ import annotations

import pytest

from safesils import generator
from safesils.mutants import MUTANTS
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


@pytest.mark.parametrize("name", sorted(MUTANTS))
def test_judge_catches_weakened_mechanism(name: str) -> None:
    m = MUTANTS[name]
    res = run(BASELINE[m.tc_id], m.make())
    verdict = {v.req_id: v.verdict for v in evaluate(res)}
    assert verdict[m.req_id] == FAIL


def test_missing_monitor_also_violates_safety_goal() -> None:
    """Without the monitor gate the corrupted brake command reaches the actuator (SG-01)."""
    verdict = {v.req_id: v.verdict for v in evaluate(run(BASELINE["TC-TSR04-001"], MUTANTS["no-monitor"].make()))}
    assert verdict["SG-01"] == FAIL
