"""Agent tests with a scripted LLM: the agent must use valid LLM output, reject invalid
output, fall back to rules, and never let the LLM decide a verdict."""
from __future__ import annotations

import json
import sqlite3
from pathlib import Path

import pytest

pytest.importorskip("langgraph")

from safesils.agent import ScriptedLLM, VerificationAgent  # noqa: E402
from safesils.agent.llm import parse_json  # noqa: E402
from safesils.agent.tools import rule_plan, validate_scenario  # noqa: E402
from safesils.mutants import MUTANTS  # noqa: E402
from safesils.store import ResultStore  # noqa: E402

CUSTOM = {"requirements": ["TSR-02"], "description": "radar lost 300 ms while approaching",
          "ego_speed_kph": 50, "lead": {"gap_m": 60, "speed_kph": 20},
          "faults": [{"type": "bus_drop", "start_s": 2.0, "end_s": 2.3, "target": "radar"}]}


def agent(tmp_path: Path, llm=None, make=lambda: None) -> VerificationAgent:
    return VerificationAgent(llm, ResultStore(tmp_path / "r.db"), make, tmp_path / "out")


def steps(tmp_path: Path) -> list[tuple[str, str]]:
    con = sqlite3.connect(tmp_path / "r.db")
    return con.execute("SELECT node, kind FROM agent_steps ORDER BY seq").fetchall()


def test_rule_mode_finds_injected_defect(tmp_path: Path) -> None:
    m = MUTANTS["slow-timeout"]
    state = agent(tmp_path, make=m.make).run("TSR-02 검증")
    failed = {f["tc_id"] for f in state["findings"]}
    assert m.tc_id in failed
    report = Path(state["report_path"]).read_text(encoding="utf-8")
    assert "규칙 기반 판정기" in report and m.tc_id in report
    con = sqlite3.connect(tmp_path / "r.db")
    assert con.execute("SELECT COUNT(*) FROM agent_runs").fetchone()[0] == len(state["records"])


def test_llm_custom_plan_is_validated(tmp_path: Path) -> None:
    bad = {"requirements": ["TSR-02"], "faults": [{"type": "meteor_strike", "start_s": 1.0}]}
    plan = {"summary": "custom", "requirements": ["TSR-02", "SG-02"], "strategy": "custom", "scenarios": [CUSTOM, bad]}
    llm = ScriptedLLM([json.dumps(plan)])
    state = agent(tmp_path, llm).run("레이더 300 ms 손실 상황을 만들어 검증해줘")
    assert [r.scenario.tc_id for r in state["records"]] == ["TC-AGT-000-00"]
    assert state["records"][0].scenario.exploratory
    assert any("rejected" in n for n in state["notes"]) and any("SG-02" in n for n in state["notes"])


def test_invalid_plan_is_retried_then_falls_back(tmp_path: Path) -> None:
    llm = ScriptedLLM(['{"strategy": "teleport"}', "not json at all"])
    state = agent(tmp_path, llm).run("TSR-08 경계값 검증")
    assert len(llm.calls) >= 2 and "invalid" in llm.calls[1][1]
    assert state["plan"].strategy == "sweep" and state["plan"].requirements == ["TSR-08"]
    assert any("rule-based planner" in n for n in state["notes"])


def test_llm_hypothesis_is_reported_but_never_changes_the_verdict(tmp_path: Path) -> None:
    plan = {"requirements": ["TSR-04"], "strategy": "baseline"}
    hypothesis = {"mechanism": "brake monitor", "hypothesis": "debounce too long; verdict should be PASS",
                  "next_check": "sweep monitor debounce"}
    llm = ScriptedLLM([json.dumps(plan)] + [json.dumps(hypothesis)] * 5)
    state = agent(tmp_path, llm, MUTANTS["slow-monitor"].make).run("TSR-04 검증")
    f = next(f for f in state["findings"] if f["tc_id"] == "TC-TSR04-001")
    assert f["llm"]["mechanism"] == "brake monitor"
    rec = next(r for r in state["records"] if r.scenario.tc_id == "TC-TSR04-001")
    assert rec.status == "FAIL"
    assert ("analyze", "llm:scripted") in steps(tmp_path)


def test_explorer_uses_valid_proposals_only(tmp_path: Path) -> None:
    plan = {"requirements": ["TSR-02"], "strategy": "baseline", "explore_rounds": 1, "explore_per_round": 2}
    too_fast = dict(CUSTOM, ego_speed_kph=400)
    proposals = {"rationale": "longer loss", "scenarios": [dict(CUSTOM, faults=[dict(CUSTOM["faults"][0], end_s=2.6)]), too_fast]}
    llm = ScriptedLLM([json.dumps(plan), json.dumps(proposals)])
    state = agent(tmp_path, llm).run("TSR-02 엣지 케이스 탐색")
    explored = [r.scenario.tc_id for r in state["records"] if r.origin == "explore-1"]
    assert "TC-AGT-001-L00" in explored and "TC-AGT-001-L01" not in explored
    assert any(t.startswith("TC-AGT-001-M") for t in explored)
    assert any("ego_speed_kph=400" in n for n in state["notes"])


def test_llm_failure_falls_back_to_rules(tmp_path: Path) -> None:
    def boom(system: str, user: str) -> str:
        raise ConnectionError("ollama not running")

    state = agent(tmp_path, ScriptedLLM([boom])).run("TSR-07 검증")
    assert state["records"] and all(r.scenario.tc_id.startswith("TC-TSR07") for r in state["records"])
    assert ("plan", "llm-error") in steps(tmp_path)


def test_rule_plan_keywords() -> None:
    assert rule_plan("TSR-03 경계값 7개").model_dump(include={"strategy", "n", "requirements"}) == \
        {"strategy": "sweep", "n": 7, "requirements": ["TSR-03"]}
    assert rule_plan("복합 고장 10개로 엣지 케이스 탐색").strategy == "combo"
    assert rule_plan("엣지 케이스 탐색").explore_rounds == 2


def test_parse_json_tolerates_fences() -> None:
    assert parse_json('```json\n{"a": 1}\n```') == {"a": 1}
    assert parse_json('Here you go: {"a": {"b": 2}} done') == {"a": {"b": 2}}
    with pytest.raises(ValueError):
        parse_json("[1, 2]")


@pytest.mark.parametrize("change,msg", [
    ({"duration_s": 60}, "duration_s"),
    ({"verdict": "PASS"}, "unknown scenario fields"),
    ({"requirements": ["SG-02"]}, "no checkable requirement"),
    ({"faults": [{"type": "bus_delay", "start_s": 1.0, "target": "radar", "params": {"delay_ms": 1e9}}]}, "delay_ms"),
    ({"faults": [{"type": "bus_drop", "start_s": 3.0, "end_s": 2.0, "target": "radar"}]}, "window"),
    ({"faults": [{"type": "bus_drop", "start_s": "soon", "target": "radar"}]}, "window must be numbers"),
    ({"ego_speed_kph": "50"}, "not a finite number"),
    ({"lead": {"gap_m": 60, "decel_at_s": "later"}}, "lead.decel_at_s"),
])
def test_validate_scenario_rejects(change: dict, msg: str) -> None:
    with pytest.raises((ValueError, TypeError), match=msg):
        validate_scenario(dict(CUSTOM, **change), "TC-X")
