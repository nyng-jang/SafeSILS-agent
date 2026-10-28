"""LangGraph workflow: plan -> execute -> analyze -> (explore -> execute -> analyze)* -> report.

The LLM proposes (plans, scenarios, cause hypotheses); deterministic tools do the
rest. Every LLM reply is validated before it is used, and an unusable reply falls
back to the rule-based path, so the agent also runs with no LLM at all. Every
LLM call and tool call is written to the result store as an audit trail.
"""
from __future__ import annotations

import json
import math
from collections.abc import Callable
from pathlib import Path
from typing import TypedDict

import numpy as np
from langgraph.graph import END, START, StateGraph
from pydantic import ValidationError

from ..judge import FAIL
from ..scenario import Scenario
from ..sim import ControllerUnderTest
from ..store import ResultStore
from . import prompts, tools
from .llm import LLM, parse_json
from .tools import Plan, RunRecord

MAX_LLM_ANALYSES = 10
MAX_PROPOSALS = 10


class AgentState(TypedDict, total=False):
    command: str
    plan: Plan
    pending: list[Scenario]
    records: list[RunRecord]
    findings: list[dict]
    round: int
    notes: list[str]
    report_path: str


class VerificationAgent:
    def __init__(self, llm: LLM | None, store: ResultStore,
                 make_controller: Callable[[], ControllerUnderTest | None] = lambda: None,
                 out_dir: str | Path = "out/agent", controller_name: str = "Python reference controller") -> None:
        self.llm = llm
        self.store = store
        self.make_controller = make_controller
        self.out_dir = Path(out_dir)
        self.controller_name = controller_name
        self.session_id = ""
        self._analyses = 0
        g = StateGraph(AgentState)
        for name in ("plan", "execute", "analyze", "explore", "report"):
            g.add_node(name, getattr(self, f"_{name}"))
        g.add_edge(START, "plan")
        g.add_edge("plan", "execute")
        g.add_edge("execute", "analyze")
        g.add_conditional_edges("analyze", self._next, {"explore": "explore", "report": "report"})
        g.add_edge("explore", "execute")
        g.add_edge("report", END)
        self.graph = g.compile()

    def run(self, command: str) -> AgentState:
        self.session_id = self.store.start_session(command, self.llm.name if self.llm else "none")
        self._analyses = 0
        state = self.graph.invoke({"command": command, "records": [], "findings": [], "round": 0, "notes": []},
                                  {"recursion_limit": 50})
        self.store.finish_session(self.session_id, state["plan"].model_dump_json(), state["report_path"])
        return state

    # -- helpers ---------------------------------------------------------------------------------

    def _ask(self, node: str, system: str, user: str) -> dict | None:
        """One LLM call, logged; None if there is no LLM or the reply is not JSON."""
        if self.llm is None:
            return None
        try:
            reply = self.llm.complete(system, user)
        except Exception as e:  # network, model missing, ...
            self.store.log_step(self.session_id, node, "llm-error", user, repr(e))
            return None
        self.store.log_step(self.session_id, node, f"llm:{self.llm.name}", user, reply)
        try:
            return parse_json(reply)
        except (ValueError, json.JSONDecodeError):
            return None

    def _tool(self, node: str, name: str, input: object, output: object) -> None:
        self.store.log_step(self.session_id, node, f"tool:{name}", json.dumps(input, ensure_ascii=False, default=str),
                            json.dumps(output, ensure_ascii=False, default=str))

    # -- nodes -----------------------------------------------------------------------------------

    def _plan(self, state: AgentState) -> dict:
        notes: list[str] = []
        plan = None
        user = state["command"]
        for attempt in range(2):
            raw = self._ask("plan", prompts.planner_system(), user)
            if raw is None:
                break
            try:
                plan = Plan.model_validate(raw)
                plan.requirements, dropped = tools.known_requirements(plan.requirements)
                notes += dropped
                break
            except ValidationError as e:
                user = f"{state['command']}\n\nYour previous plan was invalid: {e.errors()[:3]}. Reply with a corrected plan."
        if plan is None:
            if self.llm is not None:
                notes.append("LLM plan unusable, used the rule-based planner")
            plan = tools.rule_plan(state["command"])
        scenarios, sel_notes = tools.select(plan)
        notes += sel_notes
        self._tool("plan", "select", plan.model_dump(), [s.tc_id for s in scenarios])
        return {"plan": plan, "pending": scenarios, "notes": state["notes"] + notes}

    def _execute(self, state: AgentState) -> dict:
        origin = "plan" if state["round"] == 0 else f"explore-{state['round']}"
        recs, errors = tools.execute(state["pending"], self.make_controller, self.store, self.session_id, origin)
        self._tool("execute", "run", [s.tc_id for s in state["pending"]], [(r.run_id, r.status) for r in recs] + errors)
        return {"records": state["records"] + recs, "pending": [], "notes": state["notes"] + errors}

    def _analyze(self, state: AgentState) -> dict:
        done = {f["run_id"] for f in state["findings"]}
        findings = list(state["findings"])
        for rec in state["records"]:
            if rec.status != FAIL or rec.run_id in done:
                continue
            ev = tools.evidence(rec)
            finding = {"run_id": rec.run_id, "tc_id": rec.scenario.tc_id, "origin": rec.origin, "evidence": ev,
                       "failed": [f"{v.req_id}: {v.detail}" for v in rec.failures],
                       "rule_cause": [tools.rule_cause(v) for v in rec.failures]}
            if self._analyses < MAX_LLM_ANALYSES:
                raw = self._ask("analyze", prompts.analyst_system(), ev)
                if raw and isinstance(raw.get("hypothesis"), str):
                    self._analyses += 1
                    finding["llm"] = {k: str(raw.get(k, "")) for k in ("mechanism", "hypothesis", "next_check")}
            findings.append(finding)
        return {"findings": findings}

    def _next(self, state: AgentState) -> str:
        return "explore" if state["round"] < state["plan"].explore_rounds else "report"

    def _explore(self, state: AgentState) -> dict:
        rnd = state["round"] + 1
        plan = state["plan"]
        ranked = sorted((r for r in state["records"] if r.margin is not None or r.scenario.lead.present),
                        key=lambda r: (r.margin if r.margin is not None else 1.0, r.min_gap_m))
        seeds = ranked[:3] or state["records"][:3]
        rng = np.random.default_rng(plan.seed + 1000 * rnd)
        prefix = f"TC-AGT-{rnd:03d}"
        candidates: list[Scenario] = []
        per_seed = max(1, math.ceil(plan.explore_per_round / max(1, len(seeds))))
        for j, rec in enumerate(seeds):
            candidates += tools.mutate(rec.scenario, rng, per_seed, f"{prefix}-M{j}")
        notes = []
        if self.llm is not None and seeds:
            brief = [{"run_id": r.run_id, "margin": r.margin, "min_gap_m": round(r.min_gap_m, 2),
                      "verdicts": [f"{v.req_id} {v.verdict}: {v.detail}" for v in r.verdicts],
                      "scenario": r.scenario.to_dict()} for r in seeds]
            raw = self._ask("explore", prompts.explorer_system(), json.dumps(brief, ensure_ascii=False))
            proposals = raw.get("scenarios", []) if raw else []
            for i, d in enumerate(proposals[:MAX_PROPOSALS] if isinstance(proposals, list) else []):
                try:
                    scn = tools.validate_scenario(d, f"{prefix}-L{i:02d}")
                    scn.exploratory = True
                    candidates.append(scn)
                except (ValueError, TypeError) as e:
                    notes.append(f"round {rnd}: proposed scenario {i} rejected: {e}")
        self._tool("explore", "mutate", [r.run_id for r in seeds], [s.tc_id for s in candidates])
        return {"round": rnd, "pending": candidates, "notes": state["notes"] + notes}

    def _report(self, state: AgentState) -> dict:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        path = self.out_dir / f"{self.session_id}.md"
        path.write_text(render_report(self.session_id, state, self.llm.name if self.llm else "none",
                                      self.controller_name), encoding="utf-8")
        return {"report_path": str(path)}


def _fmt_margin(m: float | None) -> str:
    return "-" if m is None else f"{m:+.2f}"


def render_report(session_id: str, state: AgentState, llm_name: str, controller: str) -> str:
    plan, recs = state["plan"], state["records"]
    n_fail = sum(r.status == FAIL for r in recs)
    lines = [
        f"# 검증 에이전트 보고서 {session_id}",
        "",
        f"- 명령: {state['command']}",
        f"- 검증 대상: {controller}",
        f"- LLM: {llm_name}",
        f"- 계획: {plan.summary or plan.strategy} (전략 {plan.strategy}, 요구사항 {', '.join(plan.requirements) or '전체'}, "
        f"탐색 {plan.explore_rounds}회)",
        f"- 실행 {len(recs)}건, FAIL {n_fail}건",
        "",
        "> PASS/FAIL은 규칙 기반 판정기(`safesils/judge.py`)가 내린 결과다. 원인 분석은 가설이며 엔지니어 확인이 필요하다.",
        "",
        "## 실행 결과",
        "",
        "| RUN | TC | 출처 | 판정 | 최소 여유 | 최소 차간 (m) | 실패 요구사항 |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for r in recs:
        gap = "충돌" if r.result.collided else f"{r.min_gap_m:.1f}" if math.isfinite(r.min_gap_m) else "-"
        lines.append(f"| {r.run_id} | {r.scenario.tc_id} | {r.origin} | {r.status} | {_fmt_margin(r.margin)} | {gap} | "
                     f"{', '.join(v.req_id for v in r.failures) or '-'} |")
    if state["findings"]:
        lines += ["", "## 실패 분석", ""]
        for f in state["findings"]:
            lines += [f"### {f['run_id']} {f['tc_id']}", "", "**실패 판정**", ""]
            lines += [f"- {x}" for x in f["failed"]]
            lines += ["", "**규칙 기반 원인 메모**", ""] + [f"- {x}" for x in f["rule_cause"]]
            if "llm" in f:
                h = f["llm"]
                lines += ["", f"**LLM 원인 가설** ({llm_name})", "", f"- 메커니즘: {h['mechanism']}",
                          f"- 가설: {h['hypothesis']}", f"- 확인 방법: {h['next_check']}"]
            lines += ["", "<details><summary>근거(로그 타임라인)</summary>", "", "```", f["evidence"], "```", "", "</details>", ""]
    explored = [r for r in recs if r.origin.startswith("explore")]
    if explored:
        best = sorted(explored, key=lambda r: (r.margin if r.margin is not None else 1.0, r.min_gap_m))[:5]
        lines += ["", "## 엣지 케이스 탐색", "",
                  f"탐색 시나리오 {len(explored)}건, 그중 FAIL {sum(r.status == FAIL for r in explored)}건. 여유가 가장 작은 경우:", ""]
        lines += [f"- {r.scenario.tc_id} ({r.status}, 여유 {_fmt_margin(r.margin)}): {r.scenario.description}" for r in best]
    if state["notes"]:
        lines += ["", "## 참고", ""] + [f"- {n}" for n in state["notes"]]
    return "\n".join(lines) + "\n"
