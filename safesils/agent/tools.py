"""Deterministic tools the agent calls. None of them uses an LLM.

Planning schema and validation, scenario selection, execution through the SILS
and the rule judge, evidence extraction from the signal log, rule-based cause
notes, judge margins and scenario mutation for edge-case search.
"""
from __future__ import annotations

import copy
import math
import re
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Literal

import numpy as np
from pydantic import BaseModel, Field

from .. import generator
from .. import requirements as reqs
from ..judge import CHECKS, FAIL, NA, Verdict, evaluate, overall
from ..scenario import FAULT_TYPES, SENSORS, Scenario
from ..sim import ControllerUnderTest, RunResult, run
from ..store import ResultStore

SWEEPABLE = ("TSR-02", "TSR-03", "TSR-06", "TSR-08")
# rules whose measured value is a reaction time against a limit: the margin is meaningful
LATENCY_RULES = ("SG-01", "TSR-01", "TSR-04", "TSR-05", "TSR-06", "TSR-07", "TSR-08")
MECHANISMS = {
    "SG-01": "brake output path (monitor gate, plausibility, FAIL_SAFE gate)",
    "SG-03": "AEB braking and driver warning (FCW, fault lamp)",
    "FSR-01": "radar/camera object confirmation before braking",
    "TSR-01": "E2E check (CRC, alive counter)",
    "TSR-02": "message timeout monitor",
    "TSR-03": "radar/camera plausibility check",
    "TSR-04": "independent brake monitor",
    "TSR-05": "FAIL_SAFE output gate",
    "TSR-06": "driver override detection",
    "TSR-07": "single-sensor DEGRADED mode",
    "TSR-08": "control task watchdog",
}
BOUNDS = {"duration_s": (1.0, 20.0), "ego_speed_kph": (0.0, 130.0), "gap_m": (2.0, 200.0),
          "lead_speed_kph": (0.0, 130.0)}
FLAGS = ("e2e_radar", "e2e_camera", "timeout_radar", "timeout_camera", "mismatch", "monitor_block",
         "override", "fcw_warning", "fault_lamp")


class Plan(BaseModel):
    """What the planner (LLM or rules) decided to run."""

    summary: str = ""
    requirements: list[str] = Field(default_factory=list)
    strategy: Literal["baseline", "sweep", "combo", "custom"] = "baseline"
    n: int = Field(5, ge=1, le=50)
    seed: int = Field(0, ge=0)
    scenarios: list[dict] = Field(default_factory=list)
    explore_rounds: int = Field(0, ge=0, le=5)
    explore_per_round: int = Field(4, ge=1, le=10)


@dataclass
class RunRecord:
    scenario: Scenario
    run_id: str
    origin: str
    verdicts: list[Verdict]
    status: str
    result: RunResult = field(repr=False)

    @property
    def failures(self) -> list[Verdict]:
        return [v for v in self.verdicts if v.verdict == FAIL]

    @property
    def margin(self) -> float | None:
        ms = [m for m in (verdict_margin(v) for v in self.verdicts) if m is not None]
        return min(ms) if ms else None

    @property
    def min_gap_m(self) -> float:
        r = self.result.signals.true_range_m
        return 0.0 if self.result.collided else float(r.min()) if len(r) and r.notna().any() else math.inf


# -- catalogues for prompts ---------------------------------------------------------------

def requirement_catalog() -> str:
    doc = reqs.load()
    lines = []
    for req_id in CHECKS:
        r = doc["index"][req_id]
        crit = f" | criterion: {r['criterion']}" if "criterion" in r else ""
        lines.append(f"{req_id} (ASIL {r.get('asil', '?')}): {r['text']}{crit}")
    return "\n".join(lines)


def fault_catalog() -> str:
    lines = []
    for name, (needs_target, defaults) in FAULT_TYPES.items():
        tgt = f"target one of {list(SENSORS)}" if needs_target else "no target"
        lines.append(f"{name}: {tgt}, params {defaults or '{}'}")
    return "\n".join(lines)


# -- validation and selection ---------------------------------------------------------------

def known_requirements(ids: list[str]) -> tuple[list[str], list[str]]:
    """(checkable ids, notes about ids the judge cannot check)."""
    ok, notes = [], []
    for r in ids:
        r = r.strip().upper()
        if r in CHECKS:
            ok.append(r)
        else:
            notes.append(f"{r}: no rule-based check, ignored")
    return list(dict.fromkeys(ok)), notes


def validate_scenario(d: dict, tc_id: str) -> Scenario:
    """Build a Scenario from untrusted JSON (LLM output) and enforce physical bounds."""
    d = copy.deepcopy(d)
    d["tc_id"] = tc_id
    d.setdefault("description", "")
    d["requirements"], _ = known_requirements(list(d.get("requirements") or []))
    if not d["requirements"]:
        raise ValueError("scenario names no checkable requirement")
    allowed = set(Scenario.__dataclass_fields__)
    unknown = set(d) - allowed
    if unknown:
        raise ValueError(f"unknown scenario fields {sorted(unknown)}")
    scn = Scenario.from_dict(d)
    for owner, values in (("scenario", {k: getattr(scn, k) for k in ("duration_s", "seed", "ego_speed_kph")}),
                          ("lead", vars(scn.lead)), ("driver", vars(scn.driver))):
        for k, v in values.items():
            if v is not None and (not isinstance(v, (int, float)) or not math.isfinite(v)):
                raise ValueError(f"{owner}.{k}={v!r} is not a finite number")

    def within(name: str, value: float | None) -> None:
        lo, hi = BOUNDS[name]
        if value is None or not (lo <= float(value) <= hi):
            raise ValueError(f"{name}={value} outside [{lo}, {hi}]")

    within("duration_s", scn.duration_s)
    within("ego_speed_kph", scn.ego_speed_kph)
    if scn.lead.present:
        within("gap_m", scn.lead.gap_m)
        within("lead_speed_kph", scn.lead.speed_kph)
    for f in scn.faults:
        if not all(isinstance(x, (int, float)) for x in (f.start_s, f.end_s if f.end_s is not None else 0.0)):
            raise ValueError(f"fault {f.type} window must be numbers")
        if not 0.0 <= f.start_s < scn.duration_s or (f.end_s is not None and f.end_s <= f.start_s):
            raise ValueError(f"fault {f.type} window [{f.start_s}, {f.end_s}] invalid")
        defaults = FAULT_TYPES[f.type][1]
        extra = set(f.params) - set(defaults)
        if extra:
            raise ValueError(f"fault {f.type} has unknown params {sorted(extra)}")
        for k, v in f.params.items():
            if not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"fault {f.type} param {k}={v!r} is not a finite number")
        if f.type == "task_overrun" and not 1 <= f.params["period_ms"] <= 200:
            raise ValueError("task_overrun period_ms outside [1, 200]")
        if f.type == "bus_delay" and not 0 <= f.params["delay_ms"] <= 2000:
            raise ValueError("bus_delay delay_ms outside [0, 2000]")
    return scn


def select(plan: Plan) -> tuple[list[Scenario], list[str]]:
    """Scenarios for a plan, plus notes about anything that could not be honoured."""
    notes: list[str] = []
    wanted = set(plan.requirements)

    def touches(s: Scenario) -> bool:
        return not wanted or bool(wanted & set(s.requirements))

    if plan.strategy == "custom":
        out = []
        for i, d in enumerate(plan.scenarios):
            try:
                scn = validate_scenario(d, f"TC-AGT-000-{i:02d}")
                scn.exploratory = True
                out.append(scn)
            except (ValueError, TypeError) as e:
                notes.append(f"proposed scenario {i} rejected: {e}")
        if out:
            return out, notes
        notes.append("no valid custom scenario, falling back to the baseline set")
    if plan.strategy == "sweep":
        out = []
        for r in plan.requirements:
            if r in SWEEPABLE:
                out += generator.sweep(r, plan.n)
        if out:
            return out, notes
        notes.append(f"no boundary sweep defined for {sorted(wanted) or 'all'}; sweeps exist for {list(SWEEPABLE)}")
    if plan.strategy == "combo":
        pool = generator.combinations(max(plan.n * 4, 20), plan.seed)
        out = [s for s in pool if touches(s)][:plan.n]
        if out:
            return out, notes
        notes.append("no fault combination touches the requested requirements")
    out = [s for s in generator.baseline() if touches(s)]
    if not out:
        notes.append("no baseline TC for the requested requirements, running all")
        out = generator.baseline()
    return out, notes


def rule_plan(command: str) -> Plan:
    """Keyword planner used without an LLM or when the LLM's plan is unusable."""
    text = command.upper()
    ids = [f"{a}-{b}" for a, b in re.findall(r"\b(SG|FSR|TSR)-?(\d{2})\b", text)]
    req_ids, _ = known_requirements(ids)
    low = command.lower()
    n_match = re.search(r"(\d+)\s*(개|cases|scenarios|tc)", low)
    n = min(int(n_match.group(1)), 50) if n_match else 5
    explore = any(k in low for k in ("탐색", "엣지", "edge", "explore", "취약"))
    if any(k in low for k in ("경계", "boundary", "sweep", "스윕")) and any(r in SWEEPABLE for r in req_ids):
        strategy = "sweep"
    elif any(k in low for k in ("복합", "조합", "combo", "combination", "동시")):
        strategy = "combo"
    else:
        strategy = "baseline"
    return Plan(summary=f"rule-based plan: {strategy} for {req_ids or 'all requirements'}",
                requirements=req_ids, strategy=strategy, n=n, explore_rounds=2 if explore else 0)


# -- execution ------------------------------------------------------------------------------

def execute(scenarios: list[Scenario], make_controller: Callable[[], ControllerUnderTest | None],
            store: ResultStore, session_id: str, origin: str) -> tuple[list[RunRecord], list[str]]:
    """Run and judge each scenario; a scenario that crashes the simulation is reported, not fatal."""
    out, errors = [], []
    for scn in scenarios:
        try:
            res = run(scn, make_controller())
        except Exception as e:  # noqa: BLE001 - one bad scenario must not stop the session
            errors.append(f"{scn.tc_id} could not be simulated: {e!r}")
            continue
        verdicts = evaluate(res)
        run_id = store.save(res, verdicts)
        store.link_run(session_id, run_id, origin)
        out.append(RunRecord(scn, run_id, origin, verdicts, overall(verdicts, scn), res))
    return out, errors


def verdict_margin(v: Verdict) -> float | None:
    """Relative slack of a reaction-time rule: 1 = instant, 0 = at the limit, < 0 = violated."""
    if v.req_id not in LATENCY_RULES or v.verdict == NA or v.measured is None or not v.limit:
        return None
    m = (v.limit - v.measured) / v.limit
    return max(m, -1.0) if math.isfinite(m) else -1.0


# -- log analysis ---------------------------------------------------------------------------

def timeline(rec: RunRecord, max_events: int = 40) -> list[str]:
    """Key events from the 1 ms signal log: faults, state changes, safety flags, braking."""
    df = rec.result.signals
    ctrl = df[df.ctrl_tick]
    events: list[tuple[int, str]] = []
    for f in rec.scenario.faults:
        tgt = f"({f.target})" if f.target else ""
        events.append((f.start_ms, f"fault {f.type}{tgt} {f.params or ''} starts".replace(" {}", "")))
        if f.end_s is not None:
            events.append((int(round(f.end_s * 1000)), f"fault {f.type}{tgt} ends"))
    state = ctrl.aeb_state.to_numpy()
    t = ctrl.t_ms.to_numpy()
    for i in range(1, len(state)):
        if state[i] != state[i - 1]:
            events.append((int(t[i]), f"state {state[i - 1]} -> {state[i]}"))
    for name in FLAGS:
        v = ctrl[name].astype(bool).to_numpy()
        for i in range(1, len(v)):
            if v[i] != v[i - 1]:
                events.append((int(t[i]), f"{name} {'set' if v[i] else 'cleared'}"))
    brake = ctrl.brake_req_mps2.to_numpy()
    on = brake > 0
    for i in range(1, len(on)):
        if on[i] and not on[i - 1]:
            j = i
            while j < len(on) and on[j]:
                j += 1
            events.append((int(t[i]), f"brake request on, peak {brake[i:j].max():.1f} m/s^2 until {int(t[j - 1])} ms"))
    if rec.result.collided:
        events.append((int(df.t_ms.iloc[-1]), f"COLLISION at {rec.result.impact_speed_mps * 3.6:.1f} km/h relative"))
    events.sort(key=lambda e: e[0])
    lines = [f"t={tm} ms: {txt}" for tm, txt in events]
    if len(lines) > max_events:
        lines = lines[:max_events] + [f"... {len(lines) - max_events} more events"]
    return lines


def evidence(rec: RunRecord) -> str:
    scn = rec.scenario
    head = [f"{rec.run_id} {scn.tc_id}: {scn.description}",
            f"ego {scn.ego_speed_kph} km/h, lead {'none' if not scn.lead.present else f'{scn.lead.gap_m} m ahead at {scn.lead.speed_kph} km/h'}",
            f"overall {rec.status}; min gap {rec.min_gap_m:.1f} m"]
    verdicts = [f"{v.req_id} {v.verdict}: {v.detail} (measured {v.measured}, limit {v.limit})" for v in rec.verdicts]
    ctrl = rec.result.signals[rec.result.signals.ctrl_tick]
    never = [f for f in FLAGS if not ctrl[f].astype(bool).any()]
    states = list(dict.fromkeys(ctrl.aeb_state))
    summary = [f"states visited: {', '.join(states)}", f"flags never set: {', '.join(never) or '-'}"]
    return "\n".join(head + ["verdicts (rule judge):"] + verdicts + summary + ["timeline:"] + timeline(rec))


def rule_cause(v: Verdict) -> str:
    mech = MECHANISMS.get(v.req_id, "safety mechanism")
    if v.req_id in LATENCY_RULES and v.measured is not None and v.limit:
        if not math.isfinite(v.measured):
            return f"{mech} never reacted ({v.detail}); check whether it is triggered at all for this fault."
        return (f"{mech} reacted after {v.measured:.0f} ms but {v.req_id} allows {v.limit:.0f} ms; "
                f"check its threshold or debounce setting.")
    return f"{mech} did not meet {v.req_id}: {v.detail}."


# -- edge-case search -------------------------------------------------------------------------

def mutate(scn: Scenario, rng: np.random.Generator, k: int, prefix: str) -> list[Scenario]:
    """Up to k random neighbours of a scenario: timing, fault parameters and traffic jittered."""
    out = []
    for i in range(k):
        d = scn.to_dict()
        changes = []
        d["ego_speed_kph"] = round(float(np.clip(d["ego_speed_kph"] * rng.uniform(0.9, 1.1), 0, 130)), 1)
        changes.append(f"ego {d['ego_speed_kph']} km/h")
        if d["lead"]["present"]:
            d["lead"]["gap_m"] = round(float(np.clip(d["lead"]["gap_m"] * rng.uniform(0.8, 1.2), 2, 200)), 1)
            changes.append(f"gap {d['lead']['gap_m']} m")
        for f in d["faults"]:
            dur = None if f["end_s"] is None else f["end_s"] - f["start_s"]
            f["start_s"] = round(float(np.clip(f["start_s"] + rng.uniform(-0.3, 0.3), 0.5, d["duration_s"] - 0.5)), 2)
            if dur is not None:
                f["end_s"] = round(f["start_s"] + max(0.02, dur * rng.uniform(0.7, 1.3)), 2)
            for key, val in f["params"].items():
                new = val * rng.uniform(0.85, 1.15)
                f["params"][key] = int(round(new)) if isinstance(val, int) else round(float(new), 2)
            changes.append(f"{f['type']} {f['params'] or ''}@{f['start_s']}s".replace(" @", "@"))
        d["seed"] = int(rng.integers(0, 2**31))
        d["exploratory"] = True
        d["description"] = f"variant of {scn.tc_id}: " + ", ".join(changes)
        try:
            out.append(validate_scenario(d, f"{prefix}-{i:02d}"))
        except (ValueError, TypeError):
            continue
    return out
