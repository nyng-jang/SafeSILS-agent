"""Bidirectional traceability HZ -> SG -> FSR -> TSR -> TC -> RUN, coverage and
regression selection (A-SPICE SYS.5/SWE.6 consistency evidence and selection set).

Sources: the requirement catalogue (requirements/aeb_safety.yaml), the planned
test cases (scenario JSON files) and the result database (runs, verdicts).
"""
from __future__ import annotations

import json
import sqlite3
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from . import requirements as reqs
from .judge import CHECKS, FAIL, NA, PASS, Verdict
from .scenario import Scenario

NOT_RUN, NOT_COVERED = "NOT RUN", "NOT COVERED"
_RANK = {FAIL: 4, NOT_COVERED: 3, NOT_RUN: 2, NA: 1, PASS: 0}
LEVELS = (("HZ", "hazards"), ("SG", "safety_goals"), ("FSR", "functional_safety_requirements"),
          ("TSR", "technical_safety_requirements"))


@dataclass
class RunRow:
    run_id: str
    tc_id: str
    started_at: str
    verdict: str
    exploratory: bool
    controller: str
    sw_version: str
    scenario: Scenario
    verdicts: dict[str, Verdict]  # every rule the judge evaluated on this run


@dataclass
class ReqNode:
    id: str
    level: str
    asil: str
    text: str
    criterion: str
    parent: str | None
    children: list[str] = field(default_factory=list)
    planned_tcs: list[str] = field(default_factory=list)    # planned TCs that target it
    runs: list[RunRow] = field(default_factory=list)        # latest runs of TCs that target it
    status: str = NOT_COVERED                               # own evidence
    rollup: str = NOT_COVERED                               # own and all descendants

    @property
    def checkable(self) -> bool:
        return self.id in CHECKS


def _worst(statuses: list[str]) -> str:
    return max(statuses, key=_RANK.__getitem__) if statuses else NOT_COVERED


def load_runs(db: str | Path, session: str | None = None, controller: str | None = None) -> list[RunRow]:
    """Latest run of each TC in the database, optionally of one agent session or one controller."""
    con = sqlite3.connect(db)
    cols = {r[1] for r in con.execute("PRAGMA table_info(runs)")}
    extra = ", controller, sw_version" if "controller" in cols else ", NULL, NULL"
    conds, args = [], []
    if session:
        conds.append("run_id IN (SELECT run_id FROM agent_runs WHERE session_id = ?)")
        args.append(session)
    if controller:
        conds.append("controller = ?")
        args.append(controller)
    where = f"WHERE {' AND '.join(conds)}" if conds else ""
    rows = con.execute(f"SELECT run_id, tc_id, started_at, verdict, exploratory, scenario_json{extra} FROM runs "
                       f"{where} ORDER BY run_id", args).fetchall()
    latest: dict[str, tuple] = {}
    for r in rows:
        latest[r[1]] = r  # run ids sort chronologically, so the last one wins
    verdicts: dict[str, dict[str, Verdict]] = {}
    for run_id, req_id, verdict, measured, limit, detail in con.execute(
            "SELECT run_id, req_id, verdict, measured, limit_value, detail FROM verdicts"):
        verdicts.setdefault(run_id, {})[req_id] = Verdict(req_id, verdict, measured, limit, detail or "")
    out = []
    for run_id, tc_id, started, verdict, expl, scn_json, ctrl, sw in latest.values():
        out.append(RunRow(run_id, tc_id, started, verdict, bool(expl), ctrl or "unknown", sw or "unknown",
                          Scenario.from_dict(json.loads(scn_json)), verdicts.get(run_id, {})))
    return sorted(out, key=lambda r: r.tc_id)


def load_planned(paths: list[str | Path]) -> list[Scenario]:
    files: list[Path] = []
    for p in map(Path, paths):
        files += sorted(p.glob("*.json")) if p.is_dir() else [p]
    return [Scenario.load(f) for f in files]


def build(planned: list[Scenario], runs: list[RunRow]) -> dict[str, ReqNode]:
    doc = reqs.load()
    nodes: dict[str, ReqNode] = {}
    for prefix, key in LEVELS:
        for r in doc.get(key, []):
            parent = r.get("parent") if prefix != "HZ" else None
            nodes[r["id"]] = ReqNode(r["id"], prefix, str(r.get("asil", "")), r.get("text") or r.get("malfunction", ""),
                                     r.get("criterion", ""), parent)
    # hazards point down to their safety goal; requirements point up to their parent
    for r in doc.get("hazards", []):
        nodes[r["id"]].children.append(r["safety_goal"])
    for n in nodes.values():
        if n.parent:
            nodes[n.parent].children.append(n.id)
    for s in planned:
        for req_id in s.requirements:
            if req_id in nodes:
                nodes[req_id].planned_tcs.append(s.tc_id)
    for run in runs:
        for req_id in run.scenario.requirements:
            if req_id in nodes:
                nodes[req_id].runs.append(run)
    for n in nodes.values():
        if n.runs:
            n.status = _worst([r.verdicts[n.id].verdict if n.id in r.verdicts else NOT_RUN for r in n.runs])
        elif n.planned_tcs:
            n.status = NOT_RUN

    def rollup(req_id: str) -> str:
        n = nodes[req_id]
        own = [n.status] if (n.runs or n.planned_tcs) else []
        n.rollup = _worst(own + [rollup(c) for c in n.children])
        return n.rollup

    for n in nodes.values():
        if n.level == "HZ":
            rollup(n.id)
    for n in nodes.values():  # requirements not reachable from a hazard
        if n.rollup == NOT_COVERED and n.level != "HZ":
            rollup(n.id)
    return nodes


def descendants(nodes: dict[str, ReqNode], req_id: str) -> list[str]:
    out = []
    for c in nodes[req_id].children:
        out += [c, *descendants(nodes, c)]
    return out


def selection_set(planned: list[Scenario], changed: list[str]) -> dict[str, list[str]]:
    """TCs to re-run when requirements change: TCs targeting a changed requirement or one
    below it (A-SPICE Verification Measure Selection Set). Maps tc_id -> reasons."""
    nodes = build(planned, [])
    affected: dict[str, str] = {}
    for c in changed:
        if c not in nodes:
            continue
        for r in [c, *descendants(nodes, c)]:
            affected.setdefault(r, c)
    out: dict[str, list[str]] = {}
    for s in planned:
        hits = [f"{r}" + (f" (below {affected[r]})" if affected[r] != r else "") for r in s.requirements if r in affected]
        if hits:
            out[s.tc_id] = hits
    return out


def changed_requirements(base_ref: str, path: Path = reqs.DEFAULT_PATH) -> list[str]:
    """Requirement IDs whose YAML entry differs between git ref base_ref and the working tree."""
    repo = path.resolve().parent
    rel = subprocess.run(["git", "ls-files", "--full-name", str(path)], cwd=repo, capture_output=True,
                         text=True, check=True).stdout.strip()
    old_text = subprocess.run(["git", "show", f"{base_ref}:{rel}"], cwd=repo, capture_output=True, text=True)
    old = yaml.safe_load(old_text.stdout) if old_text.returncode == 0 else {}
    new = yaml.safe_load(path.read_text(encoding="utf-8"))

    def index(doc: dict) -> dict[str, dict]:
        return {r["id"]: r for _, key in LEVELS for r in (doc or {}).get(key, [])}

    a, b = index(old), index(new)
    return sorted(k for k in a.keys() | b.keys() if a.get(k) != b.get(k))
