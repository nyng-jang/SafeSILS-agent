"""Traceability, selection set and the verification report."""
from __future__ import annotations

import csv
import json
import shutil
import subprocess
from pathlib import Path

import pytest

from safesils import generator, report, trace
from safesils.judge import evaluate
from safesils.mutants import MUTANTS
from safesils.requirements import DEFAULT_PATH
from safesils.sim import run
from safesils.store import ResultStore

BASELINE_DIR = Path(__file__).resolve().parent.parent / "scenarios" / "baseline"


def _run_all(db: Path, make=lambda: None, controller="python-reference", only=None) -> None:
    store = ResultStore(db)
    for s in generator.baseline():
        if only is None or s.tc_id in only:
            res = run(s, make())
            store.save(res, evaluate(res), keep_signals=False, controller=controller)


@pytest.fixture(scope="module")
def reference_db(tmp_path_factory) -> Path:
    db = tmp_path_factory.mktemp("ref") / "r.db"
    _run_all(db)
    return db


def test_reference_report_meets_exit_criteria(reference_db: Path, tmp_path: Path) -> None:
    ctx = report.context(reference_db, [BASELINE_DIR])
    assert ctx["exit_ok"] and not ctx["problems"] and not ctx["not_run"]
    assert {n.id for n in ctx["covered"]} == {n.id for n in ctx["checkable"]}
    assert all(n.rollup == "PASS" for n in ctx["nodes"].values())
    files = report.write(ctx, tmp_path, html=True)
    md = files["md"].read_text(encoding="utf-8")
    for section in ("Verification Measure", "Selection Set", "Verification Result", "Consistency Evidence", "SUP.9"):
        assert section in md
    rows = list(csv.DictReader(files["csv"].open(encoding="utf-8-sig")))
    assert {"TSR-02", "TC-TSR02-001", "PASS"} <= {rows[i][k] for i in range(len(rows)) for k in ("req_id", "tc_id", "verdict")}
    assert json.loads(files["json"].read_text(encoding="utf-8"))["exit_criteria_met"] is True
    assert "<table>" in files["html"].read_text(encoding="utf-8")


def test_defect_becomes_problem_and_rolls_up(tmp_path: Path) -> None:
    m = MUTANTS["slow-timeout"]
    db = tmp_path / "m.db"
    _run_all(db, m.make, f"mutant:{m.name}", only={"TC-TSR02-001", "TC-TSR02-002", "TC-SG03-001"})
    ctx = report.context(db, [BASELINE_DIR])
    assert not ctx["exit_ok"]
    tsr02 = [p for p in ctx["problems"] if p.req_id == "TSR-02"]
    assert tsr02 and tsr02[0].asil == "D" and "timeout monitor" in tsr02[0].cause
    nodes = ctx["nodes"]
    assert nodes["TSR-02"].status == "FAIL"
    assert nodes["FSR-02"].rollup == nodes["SG-01"].rollup == nodes["HZ-01"].rollup == "FAIL"
    assert nodes["TSR-08"].status == trace.NOT_RUN
    assert "TC-TSR08-001" in ctx["not_run"]
    assert "PRB-001" in report.render_markdown(ctx)


def test_controller_filter_separates_reference_and_mutant(tmp_path: Path) -> None:
    db = tmp_path / "mix.db"
    _run_all(db, only={"TC-TSR02-001"})
    _run_all(db, MUTANTS["slow-timeout"].make, "mutant:slow-timeout", only={"TC-TSR02-001"})
    assert trace.load_runs(db)[0].controller == "mutant:slow-timeout"  # latest wins
    assert trace.load_runs(db, controller="python-reference")[0].verdict == "PASS"


def test_selection_set_follows_the_requirement_tree() -> None:
    planned = trace.load_planned([BASELINE_DIR])
    assert set(trace.selection_set(planned, ["TSR-02"])) == {"TC-TSR02-001", "TC-TSR02-002"}
    fsr02 = trace.selection_set(planned, ["FSR-02"])
    assert set(fsr02) == {"TC-TSR01-001", "TC-TSR01-002", "TC-TSR02-001", "TC-TSR02-002"}
    assert "below FSR-02" in fsr02["TC-TSR02-001"][0]
    assert len(trace.selection_set(planned, ["SG-01"])) > len(fsr02)


@pytest.mark.skipif(not shutil.which("git"), reason="needs git")
def test_changed_requirements_from_git(tmp_path: Path) -> None:
    yml = tmp_path / "aeb_safety.yaml"
    shutil.copy(DEFAULT_PATH, yml)
    git = ["git", "-c", "user.name=t", "-c", "user.email=t@t", "-c", "commit.gpgsign=false"]
    subprocess.run(["git", "init", "-q"], cwd=tmp_path, check=True)
    subprocess.run(git + ["add", "."], cwd=tmp_path, check=True)
    subprocess.run(git + ["commit", "-qm", "base"], cwd=tmp_path, check=True)
    yml.write_text(yml.read_text(encoding="utf-8").replace("params: {timeout_ms: 150}", "params: {timeout_ms: 120}"),
                   encoding="utf-8")
    assert trace.changed_requirements("HEAD", yml) == ["TSR-02"]


def test_agent_llm_hypothesis_reaches_the_problem_record(tmp_path: Path) -> None:
    pytest.importorskip("langgraph")
    from safesils.agent import ScriptedLLM, VerificationAgent

    plan = json.dumps({"requirements": ["TSR-02"], "strategy": "baseline"})
    hyp = json.dumps({"mechanism": "timeout monitor", "hypothesis": "threshold near 400 ms", "next_check": "sweep"})
    db = tmp_path / "a.db"
    agent = VerificationAgent(ScriptedLLM([plan, hyp, hyp]), ResultStore(db), MUTANTS["slow-timeout"].make,
                              tmp_path / "out", controller_tag="mutant:slow-timeout")
    agent.run("TSR-02 검증")
    ctx = report.context(db, [BASELINE_DIR], session=agent.session_id)
    assert ctx["sessions"] == [agent.session_id]
    assert any(p.hypothesis and p.hypothesis["hypothesis"] == "threshold near 400 ms" for p in ctx["problems"])


def test_pdf(reference_db: Path, tmp_path: Path) -> None:
    pytest.importorskip("weasyprint")
    files = report.write(report.context(reference_db, [BASELINE_DIR]), tmp_path, pdf=True)
    assert files["pdf"].read_bytes()[:4] == b"%PDF"
