"""Verification report, structured by the A-SPICE SYS.5/SWE.6 output information items:
verification measure, measure data, selection set, verification result, consistency
evidence (traceability), plus SUP.9 problem records for every failed rule.

Markdown is rendered with Jinja2; HTML (python-markdown) and PDF (WeasyPrint) are optional.
"""
from __future__ import annotations

import csv
import json
import platform
import sqlite3
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from jinja2 import Environment, PackageLoader, StrictUndefined

from . import trace
from .analysis import MECHANISMS, rule_cause
from .judge import ALWAYS, FAIL, NA, PASS
from .store import sw_version

SEVERITY = {"D": "A (ASIL D)", "C": "A (ASIL C)", "B": "B (ASIL B)", "A": "C (ASIL A)", "QM": "C (QM)"}


@dataclass
class Problem:
    id: str
    run_id: str
    tc_id: str
    req_id: str
    asil: str
    severity: str
    mechanism: str
    description: str
    cause: str
    hypothesis: dict | None
    exploratory: bool


def llm_hypotheses(db: str | Path) -> dict[str, dict]:
    """Cause hypotheses the agent's LLM wrote for failed runs, from the audit trail."""
    con = sqlite3.connect(db)
    if not con.execute("SELECT name FROM sqlite_master WHERE name = 'agent_steps'").fetchone():
        return {}
    out = {}
    for session, kind, inp, outp in con.execute(
            "SELECT session_id, kind, input, output FROM agent_steps WHERE node = 'analyze' AND kind LIKE 'llm:%'"):
        run_id = (inp or "").split(" ", 1)[0]
        text = outp or ""
        try:
            h = json.loads(text[text.find("{"):text.rfind("}") + 1])
        except (TypeError, json.JSONDecodeError):
            continue
        if isinstance(h, dict) and isinstance(h.get("hypothesis"), str):
            out[run_id] = {"session": session, "model": kind.removeprefix("llm:"),
                           **{k: str(h.get(k, "")) for k in ("mechanism", "hypothesis", "next_check")}}
    return out


def problems(runs: list[trace.RunRow], nodes: dict[str, trace.ReqNode], hypotheses: dict[str, dict]) -> list[Problem]:
    out = []
    for run in runs:
        for v in run.verdicts.values():
            if v.verdict != FAIL:
                continue
            asil = nodes[v.req_id].asil if v.req_id in nodes else ""
            out.append(Problem(f"PRB-{len(out) + 1:03d}", run.run_id, run.tc_id, v.req_id, asil,
                               SEVERITY.get(asil, "-"), MECHANISMS.get(v.req_id, "-"), v.detail, rule_cause(v),
                               hypotheses.get(run.run_id), run.exploratory))
    return out


def context(db: str | Path, planned_paths: list[str | Path], changed: list[str] | None = None,
            session: str | None = None, controller: str | None = None, title: str | None = None) -> dict:
    planned = trace.load_planned(planned_paths)
    runs = trace.load_runs(db, session, controller)
    nodes = trace.build(planned, runs)
    planned_ids = {s.tc_id for s in planned}
    run_ids = {r.tc_id for r in runs}
    checkable = [n for n in nodes.values() if n.checkable]
    invariant_fails = Counter(v.req_id for r in runs for v in r.verdicts.values()
                              if v.verdict == FAIL and v.req_id in ALWAYS and v.req_id not in r.scenario.requirements)
    con = sqlite3.connect(db)
    sessions = []
    if con.execute("SELECT name FROM sqlite_master WHERE name = 'agent_runs'").fetchone():
        ids = tuple(r.run_id for r in runs)
        if ids:
            q = ",".join("?" * len(ids))
            sessions = [s for (s,) in con.execute(
                f"SELECT DISTINCT session_id FROM agent_runs WHERE run_id IN ({q}) ORDER BY session_id", ids)]
    signal_rows = 0
    if con.execute("SELECT name FROM sqlite_master WHERE name = 'signals'").fetchone() and runs:
        q = ",".join("?" * len(runs))
        signal_rows = con.execute(f"SELECT COUNT(*) FROM signals WHERE run_id IN ({q})",
                                  [r.run_id for r in runs]).fetchone()[0]
    probs = problems(runs, nodes, llm_hypotheses(db))
    status_count = Counter(r.verdict for r in runs)
    exit_ok = (not probs and planned_ids <= run_ids
               and all(n.status in (PASS,) for n in checkable if n.planned_tcs))
    return {
        "title": title or "AEB 기능안전 SILS 검증 보고서",
        "generated": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC"),
        "db": str(db),
        "planned_paths": [str(p) for p in planned_paths],
        "sw_versions": sorted({r.sw_version for r in runs}) or [sw_version()],
        "controllers": sorted({r.controller for r in runs}),
        "python": platform.python_version(),
        "platform": platform.platform(terse=True),
        "session": session,
        "sessions": sessions,
        "runs": runs,
        "run_by_tc": {r.tc_id: r for r in runs},
        "planned_by_tc": {s.tc_id: s for s in planned},
        "not_run": sorted(planned_ids - run_ids),
        "extra_tcs": sorted(run_ids - planned_ids),
        "status_count": status_count,
        "nodes": nodes,
        "levels": [p for p, _ in trace.LEVELS],
        "checkable": checkable,
        "covered": [n for n in checkable if n.runs],
        "uncovered": [n for n in nodes.values() if n.level != "HZ" and n.rollup == trace.NOT_COVERED],
        "invariant_fails": invariant_fails,
        "signal_rows": signal_rows,
        "changed": changed,
        "selection": trace.selection_set(planned, changed) if changed else None,
        "problems": probs,
        "exit_ok": exit_ok,
        "always": ALWAYS,
        "PASS": PASS, "FAIL": FAIL, "NA": NA,
    }


def _env() -> Environment:
    env = Environment(loader=PackageLoader("safesils", "templates"), undefined=StrictUndefined,
                      trim_blocks=True, lstrip_blocks=True, keep_trailing_newline=True)
    env.filters["cell"] = lambda s: str(s).replace("|", "\\|").replace("\n", " ")
    env.filters["mark"] = lambda s: {"PASS": "✅ PASS", "FAIL": "❌ FAIL", "N/A": "➖ N/A",
                                     "NOT RUN": "⏸ NOT RUN", "NOT COVERED": "⚠️ NOT COVERED"}.get(s, s)
    return env


def render_markdown(ctx: dict) -> str:
    return _env().get_template("report.md.j2").render(**ctx)


def render_html(markdown_text: str, title: str) -> str:
    import markdown

    body = markdown.markdown(markdown_text, extensions=["tables", "fenced_code", "toc", "md_in_html"])
    return _env().get_template("report.html.j2").render(title=title, body=body)


def write_matrix_csv(ctx: dict, path: Path) -> None:
    """Flat traceability matrix, one row per (requirement, TC) pair, for spreadsheets."""
    nodes = ctx["nodes"]
    runs = {r.tc_id: r for r in ctx["runs"]}
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["level", "req_id", "asil", "parent", "text", "tc_id", "run_id", "verdict", "detail", "req_status"])
        for n in nodes.values():
            tcs = sorted(set(n.planned_tcs) | {r.tc_id for r in n.runs})
            for tc in tcs or [""]:
                run = runs.get(tc)
                v = run.verdicts.get(n.id) if run else None
                w.writerow([n.level, n.id, n.asil, n.parent or "", n.text, tc, run.run_id if run else "",
                            v.verdict if v else (trace.NOT_RUN if tc else ""), v.detail if v else "", n.rollup])


def write(ctx: dict, out_dir: str | Path, html: bool = True, pdf: bool = False) -> dict[str, Path]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    files = {"md": out / "verification_report.md", "csv": out / "traceability_matrix.csv",
             "json": out / "verification_summary.json"}
    text = render_markdown(ctx)
    files["md"].write_text(text, encoding="utf-8")
    write_matrix_csv(ctx, files["csv"])
    summary = {
        "generated": ctx["generated"], "sw_versions": ctx["sw_versions"], "controllers": ctx["controllers"],
        "runs": len(ctx["runs"]), "status": dict(ctx["status_count"]), "not_run": ctx["not_run"],
        "requirements": {n.id: {"status": n.status, "rollup": n.rollup} for n in ctx["nodes"].values()},
        "problems": [p.__dict__ for p in ctx["problems"]], "exit_criteria_met": ctx["exit_ok"],
    }
    files["json"].write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if html or pdf:
        page = render_html(text, ctx["title"])
        files["html"] = out / "verification_report.html"
        files["html"].write_text(page, encoding="utf-8")
        if pdf:
            from weasyprint import HTML

            files["pdf"] = out / "verification_report.pdf"
            HTML(string=page, base_url=str(out)).write_pdf(files["pdf"])
    return files
