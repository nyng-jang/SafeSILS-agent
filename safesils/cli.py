"""Command line entry point: generate scenarios, run them, summarise results.

  safesils generate baseline --out scenarios/baseline
  safesils generate sweep --req TSR-02 --n 7 --out scenarios/sweep
  safesils generate combo --n 20 --seed 1 --out scenarios/exploratory
  safesils run scenarios/baseline --db out/results.db
  safesils run scenarios/baseline --fmu matlab/AEB_Controller.fmu
  safesils b2b scenarios/baseline                     # open loop vs MATLAB code (Octave/MATLAB)
  safesils b2b scenarios/baseline --fmu matlab/AEB_Controller.fmu   # closed loop vs FMU
  safesils b2b scenarios/baseline --export matlab/b2b_vectors        # test vectors for b2b_simulink.m
  safesils agent "TSR-02 경계값 검증하고 엣지 케이스 탐색해줘" --llm ollama:llama3.1
  safesils agent "전체 회귀 검증" --llm none --mutant slow-timeout     # demo: find an injected defect
  safesils report --db out/results.db --out out/report --pdf           # traceability + verification report
  safesils select --base-ref origin/main                                # TCs affected by requirement changes
"""
from __future__ import annotations

import argparse
from pathlib import Path

from . import generator
from .mutants import MUTANTS
from .judge import FAIL, evaluate, overall
from .scenario import Scenario
from .sim import run
from .store import ResultStore


def _scenario_files(paths: list[str]) -> list[Path]:
    files: list[Path] = []
    for p in map(Path, paths):
        files += sorted(p.glob("*.json")) if p.is_dir() else [p]
    return files


def cmd_generate(a: argparse.Namespace) -> None:
    if a.kind == "baseline":
        scns = generator.baseline()
    elif a.kind == "sweep":
        scns = generator.sweep(a.req, a.n)
    else:
        scns = generator.combinations(a.n, a.seed)
    for s in scns:
        s.save(Path(a.out) / f"{s.tc_id}.json")
    print(f"wrote {len(scns)} scenarios to {a.out}")


def _controller(fmu: str | None):
    if not fmu:
        return None
    from .external import FMUController
    return FMUController(fmu)


def cmd_run(a: argparse.Namespace) -> int:
    store = ResultStore(a.db)
    failed = 0
    for f in _scenario_files(a.paths):
        scn = Scenario.load(f)
        res = run(scn, _controller(a.fmu))
        verdicts = evaluate(res)
        run_id = store.save(res, verdicts, keep_signals=not a.no_signals,
                            controller=f"fmu:{Path(a.fmu).name}" if a.fmu else "python-reference")
        status = overall(verdicts, scn)
        failed += status == FAIL
        print(f"{run_id}  {scn.tc_id:<18} {status:<5} " +
              "  ".join(f"{v.req_id}={v.verdict}" for v in verdicts))
        if status == FAIL:
            for v in verdicts:
                if v.verdict == FAIL:
                    print(f"    {v.req_id}: {v.detail}")
    print(f"\n{failed} failed, results in {a.db}")
    return 1 if failed and a.strict else 0


def cmd_b2b(a: argparse.Namespace) -> int:
    from . import b2b
    if a.export:
        files = _scenario_files(a.paths)
        for f in files:
            b2b.export_vectors(Scenario.load(f), Path(a.export))
        print(f"wrote test vectors for {len(files)} scenarios to {a.export}")
        return 0
    differing = 0
    for f in _scenario_files(a.paths):
        scn = Scenario.load(f)
        if a.fmu:
            diffs = [str(d) for d in b2b.closed_loop(scn, _controller(a.fmu))]
        else:
            diffs = [f"t={d.t_ms} ms {d.signal}: python {d.python} vs matlab {d.matlab}" for d in b2b.open_loop(scn)]
        differing += bool(diffs)
        print(f"{scn.tc_id:<18} {'MATCH' if not diffs else f'{len(diffs)} differences'}")
        for d in diffs[:5]:
            print(f"    {d}")
    print(f"\n{differing} scenarios differ")
    return 1 if differing else 0


def cmd_agent(a: argparse.Namespace) -> int:
    from .agent import VerificationAgent, make_llm

    if a.mutant:
        m = MUTANTS[a.mutant]
        make, name, tag = m.make, f"defect-injected controller '{m.name}' ({m.description})", f"mutant:{m.name}"
    elif a.fmu:
        make, name, tag = (lambda: _controller(a.fmu)), f"FMU {a.fmu}", f"fmu:{Path(a.fmu).name}"
    else:
        make, name, tag = (lambda: None), "Python reference controller", "python-reference"
    agent = VerificationAgent(make_llm(a.llm), ResultStore(a.db), make, a.out, name, tag)
    state = agent.run(a.command)
    for note in state["notes"]:
        print(f"note: {note}")
    for r in state["records"]:
        print(f"{r.run_id}  {r.scenario.tc_id:<18} {r.origin:<10} {r.status:<5} " +
              "  ".join(f"{v.req_id}={v.verdict}" for v in r.verdicts))
    failed = sum(r.status == "FAIL" for r in state["records"])
    print(f"\n{len(state['records'])} runs, {failed} failed; report {state['report_path']}, session {agent.session_id} in {a.db}")
    return 0


def _changed(a: argparse.Namespace) -> list[str] | None:
    from .trace import changed_requirements

    ids = [c.strip().upper() for c in (a.changed or "").split(",") if c.strip()]
    if a.base_ref:
        ids += changed_requirements(a.base_ref)
    return sorted(set(ids)) if (a.changed or a.base_ref) else None


def cmd_report(a: argparse.Namespace) -> int:
    from . import report

    ctx = report.context(a.db, a.scenarios, _changed(a), a.session, a.controller)
    files = report.write(ctx, a.out, html=not a.no_html, pdf=a.pdf)
    for kind, path in files.items():
        print(f"{kind:<5} {path}")
    print(f"{len(ctx['runs'])} runs, {len(ctx['problems'])} problems, exit criteria {'met' if ctx['exit_ok'] else 'not met'}")
    return 1 if a.strict and not ctx["exit_ok"] else 0


def cmd_select(a: argparse.Namespace) -> int:
    from .trace import load_planned, selection_set

    changed = _changed(a) or []
    planned = load_planned(a.scenarios)
    sel = selection_set(planned, changed)
    print(f"changed requirements: {', '.join(changed) or 'none'}")
    for tc, why in sel.items():
        print(f"{tc:<18} {', '.join(why)}")
    if a.out:
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        for s in planned:
            if s.tc_id in sel:
                s.save(out / f"{s.tc_id}.json")
        print(f"wrote {len(sel)} scenarios to {out}")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="safesils")
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("generate", help="write scenario JSON files")
    g.add_argument("kind", choices=["baseline", "sweep", "combo"])
    g.add_argument("--req", default="TSR-02")
    g.add_argument("--n", type=int, default=5)
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("--out", required=True)
    r = sub.add_parser("run", help="run scenarios through SILS and judge them")
    r.add_argument("paths", nargs="+")
    r.add_argument("--db", default="out/results.db")
    r.add_argument("--no-signals", action="store_true", help="store verdicts only")
    r.add_argument("--strict", action="store_true", help="exit 1 if any run fails")
    r.add_argument("--fmu", help="controller under test as an FMU instead of the Python reference")
    b = sub.add_parser("b2b", help="back-to-back: Python reference vs MATLAB code or FMU")
    b.add_argument("paths", nargs="+")
    b.add_argument("--fmu", help="compare closed loop against this FMU; default: open loop against matlab/*.m")
    b.add_argument("--export", metavar="DIR", help="only write input/expected-output CSV vectors for MATLAB")
    ag = sub.add_parser("agent", help="LLM verification agent: natural-language command to runs, analysis and report")
    ag.add_argument("command")
    ag.add_argument("--llm", default="ollama:llama3.1", help="none | ollama:<model> | openai:<model>")
    ag.add_argument("--db", default="out/results.db")
    ag.add_argument("--out", default="out/agent", help="report directory")
    ag.add_argument("--fmu", help="verify this FMU controller instead of the Python reference")
    ag.add_argument("--mutant", choices=sorted(MUTANTS),
                    help="verify a controller with one injected defect (demo)")
    rp = sub.add_parser("report", help="traceability matrix and A-SPICE structured verification report")
    rp.add_argument("--db", default="out/results.db")
    rp.add_argument("--scenarios", nargs="+", default=["scenarios/baseline"], help="planned TCs")
    rp.add_argument("--out", default="out/report")
    rp.add_argument("--session", help="only runs of this agent session")
    rp.add_argument("--controller", help="only runs of this controller, e.g. python-reference")
    rp.add_argument("--pdf", action="store_true", help="also write PDF (pip install -e '.[pdf]')")
    rp.add_argument("--no-html", action="store_true")
    rp.add_argument("--strict", action="store_true", help="exit 1 if the exit criteria are not met")
    sp = sub.add_parser("select", help="regression selection set for changed requirements")
    sp.add_argument("--scenarios", nargs="+", default=["scenarios/baseline"])
    sp.add_argument("--out", help="copy the selected scenario files here")
    for p in (rp, sp):
        p.add_argument("--changed", help="comma-separated changed requirement IDs")
        p.add_argument("--base-ref", help="git ref to diff requirements/aeb_safety.yaml against")
    a = ap.parse_args(argv)
    if a.cmd == "generate":
        cmd_generate(a)
        return 0
    if a.cmd == "b2b":
        return cmd_b2b(a)
    if a.cmd == "agent":
        return cmd_agent(a)
    if a.cmd == "report":
        return cmd_report(a)
    if a.cmd == "select":
        return cmd_select(a)
    return cmd_run(a)


if __name__ == "__main__":
    raise SystemExit(main())
