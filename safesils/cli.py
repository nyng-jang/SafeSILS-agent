"""Command line entry point: generate scenarios, run them, summarise results.

  safesils generate baseline --out scenarios/baseline
  safesils generate sweep --req TSR-02 --n 7 --out scenarios/sweep
  safesils generate combo --n 20 --seed 1 --out scenarios/exploratory
  safesils run scenarios/baseline --db out/results.db
  safesils run scenarios/baseline --fmu matlab/AEB_Controller.fmu
  safesils b2b scenarios/baseline                     # open loop vs MATLAB code (Octave/MATLAB)
  safesils b2b scenarios/baseline --fmu matlab/AEB_Controller.fmu   # closed loop vs FMU
  safesils b2b scenarios/baseline --export matlab/b2b_vectors        # test vectors for b2b_simulink.m
"""
from __future__ import annotations

import argparse
from pathlib import Path

from . import generator
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
        run_id = store.save(res, verdicts, keep_signals=not a.no_signals)
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
    a = ap.parse_args(argv)
    if a.cmd == "generate":
        cmd_generate(a)
        return 0
    if a.cmd == "b2b":
        return cmd_b2b(a)
    return cmd_run(a)


if __name__ == "__main__":
    raise SystemExit(main())
