"""Command line entry point: generate scenarios, run them, summarise results.

  safesils generate baseline --out scenarios/baseline
  safesils generate sweep --req TSR-02 --n 7 --out scenarios/sweep
  safesils generate combo --n 20 --seed 1 --out scenarios/exploratory
  safesils run scenarios/baseline --db out/results.db
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


def cmd_run(a: argparse.Namespace) -> int:
    store = ResultStore(a.db)
    failed = 0
    for f in _scenario_files(a.paths):
        scn = Scenario.load(f)
        res = run(scn)
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
    a = ap.parse_args(argv)
    if a.cmd == "generate":
        cmd_generate(a)
        return 0
    return cmd_run(a)


if __name__ == "__main__":
    raise SystemExit(main())
