"""SQLite result store: runs, verdicts, the full signal log of every run, and the
agent's audit trail (sessions, every LLM call and tool call, runs it started)."""
from __future__ import annotations

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

from .judge import Verdict, overall
from .sim import RunResult

SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    run_id TEXT PRIMARY KEY,
    tc_id TEXT NOT NULL,
    started_at TEXT NOT NULL,
    verdict TEXT NOT NULL,
    exploratory INTEGER NOT NULL,
    collided INTEGER NOT NULL,
    impact_speed_mps REAL,
    description TEXT,
    scenario_json TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS verdicts (
    run_id TEXT NOT NULL REFERENCES runs(run_id),
    tc_id TEXT NOT NULL,
    req_id TEXT NOT NULL,
    verdict TEXT NOT NULL,
    measured REAL,
    limit_value REAL,
    detail TEXT
);
CREATE TABLE IF NOT EXISTS agent_sessions (
    session_id TEXT PRIMARY KEY,
    started_at TEXT NOT NULL,
    command TEXT NOT NULL,
    llm TEXT NOT NULL,
    plan_json TEXT,
    report_path TEXT
);
CREATE TABLE IF NOT EXISTS agent_steps (
    session_id TEXT NOT NULL REFERENCES agent_sessions(session_id),
    seq INTEGER NOT NULL,
    node TEXT NOT NULL,
    kind TEXT NOT NULL,
    input TEXT,
    output TEXT,
    at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS agent_runs (
    session_id TEXT NOT NULL REFERENCES agent_sessions(session_id),
    run_id TEXT NOT NULL REFERENCES runs(run_id),
    origin TEXT NOT NULL
);
"""


class ResultStore:
    def __init__(self, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.con = sqlite3.connect(path)
        self.con.executescript(SCHEMA)

    def next_run_id(self) -> str:
        day = datetime.now(timezone.utc).strftime("%Y%m%d")
        n = self.con.execute("SELECT COUNT(*) FROM runs WHERE run_id LIKE ?", (f"RUN-{day}-%",)).fetchone()[0]
        return f"RUN-{day}-{n + 1:04d}"

    def save(self, res: RunResult, verdicts: list[Verdict], keep_signals: bool = True) -> str:
        scn = res.scenario
        run_id = self.next_run_id()
        self.con.execute(
            "INSERT INTO runs VALUES (?,?,?,?,?,?,?,?,?)",
            (run_id, scn.tc_id, datetime.now(timezone.utc).isoformat(timespec="seconds"), overall(verdicts, scn),
             int(scn.exploratory), int(res.collided), res.impact_speed_mps, scn.description,
             json.dumps(scn.to_dict(), ensure_ascii=False)),
        )
        self.con.executemany(
            "INSERT INTO verdicts VALUES (?,?,?,?,?,?,?)",
            [(run_id, scn.tc_id, v.req_id, v.verdict, _num(v.measured), _num(v.limit), v.detail) for v in verdicts],
        )
        if keep_signals:
            df = res.signals.copy()
            df.insert(0, "run_id", run_id)
            df.insert(1, "tc_id", scn.tc_id)
            df.to_sql("signals", self.con, if_exists="append", index=False)
        self.con.commit()
        return run_id

    # -- agent audit trail ---------------------------------------------------------------

    def start_session(self, command: str, llm: str) -> str:
        now = datetime.now(timezone.utc)
        prefix = f"AGT-{now:%Y%m%d}-"
        n = self.con.execute("SELECT COUNT(*) FROM agent_sessions WHERE session_id LIKE ?", (prefix + "%",)).fetchone()[0]
        session_id = f"{prefix}{n + 1:04d}"
        self.con.execute("INSERT INTO agent_sessions VALUES (?,?,?,?,?,?)",
                         (session_id, now.isoformat(timespec="seconds"), command, llm, None, None))
        self.con.commit()
        return session_id

    def log_step(self, session_id: str, node: str, kind: str, input: str, output: str) -> None:
        seq = self.con.execute("SELECT COUNT(*) FROM agent_steps WHERE session_id = ?", (session_id,)).fetchone()[0]
        self.con.execute("INSERT INTO agent_steps VALUES (?,?,?,?,?,?,?)",
                         (session_id, seq + 1, node, kind, input, output,
                          datetime.now(timezone.utc).isoformat(timespec="seconds")))
        self.con.commit()

    def link_run(self, session_id: str, run_id: str, origin: str) -> None:
        self.con.execute("INSERT INTO agent_runs VALUES (?,?,?)", (session_id, run_id, origin))
        self.con.commit()

    def finish_session(self, session_id: str, plan_json: str, report_path: str) -> None:
        self.con.execute("UPDATE agent_sessions SET plan_json = ?, report_path = ? WHERE session_id = ?",
                         (plan_json, report_path, session_id))
        self.con.commit()


def _num(x: float | None) -> float | None:
    if x is None or x != x or x in (float("inf"), float("-inf")):
        return None
    return float(x)
