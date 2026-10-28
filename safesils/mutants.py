"""Controllers with one deliberately weakened safety mechanism.

Used by the mutation tests (the judge must catch each defect) and by the agent
demo (the agent must find the failing requirement and explain the cause).
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from .controller import AEBController, Params


class _NoMonitor(AEBController):
    """The independent brake monitor never intervenes."""

    def _monitor(self, t_ms: int, brake_cmd: float) -> tuple[float, bool]:
        return brake_cmd, False


@dataclass(frozen=True)
class Mutant:
    name: str
    req_id: str          # requirement the defect violates
    tc_id: str           # baseline TC that exposes it
    description: str
    make: Callable[[], AEBController]


MUTANTS = {m.name: m for m in [
    Mutant("slow-timeout", "TSR-02", "TC-TSR02-001", "message timeout threshold 400 ms instead of 150 ms",
           lambda: AEBController(Params(timeout_ms=400))),
    Mutant("no-plausibility", "TSR-03", "TC-TSR03-002", "radar/camera disagreement threshold effectively disabled",
           lambda: AEBController(Params(mismatch_abs_m=1000.0, mismatch_rel=10.0))),
    Mutant("slow-monitor", "TSR-04", "TC-TSR04-001", "brake monitor debounce 500 ms instead of 50 ms",
           lambda: AEBController(Params(monitor_debounce_ms=500))),
    Mutant("deaf-override", "TSR-06", "TC-TSR06-001", "steering intervention threshold 500 deg/s instead of 100 deg/s",
           lambda: AEBController(Params(steer_rate_dps=500.0))),
    Mutant("no-degraded", "TSR-07", "TC-TSR07-001", "single sensor failure never leads to DEGRADED",
           lambda: AEBController(Params(mismatch_degraded_ms=10**6, timeout_ms=10**6))),
    Mutant("lax-watchdog", "TSR-08", "TC-TSR08-001", "watchdog tolerates 1000 period violations",
           lambda: AEBController(Params(wd_max_violations=1000))),
    Mutant("no-monitor", "TSR-04", "TC-TSR04-001", "independent brake monitor removed", _NoMonitor),
]}
