"""Judge-result interpretation shared by the agent and the reports: which safety
mechanism a requirement exercises, reaction-time margins and rule-based cause notes."""
from __future__ import annotations

import math

from .judge import NA, Verdict

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


def verdict_margin(v: Verdict) -> float | None:
    """Relative slack of a reaction-time rule: 1 = instant, 0 = at the limit, < 0 = violated."""
    if v.req_id not in LATENCY_RULES or v.verdict == NA or v.measured is None or not v.limit:
        return None
    m = (v.limit - v.measured) / v.limit
    return max(m, -1.0) if math.isfinite(m) else -1.0


def rule_cause(v: Verdict) -> str:
    mech = MECHANISMS.get(v.req_id, "safety mechanism")
    if v.req_id in LATENCY_RULES and v.measured is not None and v.limit:
        if not math.isfinite(v.measured):
            return f"{mech} never reacted ({v.detail}); check whether it is triggered at all for this fault."
        return (f"{mech} reacted after {v.measured:.0f} ms but {v.req_id} allows {v.limit:.0f} ms; "
                f"check its threshold or debounce setting.")
    return f"{mech} did not meet {v.req_id}: {v.detail}."
