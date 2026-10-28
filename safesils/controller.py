"""AEB controller software under test, with its safety mechanisms.

Safety mechanism -> requirement:
  E2E check (CRC, alive counter) ........ TSR-01 / FSR-02
  Message timeout ......................... TSR-02 / FSR-02
  Radar/camera plausibility ............... TSR-03 / FSR-01
  Independent brake monitor ............... TSR-04 / FSR-03
  FAIL_SAFE output gate ................... TSR-05 / SG-01
  Driver override ......................... TSR-06 / FSR-04
  Single-sensor DEGRADED mode ............. TSR-07 / FSR-05
  Task watchdog ........................... TSR-08 / FSR-03

The main function and the monitor deliberately use separate computations so
that a fault in one is caught by the other.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .bus import COUNTER_MOD, Message, crc8

KPH = 1 / 3.6


@dataclass
class Params:
    period_ms: int = 10
    timeout_ms: int = 150
    e2e_max_delta: int = 2
    e2e_inhibit_ms: int = 150
    mismatch_abs_m: float = 3.0
    mismatch_rel: float = 0.15
    mismatch_debounce_ms: int = 100
    mismatch_degraded_ms: int = 1000
    monitor_debounce_ms: int = 50
    monitor_ttc_s: float = 1.9
    wd_tolerance: float = 1.5
    wd_max_violations: int = 3
    steer_rate_dps: float = 100.0
    speed_min_kph: float = 10.0
    speed_max_kph: float = 80.0
    ttc_warn_s: float = 2.6
    ttc_partial_s: float = 1.6
    ttc_full_s: float = 0.9
    partial_mps2: float = 4.0
    full_mps2: float = 9.0


@dataclass
class Track:
    """Receiver side of one E2E-protected channel."""

    name: str
    params: Params
    data: dict | None = None
    last_counter: int | None = None
    last_valid_ms: int = 0
    last_e2e_error_ms: int = -10**9
    e2e_error_now: bool = False

    def process(self, msgs: list[Message], t_ms: int) -> None:
        self.e2e_error_now = False
        for m in msgs:
            if m.crc != crc8(m.payload, m.counter):
                self._error(t_ms)
                continue
            if self.last_counter is None:  # first frame after start-up
                self.last_counter = m.counter
                self._accept(m, t_ms)
                continue
            delta = (m.counter - self.last_counter) % COUNTER_MOD
            resync = self.timeout(t_ms)
            self.last_counter = m.counter
            if 1 <= delta <= self.params.e2e_max_delta:
                self._accept(m, t_ms)
            elif resync and delta != 0:
                # After a timeout the 4-bit counter may have wrapped, so this frame
                # only becomes the new reference; the next in-sequence frame is accepted.
                continue
            else:
                self._error(t_ms)

    def _accept(self, m: Message, t_ms: int) -> None:
        self.data = m.payload
        self.last_valid_ms = t_ms

    def _error(self, t_ms: int) -> None:
        self.e2e_error_now = True
        self.last_e2e_error_ms = t_ms

    def timeout(self, t_ms: int) -> bool:
        return t_ms - self.last_valid_ms > self.params.timeout_ms

    def healthy(self, t_ms: int) -> bool:
        return not self.timeout(t_ms) and self.data is not None and bool(self.data["status_ok"])

    def e2e_inhibit(self, t_ms: int) -> bool:
        return t_ms - self.last_e2e_error_ms < self.params.e2e_inhibit_ms


def _ttc(range_m: float, rel_vel: float) -> float:
    return range_m / -rel_vel if rel_vel < -0.1 else math.inf


@dataclass
class AEBController:
    params: Params = field(default_factory=Params)

    def __post_init__(self) -> None:
        p = self.params
        self.radar = Track("radar", p)
        self.camera = Track("camera", p)
        self.state = "MONITOR"
        self.fail_safe = False
        self.override = False
        self.mismatch_since: int | None = None
        self.monitor_bad_since: int | None = None
        self.monitor_legit = False
        self.last_call_ms: int | None = None
        self.wd_violations = 0

    # -- main function -------------------------------------------------------
    def step(self, t_ms: int, radar_msgs: list[Message], camera_msgs: list[Message],
             ego_speed_mps: float, driver_brake: bool, steer_rate_dps: float,
             brake_cmd_fault: float | None = None) -> dict:
        p = self.params
        self._watchdog(t_ms)
        self.radar.process(radar_msgs, t_ms)
        self.camera.process(camera_msgs, t_ms)

        r_ok, c_ok = self.radar.healthy(t_ms), self.camera.healthy(t_ms)
        r = self.radar.data if r_ok else None
        c = self.camera.data if c_ok else None

        # TSR-03 plausibility: both see an object but disagree on range
        disagree = bool(r and c and r["obj"] and c["obj"]
                        and abs(r["range_m"] - c["range_m"]) > max(p.mismatch_abs_m, p.mismatch_rel * r["range_m"]))
        if disagree:
            self.mismatch_since = t_ms if self.mismatch_since is None else self.mismatch_since
        else:
            self.mismatch_since = None
        mismatch_age = t_ms - self.mismatch_since if self.mismatch_since is not None else -1
        mismatch = mismatch_age >= p.mismatch_debounce_ms

        confirmed = bool(r and c and r["obj"] and c["obj"] and not disagree)
        ttc = _ttc(r["range_m"], r["rel_vel_mps"]) if confirmed else math.inf
        single = r if (r and not c) else c if (c and not r) else None
        ttc_single = _ttc(single["range_m"], single["rel_vel_mps"]) if single and single["obj"] else math.inf

        # TSR-06 driver override, latched until the threat is gone
        if driver_brake or abs(steer_rate_dps) > p.steer_rate_dps:
            self.override = True
        elif ttc > p.ttc_warn_s:
            self.override = False

        started = t_ms >= p.timeout_ms  # every sensor has had a chance to report
        if (started and not r_ok and not c_ok) or self.wd_violations >= p.wd_max_violations:
            self.fail_safe = True
        degraded = started and ((r_ok != c_ok) or mismatch_age >= p.mismatch_degraded_ms)
        inhibit = (mismatch or self.override or self.radar.e2e_inhibit(t_ms) or self.camera.e2e_inhibit(t_ms))
        in_range = p.speed_min_kph * KPH <= ego_speed_mps <= p.speed_max_kph * KPH
        braking = self.state in ("PARTIAL_BRAKE", "FULL_BRAKE")

        if self.fail_safe:
            state = "FAIL_SAFE"
        elif degraded:
            state = "DEGRADED"
        elif braking and not inhibit and confirmed and ttc < math.inf and ego_speed_mps > 0.3:
            state = "FULL_BRAKE" if (self.state == "FULL_BRAKE" or ttc <= p.ttc_full_s) else "PARTIAL_BRAKE"
        elif not in_range:
            state = "STANDBY"
        elif not inhibit and ttc <= p.ttc_full_s:
            state = "FULL_BRAKE"
        elif not inhibit and ttc <= p.ttc_partial_s:
            state = "PARTIAL_BRAKE"
        elif ttc <= p.ttc_warn_s:
            state = "WARNING"
        else:
            state = "MONITOR"
        self.state = state

        brake_cmd = {"PARTIAL_BRAKE": p.partial_mps2, "FULL_BRAKE": p.full_mps2}.get(state, 0.0)
        if brake_cmd_fault is not None:
            brake_cmd = brake_cmd_fault  # injected corruption of the internal variable

        brake_out, monitor_block = self._monitor(t_ms, brake_cmd)
        if self.fail_safe:  # TSR-05 output gate
            brake_out = 0.0
            self.state = "FAIL_SAFE"

        fcw = state in ("WARNING", "PARTIAL_BRAKE", "FULL_BRAKE") or (state == "DEGRADED" and ttc_single <= p.ttc_warn_s)
        return {
            "aeb_state": self.state,
            "brake_req_mps2": brake_out,
            "brake_cmd_internal": brake_cmd,
            "fcw_warning": bool(fcw and not self.fail_safe),
            "fault_lamp": bool(self.fail_safe or degraded),
            "ttc_s": ttc,
            "confirmed": confirmed,
            "override": self.override,
            "e2e_radar": self.radar.e2e_error_now,
            "e2e_camera": self.camera.e2e_error_now,
            "timeout_radar": self.radar.timeout(t_ms),
            "timeout_camera": self.camera.timeout(t_ms),
            "last_valid_radar": self.radar.last_valid_ms,
            "last_valid_camera": self.camera.last_valid_ms,
            "mismatch": mismatch,
            "monitor_block": monitor_block,
            "wd_violations": self.wd_violations,
            "radar_range_rx": r["range_m"] if r and r["obj"] else math.nan,
            "camera_range_rx": c["range_m"] if c and c["obj"] else math.nan,
        }

    # -- independent monitor (TSR-04) ------------------------------------------
    def _monitor(self, t_ms: int, brake_cmd: float) -> tuple[float, bool]:
        """Recompute from raw track data whether braking is justified; gate the output."""
        p = self.params
        r, c = self.radar.data, self.camera.data
        fresh = not self.radar.timeout(t_ms) and not self.camera.timeout(t_ms)
        both = bool(fresh and r and c and r["obj"] and c["obj"] and r["status_ok"] and c["status_ok"])
        closing = both and r["rel_vel_mps"] < -0.1 and c["rel_vel_mps"] < -0.1
        rng = min(r["range_m"], c["range_m"]) if both else math.inf
        ttc_mon = rng / -r["rel_vel_mps"] if closing else math.inf
        if not closing or self.override:
            self.monitor_legit = False
        elif ttc_mon <= p.monitor_ttc_s:
            self.monitor_legit = True
        allowed = p.full_mps2 if self.monitor_legit else 0.0

        bad = brake_cmd > allowed + 0.1
        if bad:
            self.monitor_bad_since = t_ms if self.monitor_bad_since is None else self.monitor_bad_since
            if t_ms - self.monitor_bad_since >= p.monitor_debounce_ms:
                self.fail_safe = True
            return 0.0, True
        self.monitor_bad_since = None
        return brake_cmd, False

    # -- watchdog (TSR-08) ---------------------------------------------------------
    def _watchdog(self, t_ms: int) -> None:
        p = self.params
        if self.last_call_ms is not None:
            if t_ms - self.last_call_ms > p.period_ms * p.wd_tolerance:
                self.wd_violations += 1
            else:
                self.wd_violations = 0
        self.last_call_ms = t_ms
