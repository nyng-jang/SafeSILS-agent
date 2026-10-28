function y = aeb_controller_step(reset, t_ms, radar_frames, camera_frames, ego_speed_mps, ...
                                 driver_brake, steer_rate_dps, fault_active, fault_value)
%AEB_CONTROLLER_STEP  AEB controller under test, one activation of the 10 ms task.
%
%   Mirrors safesils/controller.py (the Python reference) so the two can be
%   compared back-to-back. Code-generation compatible: used inside the
%   MATLAB Function block of AEB_Controller.slx and exported as an FMU.
%
%   Inputs
%     reset           true on the first call: clears all internal state
%     t_ms            activation time [ms] (the watchdog supervises its spacing)
%     *_frames        4x7 received CAN frames this activation, one row per slot:
%                     [present counter crc obj status_ok range_raw vel_raw]
%                     range_raw: 0.01 m (uint16), vel_raw: 0.01 m/s (int16)
%     ego_speed_mps   ego vehicle speed [m/s]
%     driver_brake    brake pedal pressed (0/1)
%     steer_rate_dps  steering wheel rate [deg/s]
%     fault_active    fault injection: overwrite internal brake command (0/1)
%     fault_value     value written into the internal brake command [m/s^2]
%
%   Output y (1x19), see AEB_OUTPUT_NAMES in build_aeb_model.m:
%     1 aeb_state code (0 MONITOR 1 STANDBY 2 WARNING 3 PARTIAL_BRAKE
%                       4 FULL_BRAKE 5 DEGRADED 6 FAIL_SAFE)
%     2 brake_req_mps2   3 brake_cmd_internal   4 fcw_warning   5 fault_lamp
%     6 ttc_s   7 confirmed   8 override   9 e2e_radar   10 e2e_camera
%     11 timeout_radar   12 timeout_camera   13 last_valid_radar
%     14 last_valid_camera   15 mismatch   16 monitor_block   17 wd_violations
%     18 radar_range_rx   19 camera_range_rx
%
%   Requirement trace: E2E TSR-01, timeout TSR-02, plausibility FSR-01/TSR-03,
%   monitor TSR-04, output gate TSR-05, override TSR-06, DEGRADED TSR-07,
%   watchdog TSR-08.

%#codegen

% ---- parameters (requirements/aeb_safety.yaml) ---------------------------------
P_PERIOD_MS = 10;      P_TIMEOUT_MS = 150;     P_E2E_MAX_DELTA = 2;
P_E2E_INHIBIT_MS = 150;
P_MISMATCH_ABS_M = 3.0; P_MISMATCH_REL = 0.15; P_MISMATCH_DEBOUNCE_MS = 100;
P_MISMATCH_DEGRADED_MS = 1000;
P_MONITOR_DEBOUNCE_MS = 50; P_MONITOR_TTC_S = 1.9;
P_WD_TOLERANCE = 1.5;  P_WD_MAX_VIOLATIONS = 3;
P_STEER_RATE_DPS = 100.0;
P_SPEED_MIN_MPS = 10.0 / 3.6; P_SPEED_MAX_MPS = 80.0 / 3.6;
P_TTC_WARN_S = 2.6;    P_TTC_PARTIAL_S = 1.6;  P_TTC_FULL_S = 0.9;
P_PARTIAL_MPS2 = 4.0;  P_FULL_MPS2 = 9.0;

S_MONITOR = 0; S_STANDBY = 1; S_WARNING = 2; S_PARTIAL = 3; S_FULL = 4;
S_DEGRADED = 5; S_FAIL_SAFE = 6;

persistent trk st
if isempty(st) || reset
    trk = repmat(new_track(), 1, 2);
    st = struct('state', S_MONITOR, 'fail_safe', false, 'override', false, ...
                'has_mismatch_since', false, 'mismatch_since', 0, ...
                'has_monitor_bad_since', false, 'monitor_bad_since', 0, ...
                'monitor_legit', false, 'has_last_call', false, 'last_call_ms', 0, ...
                'wd_violations', 0);
end

% ---- watchdog (TSR-08) ---------------------------------------------------------------
if st.has_last_call
    if t_ms - st.last_call_ms > P_PERIOD_MS * P_WD_TOLERANCE
        st.wd_violations = st.wd_violations + 1;
    else
        st.wd_violations = 0;
    end
end
st.has_last_call = true;
st.last_call_ms = t_ms;

% ---- E2E receive (TSR-01, TSR-02) ---------------------------------------------------
trk(1) = track_process(trk(1), radar_frames, t_ms, P_TIMEOUT_MS, P_E2E_MAX_DELTA);
trk(2) = track_process(trk(2), camera_frames, t_ms, P_TIMEOUT_MS, P_E2E_MAX_DELTA);

r_to = (t_ms - trk(1).last_valid_ms) > P_TIMEOUT_MS;
c_to = (t_ms - trk(2).last_valid_ms) > P_TIMEOUT_MS;
r_ok = ~r_to && trk(1).has_data && trk(1).ok;
c_ok = ~c_to && trk(2).has_data && trk(2).ok;

% ---- plausibility (FSR-01, TSR-03) ----------------------------------------------------
disagree = r_ok && c_ok && trk(1).obj && trk(2).obj && ...
    abs(trk(1).range_m - trk(2).range_m) > max(P_MISMATCH_ABS_M, P_MISMATCH_REL * trk(1).range_m);
if disagree
    if ~st.has_mismatch_since
        st.has_mismatch_since = true;
        st.mismatch_since = t_ms;
    end
else
    st.has_mismatch_since = false;
end
if st.has_mismatch_since
    mismatch_age = t_ms - st.mismatch_since;
else
    mismatch_age = -1;
end
mismatch = mismatch_age >= P_MISMATCH_DEBOUNCE_MS;

confirmed = r_ok && c_ok && trk(1).obj && trk(2).obj && ~disagree;
if confirmed
    ttc = ttc_of(trk(1).range_m, trk(1).vel_mps);
else
    ttc = Inf;
end
ttc_single = Inf;
if r_ok && ~c_ok && trk(1).obj
    ttc_single = ttc_of(trk(1).range_m, trk(1).vel_mps);
elseif c_ok && ~r_ok && trk(2).obj
    ttc_single = ttc_of(trk(2).range_m, trk(2).vel_mps);
end

% ---- driver override (TSR-06) -----------------------------------------------------------
if driver_brake || abs(steer_rate_dps) > P_STEER_RATE_DPS
    st.override = true;
elseif ttc > P_TTC_WARN_S
    st.override = false;
end

% ---- mode logic ---------------------------------------------------------------------------
started = t_ms >= P_TIMEOUT_MS;
if (started && ~r_ok && ~c_ok) || st.wd_violations >= P_WD_MAX_VIOLATIONS
    st.fail_safe = true;
end
degraded = started && ((r_ok ~= c_ok) || mismatch_age >= P_MISMATCH_DEGRADED_MS);
inhibit = mismatch || st.override || ...
    (t_ms - trk(1).last_e2e_ms) < P_E2E_INHIBIT_MS || (t_ms - trk(2).last_e2e_ms) < P_E2E_INHIBIT_MS;
in_range = ego_speed_mps >= P_SPEED_MIN_MPS && ego_speed_mps <= P_SPEED_MAX_MPS;
braking = st.state == S_PARTIAL || st.state == S_FULL;

if st.fail_safe
    state = S_FAIL_SAFE;
elseif degraded
    state = S_DEGRADED;
elseif braking && ~inhibit && confirmed && ttc < Inf && ego_speed_mps > 0.3
    if st.state == S_FULL || ttc <= P_TTC_FULL_S
        state = S_FULL;
    else
        state = S_PARTIAL;
    end
elseif ~in_range
    state = S_STANDBY;
elseif ~inhibit && ttc <= P_TTC_FULL_S
    state = S_FULL;
elseif ~inhibit && ttc <= P_TTC_PARTIAL_S
    state = S_PARTIAL;
elseif ttc <= P_TTC_WARN_S
    state = S_WARNING;
else
    state = S_MONITOR;
end
st.state = state;

if state == S_PARTIAL
    brake_cmd = P_PARTIAL_MPS2;
elseif state == S_FULL
    brake_cmd = P_FULL_MPS2;
else
    brake_cmd = 0.0;
end
if fault_active
    brake_cmd = fault_value;  % injected corruption of the internal variable
end

% ---- independent monitor (TSR-04) -----------------------------------------------------
fresh = ~r_to && ~c_to;
both = fresh && trk(1).has_data && trk(2).has_data && trk(1).obj && trk(2).obj && trk(1).ok && trk(2).ok;
closing = both && trk(1).vel_mps < -0.1 && trk(2).vel_mps < -0.1;
if closing
    ttc_mon = min(trk(1).range_m, trk(2).range_m) / -trk(1).vel_mps;
else
    ttc_mon = Inf;
end
if ~closing || st.override
    st.monitor_legit = false;
elseif ttc_mon <= P_MONITOR_TTC_S
    st.monitor_legit = true;
end
if st.monitor_legit
    allowed = P_FULL_MPS2;
else
    allowed = 0.0;
end
if brake_cmd > allowed + 0.1
    if ~st.has_monitor_bad_since
        st.has_monitor_bad_since = true;
        st.monitor_bad_since = t_ms;
    end
    if t_ms - st.monitor_bad_since >= P_MONITOR_DEBOUNCE_MS
        st.fail_safe = true;
    end
    brake_out = 0.0;
    monitor_block = true;
else
    st.has_monitor_bad_since = false;
    brake_out = brake_cmd;
    monitor_block = false;
end

% ---- output gate (TSR-05) --------------------------------------------------------------
if st.fail_safe
    brake_out = 0.0;
    st.state = S_FAIL_SAFE;
end

fcw = (state == S_WARNING || state == S_PARTIAL || state == S_FULL) || ...
      (state == S_DEGRADED && ttc_single <= P_TTC_WARN_S);
radar_rx = NaN;
if r_ok && trk(1).obj
    radar_rx = trk(1).range_m;
end
camera_rx = NaN;
if c_ok && trk(2).obj
    camera_rx = trk(2).range_m;
end

y = [st.state, brake_out, brake_cmd, double(fcw && ~st.fail_safe), double(st.fail_safe || degraded), ...
     ttc, double(confirmed), double(st.override), double(trk(1).e2e_now), double(trk(2).e2e_now), ...
     double(r_to), double(c_to), trk(1).last_valid_ms, trk(2).last_valid_ms, ...
     double(mismatch), double(monitor_block), st.wd_violations, radar_rx, camera_rx];
end

% ==== local functions ======================================================================

function t = new_track()
t = struct('has_data', false, 'obj', false, 'ok', false, 'range_m', 0.0, 'vel_mps', 0.0, ...
           'has_counter', false, 'last_counter', 0, 'last_valid_ms', 0, ...
           'last_e2e_ms', -1e9, 'e2e_now', false);
end

function t = track_process(t, frames, t_ms, timeout_ms, max_delta)
t.e2e_now = false;
for k = 1:size(frames, 1)
    f = frames(k, :);
    if f(1) == 0
        continue;
    end
    counter = f(2); obj = f(4); ok = f(5); range_raw = f(6); vel_raw = f(7);
    if f(3) ~= crc8_j1850(range_raw, vel_raw, obj, ok, counter)
        t.e2e_now = true; t.last_e2e_ms = t_ms;
        continue;
    end
    if ~t.has_counter  % first frame after start-up
        t.has_counter = true; t.last_counter = counter;
        t = accept(t, obj, ok, range_raw, vel_raw, t_ms);
        continue;
    end
    delta = mod(counter - t.last_counter, 15);
    resync = (t_ms - t.last_valid_ms) > timeout_ms;
    t.last_counter = counter;
    if delta >= 1 && delta <= max_delta
        t = accept(t, obj, ok, range_raw, vel_raw, t_ms);
    elseif resync && delta ~= 0
        % after a timeout the 4-bit counter may have wrapped: new reference only
    else
        t.e2e_now = true; t.last_e2e_ms = t_ms;
    end
end
end

function t = accept(t, obj, ok, range_raw, vel_raw, t_ms)
t.has_data = true; t.obj = obj ~= 0; t.ok = ok ~= 0;
t.range_m = range_raw / 100; t.vel_mps = vel_raw / 100;
t.last_valid_ms = t_ms;
end

function v = ttc_of(range_m, vel_mps)
if vel_mps < -0.1
    v = range_m / -vel_mps;
else
    v = Inf;
end
end

function crc = crc8_j1850(range_raw, vel_raw, obj, ok, counter)
% CRC-8 SAE J1850 over [range_hi range_lo vel_hi vel_lo flags counter]
vel_u16 = mod(vel_raw, 65536);
bytes = [floor(range_raw / 256), mod(range_raw, 256), floor(vel_u16 / 256), mod(vel_u16, 256), ...
         obj + 2 * ok, mod(counter, 16)];
crc = 255;
for i = 1:6
    crc = bitxor(crc, bytes(i));
    for b = 1:8
        if crc >= 128
            crc = bitxor(mod(crc * 2, 256), 29);
        else
            crc = mod(crc * 2, 256);
        end
    end
end
crc = bitxor(crc, 255);
end
