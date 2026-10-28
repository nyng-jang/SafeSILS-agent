function ok = b2b_simulink(vec_dir, use_model)
%B2B_SIMULINK  Back-to-back test of AEB_Controller.slx against the Python reference, inside MATLAB.
%
%   b2b_simulink                   compares with the vectors in matlab/b2b_vectors
%   b2b_simulink(dir)              uses another vector folder
%   b2b_simulink(dir, false)       checks only aeb_controller_step.m (no Simulink; runs in Octave)
%
%   <tc>.in.csv holds the recorded controller inputs of one scenario, one 10 ms
%   activation per row (layout of replay_inputs.m); <tc>.out.csv holds the Python
%   reference outputs (layout of aeb_controller_step.m). Both are written by
%     safesils b2b scenarios/baseline --export matlab/b2b_vectors
%   The model is simulated open loop with these inputs and every output of every
%   activation must match. Needs AEB_Controller.slx from build_aeb_model.

here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(vec_dir)
    vec_dir = fullfile(here, 'b2b_vectors');
end
if nargin < 2
    use_model = true;
end
addpath(here);

NAMES = {'aeb_state', 'brake_req_mps2', 'brake_cmd_internal', 'fcw_warning', 'fault_lamp', ...
         'ttc_s', 'confirmed', 'override', 'e2e_radar', 'e2e_camera', 'timeout_radar', ...
         'timeout_camera', 'last_valid_radar', 'last_valid_camera', 'mismatch', ...
         'monitor_block', 'wd_violations', 'radar_range_rx', 'camera_range_rx'};
mdl = 'AEB_Controller';
if use_model
    load_system(fullfile(here, [mdl '.slx']));
end

files = dir(fullfile(vec_dir, '*.in.csv'));
if isempty(files)
    error('b2b_simulink:noVectors', 'no *.in.csv files in %s', vec_dir);
end
n_bad = 0;
for i = 1:numel(files)
    tc = strrep(files(i).name, '.in.csv', '');
    fin = fullfile(vec_dir, files(i).name);
    U = dlmread(fin, ',');
    Yexp = dlmread(fullfile(vec_dir, [tc '.out.csv']), ',');

    [n_code, msg_code] = compare(Yexp, replay_inputs(fin), U, NAMES);
    line = sprintf('%-18s MATLAB code %-14s', tc, verdict(n_code));
    msgs = msg_code;
    n_model = 0;
    if use_model
        [n_model, msg_model] = compare(Yexp, simulate(mdl, U), U, NAMES);
        line = [line sprintf(' Simulink %s', verdict(n_model))];
        msgs = [msgs msg_model];
    end
    fprintf('%s\n', line);
    for k = 1:min(numel(msgs), 5)
        fprintf('    %s\n', msgs{k});
    end
    n_bad = n_bad + (n_code > 0 || n_model > 0);
end
fprintf('\n%d of %d scenarios differ\n', n_bad, numel(files));
ok = n_bad == 0;
if use_model
    close_system(mdl, 0);
end
end

function Y = simulate(mdl, U)
% One model step per recorded activation; reset is high in the first step only.
n = size(U, 1);
t = (0:n - 1)' * 0.01;
in = Simulink.SimulationInput(mdl);
in = in.setExternalInput([t, [1; zeros(n - 1, 1)], U]);
inports = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Inport');
for k = 1:numel(inports)
    in = in.setBlockParameter(inports{k}, 'Interpolate', 'off');
end
in = in.setModelParameter('StopTime', sprintf('%.2f', (n - 1) * 0.01), ...
    'LoadExternalInput', 'on', 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
    'SaveFormat', 'Array', 'SaveTime', 'on', 'TimeSaveName', 'tout', ...
    'LimitDataPoints', 'off', 'ReturnWorkspaceOutputs', 'on');
out = sim(in);
step = round(out.tout / 0.01) + 1;
keep = step >= 1 & step <= n;
Y = nan(n, size(out.yout, 2));
Y(step(keep), :) = out.yout(keep, :);
end

function [n, msgs] = compare(Yexp, Y, U, names)
% Same rule as safesils.b2b._same: equal, both NaN, or relative difference <= 1e-9.
msgs = {};
if ~isequal(size(Yexp), size(Y))
    n = 1;
    msgs = {sprintf('size %dx%d vs expected %dx%d', size(Y, 1), size(Y, 2), size(Yexp, 1), size(Yexp, 2))};
    return
end
same = (Yexp == Y) | (isnan(Yexp) & isnan(Y)) | abs(Yexp - Y) <= 1e-9 * max(1, abs(Yexp));
[r, c] = find(~same);
n = numel(r);
for k = 1:min(n, 5)
    msgs{end + 1} = sprintf('t=%d ms %s: python %.17g vs matlab %.17g', ...
        U(r(k), 1), names{c(k)}, Yexp(r(k), c(k)), Y(r(k), c(k))); %#ok<AGROW>
end
end

function s = verdict(n)
if n == 0
    s = 'MATCH';
else
    s = sprintf('%d differences', n);
end
end
