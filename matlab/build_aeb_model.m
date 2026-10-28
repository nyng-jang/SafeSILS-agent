function build_aeb_model(export_fmu)
%BUILD_AEB_MODEL  Create AEB_Controller.slx and optionally export it as an FMU.
%
%   build_aeb_model          builds and saves matlab/AEB_Controller.slx
%   build_aeb_model(true)    also exports matlab/AEB_Controller.fmu
%                            (FMI 2.0 co-simulation, used by safesils.external.FMUController)
%
%   The model is a fixed-step discrete (10 ms) wrapper around
%   aeb_controller_step.m inside a MATLAB Function block. Port names and order
%   are the contract with the Python harness (safesils/external.py):
%     inputs  reset, t_ms, radar_frames[28], camera_frames[28], ego_speed_mps,
%             driver_brake, steer_rate_dps, fault_active, fault_value
%     outputs one scalar port per entry of OUTPUT_NAMES below
%
%   Written for MATLAB/Simulink R2026b. Standalone FMU export may need
%   Simulink Compiler depending on the licence; if exportToFMU is unavailable,
%   use Simulink > Save As > Export Model to > Standalone FMU with the same settings.

if nargin < 1
    export_fmu = false;
end

mdl = 'AEB_Controller';
here = fileparts(mfilename('fullpath'));
addpath(here);

INPUTS = {'reset', 1; 't_ms', 1; 'radar_frames', 28; 'camera_frames', 28; ...
          'ego_speed_mps', 1; 'driver_brake', 1; 'steer_rate_dps', 1; ...
          'fault_active', 1; 'fault_value', 1};
OUTPUT_NAMES = {'aeb_state', 'brake_req_mps2', 'brake_cmd_internal', 'fcw_warning', 'fault_lamp', ...
                'ttc_s', 'confirmed', 'override', 'e2e_radar', 'e2e_camera', 'timeout_radar', ...
                'timeout_camera', 'last_valid_radar', 'last_valid_camera', 'mismatch', ...
                'monitor_block', 'wd_violations', 'radar_range_rx', 'camera_range_rx'};

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
slx = fullfile(here, [mdl '.slx']);
if exist(slx, 'file')
    delete(slx);  % rebuilt from scratch; avoids the "model name is shadowing" warning
end
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
          'FixedStep', '0.01', 'StopTime', 'inf');

% ---- MATLAB Function block --------------------------------------------------------
fcn = [mdl '/aeb_controller'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, 'Position', [400 40 620 40 + 22 * numel(OUTPUT_NAMES)]);
block = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', fcn);
in_list = strjoin(INPUTS(:, 1)', ', ');
out_list = strjoin(OUTPUT_NAMES, ', ');
body = sprintf([ ...
    'function [%s] = aeb_controller(%s)\n' ...
    '%%#codegen\n' ...
    'y = aeb_controller_step(reset ~= 0, t_ms, reshape(radar_frames, 7, 4)'', reshape(camera_frames, 7, 4)'', ...\n' ...
    '    ego_speed_mps, driver_brake ~= 0, steer_rate_dps, fault_active ~= 0, fault_value);\n'], out_list, in_list);
for k = 1:numel(OUTPUT_NAMES)
    body = [body sprintf('%s = y(%d);\n', OUTPUT_NAMES{k}, k)]; %#ok<AGROW>
end
block.Script = body;

% ---- ports -------------------------------------------------------------------------------
for k = 1:size(INPUTS, 1)
    name = INPUTS{k, 1};
    blk = [mdl '/' name];
    add_block('simulink/Sources/In1', blk, 'Position', [100 30 + 40 * k 130 44 + 40 * k]);
    set_param(blk, 'PortDimensions', num2str(INPUTS{k, 2}), 'OutDataTypeStr', 'double', 'SampleTime', '0.01');
    add_line(mdl, [name '/1'], sprintf('aeb_controller/%d', k), 'autorouting', 'on');
end
for k = 1:numel(OUTPUT_NAMES)
    name = OUTPUT_NAMES{k};
    add_block('simulink/Sinks/Out1', [mdl '/' name], 'Position', [760 30 + 30 * k 790 44 + 30 * k]);
    add_line(mdl, sprintf('aeb_controller/%d', k), [name '/1'], 'autorouting', 'on');
end

set_param(mdl, 'SimulationCommand', 'update');  % compile once to catch errors early
save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('saved %s\n', fullfile(here, [mdl '.slx']));

if export_fmu
    old = cd(here);
    cleanup = onCleanup(@() cd(old));
    if ~isempty(which('exportToFMU'))
        exportToFMU(mdl, 'FMIVersion', '2.0', 'FMUType', 'CS', 'SaveDirectory', here);
    elseif ~isempty(which('exportToFMU2CS'))  % releases before R2023b
        exportToFMU2CS(mdl, 'SaveDirectory', here);
    else
        error('build_aeb_model:noFMUExport', ...
              ['%s.slx was saved, but FMU export is not available.\n' ...
               'Standalone FMU export needs the Simulink Compiler add-on ' ...
               '(Home > Add-Ons > Get Add-Ons > "Simulink Compiler").\n' ...
               'Check with: ver simulinkcompiler'], mdl);
    end
    fprintf('exported %s\n', fullfile(here, [mdl '.fmu']));
end
end
