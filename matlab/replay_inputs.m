function Y = replay_inputs(in_csv, out_csv)
%REPLAY_INPUTS  Open-loop replay of recorded controller inputs (back-to-back test).
%   Each row of IN_CSV is one activation as written by safesils.b2b:
%   [t_ms, radar_frames(28, row-major 4x7), camera_frames(28), ego_speed_mps,
%    driver_brake, steer_rate_dps, fault_active, fault_value].
%   Runs in MATLAB and in GNU Octave.
U = dlmread(in_csv, ',');
n = size(U, 1);
Y = zeros(n, 19);
for k = 1:n
    u = U(k, :);
    radar = reshape(u(2:29), 7, 4)';
    camera = reshape(u(30:57), 7, 4)';
    Y(k, :) = aeb_controller_step(k == 1, u(1), radar, camera, u(58), u(59), u(60), u(61), u(62));
end
if nargin > 1
    dlmwrite(out_csv, Y, 'delimiter', ',', 'precision', 17);
end
end
