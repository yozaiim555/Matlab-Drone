run('params.m')
simOut = sim('quad_sim');
sl = simOut.state_log;
if isfield(sl,'time'), t=sl.time; st=sl.signals.values;
else, t=sl.Time; st=sl.Data; end

% Position and attitude
figure('Name','Position and attitude')
cols   = [1,    2,    3,    7,       8,         9];
labels = {'x (m)','y (m)','z (m)','\phi (deg)','\theta (deg)','\psi (deg)'};
refs   = [0, 0, z_ref, phi_ref, theta_ref, psi_ref];
scales = [1, 1, 1, 180/pi, 180/pi, 180/pi];
for i = 1:6
    subplot(2,3,i)
    plot(t, st(:,cols(i))*scales(i), 'LineWidth', 1.5)
    yline(refs(i)*scales(i), '--r')
    title(labels{i}); grid on
end

% Body rates
figure('Name','Body rates')
rate_cols  = [10, 11, 12];
rate_names = {'p (rad/s)','q (rad/s)','r (rad/s)'};
for i = 1:3
    subplot(3,1,i)
    plot(t, st(:,rate_cols(i)), 'LineWidth', 1.5)
    yline(0,'--r'); title(rate_names{i}); grid on
end

% Metrics
channels = {'Altitude z','Roll phi','Pitch theta','Roll rate p'};
sigs     = {st(:,3)', st(:,7)', st(:,8)', st(:,10)'};
inits    = {pos0(3),   ang0(1),  ang0(2),  rate0(1)};
tgts     = {z_ref,     phi_ref,  theta_ref, 0};

fprintf('\n%-15s  %10s  %12s\n','Channel','Overshoot','Settling (s)')
fprintf('%s\n', repmat('-',1,42))
for i = 1:4
    step_size = tgts{i} - inits{i};
    if abs(step_size) < 1e-6
        fprintf('%-15s  %10s  %12s\n', channels{i}, 'N/A', 'N/A')
        continue
    end
    if step_size > 0
        os = max(0, (max(sigs{i}) - tgts{i}) / abs(step_size) * 100);
    else
        os = max(0, (tgts{i} - min(sigs{i})) / abs(step_size) * 100);
    end
    band     = 0.02 * abs(step_size);
    outside  = abs(sigs{i} - tgts{i}) > band;
    last_out = find(outside, 1, 'last');
    if isempty(last_out),       ts = 0;
    elseif last_out==length(t), ts = Inf;
    else,                       ts = t(last_out+1);
    end
    fprintf('%-15s  %9.1f%%  %12.3f\n', channels{i}, os, ts)
end

cols   = [1,    2,    3,    7,       8,         9];
labels = {'x (m)','y (m)','z (m)','\phi (deg)','\theta (deg)','\psi (deg)'};
refs   = [0, 0, z_ref, phi_ref, theta_ref, psi_ref];
scales = [1, 1, 1, 180/pi, 180/pi, 180/pi];

figure
for i = 1:6
    subplot(2,3,i)
    plot(t, st(:,cols(i))*scales(i), 'LineWidth', 1.5)
    title(labels{i})
    grid on
end