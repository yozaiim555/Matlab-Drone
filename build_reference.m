%% build_reference.m
%  Turn the obstacle-aware aisle-sweep path into TIME-VARYING references for
%  quad_sim.slx, so the controller flies the planned (collision-free) route
%  instead of a single straight-line setpoint from params.m.
%
%  PREREQ: run init_warehouse.m then phase2_planner.m first, so `full_path`
%          (the Nx3 collision-checked waypoint list) exists in the workspace.

assert(exist('full_path','var')==1, ...
    'full_path not found -- run init_warehouse.m then phase2_planner.m first.');

%% ---------------- CONFIG ----------------
flight_speed = 2.0;    % m/s along the path (match the planner)
dt_ref       = 0.01;   % reference sample period (s)
psi_const    = 0;      % hold heading (rad). Change later if you want to yaw.
%% ----------------------------------------

% --- densify the waypoint path at constant speed: one sample per dt_ref ---
ref = [];
for i = 1:size(full_path,1) - 1
    p1 = full_path(i,   :);
    p2 = full_path(i+1, :);
    seg = norm(p2 - p1);
    if seg < 1e-3, continue; end
    n = max(2, ceil((seg / flight_speed) / dt_ref));
    for k = 0:n-1
        a = k / n;
        ref = [ref; p1 + a*(p2 - p1)];
    end
end
ref = [ref; full_path(end, :)];

t_ref = (0:size(ref,1)-1).' * dt_ref;
T_end = t_ref(end);

% --- timeseries ready for "From Workspace" blocks (or root-inport Input) ---
x_ref_ts   = timeseries(ref(:,1),                 t_ref);
y_ref_ts   = timeseries(ref(:,2),                 t_ref);
z_ref_ts   = timeseries(ref(:,3),                 t_ref);
psi_ref_ts = timeseries(psi_const*ones(size(t_ref)), t_ref);

fprintf('Reference built: duration %.1f s, %d samples.\n', T_end, numel(t_ref));
fprintf('==> Set the quad_sim.slx Stop Time to at least %.0f s.\n', ceil(T_end));

% Quick look at the reference path (should weave down the aisle, not cross shelves)
figure('Name','Reference path fed to Simulink');
plot3(ref(:,1), ref(:,2), ref(:,3), 'b-', 'LineWidth', 1.5); grid on; axis equal;
xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
title('Time-varying reference (aisle sweep)'); view(45,30);
