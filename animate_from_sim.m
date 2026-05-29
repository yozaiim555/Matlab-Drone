%% animate_from_sim.m
%  Post-hoc playback: drive the warehouse uavScenario animation from the state
%  trajectory logged out of quad_sim.slx.
%
%  WORKFLOW
%    1) Run the Simulink sim. You already log the full state as `state_log`
%       inside the single-output object `simOut`, using "Structure with time"
%       format -- this script reads that directly, no extra Mux needed.
%
%    2) Run this script.
%
%  IMPORTANT: confirm the column map below matches the order your state bus is
%  Mux'd in inside Simulink. The .m controller uses
%       [x y z  xd yd zd  phi theta psi  p q r]   (x=1 ... psi=9)
%  but the .slx bus order may differ. The script prints the first logged sample
%  on startup so you can eyeball it -- if the drone lands in the wrong spot or
%  faces the wrong way, fix COL_* and rerun.

%% ---------------- CONFIG ----------------
REBUILD_SCENE = true;       % run init_warehouse.m to (re)create `scene` and `plat`
SHOW_TRAIL    = true;       % draw the flown path as the drone moves

LOG_VAR    = 'state_log';   % To Workspace variable name
SIM_OUTPUT = 'out';         % preferred single-output object name (auto-detect falls back)

% Column indices into the logged data (defaults = .m 12-state order)
COL_X = 1;  COL_Y = 2;  COL_Z = 3;
COL_PHI = 7;  COL_THETA = 8;  COL_PSI = 9;
%% ----------------------------------------

% --- (re)build the warehouse so `scene` and `plat` exist ---
if REBUILD_SCENE
    run('init_warehouse.m');          % defines scene, plat, width, length, height
end
assert(exist('scene','var')==1 && exist('plat','var')==1, ...
    'scene/plat not found. Run init_warehouse.m first or set REBUILD_SCENE = true.');

% --- find LOG_VAR, robust to stale leftover output objects ---------------
% Several runs can leave different output objects in the workspace (e.g. a
% stale `simOut` from an early test AND a fresh `out` from the latest run).
% Reading the wrong one replays an old flight. So: scan every plausible
% container, report what each holds, and use the one with the LONGEST log
% (the latest full run -- a stale test is almost always the short one).
candidates = unique({SIM_OUTPUT, 'out', 'simOut'}, 'stable');
candidates(cellfun(@isempty, candidates)) = [];

found = {}; raws = {};
% loose variable (logged without single-simulation-output)
if exist(LOG_VAR,'var') == 1
    found{end+1} = LOG_VAR; raws{end+1} = eval(LOG_VAR);
end
for ci = 1:numel(candidates)
    nm = candidates{ci};
    if exist(nm,'var') ~= 1, continue; end
    obj = eval(nm);
    try
        L = obj.(LOG_VAR);
        found{end+1} = [nm '.' LOG_VAR]; raws{end+1} = L; %#ok<*SAGROW>
    catch
        % object exists but has no such field -- skip
    end
end

if isempty(found)
    error(['Could not find %s anywhere (loose, out.%s, or simOut.%s). ' ...
           'Run the sim first.'], LOG_VAR, LOG_VAR, LOG_VAR);
end

% measure each candidate's duration, pick the longest
durs = zeros(1, numel(found));
for k = 1:numel(found)
    [tk, ~] = unpack_log(raws{k});
    durs(k) = tk(end) - tk(1);
end
[~, pick] = max(durs);

fprintf('Sources containing %s:\n', LOG_VAR);
for k = 1:numel(found)
    tag = ''; if k == pick, tag = '   <== using (longest = latest run)'; end
    fprintf('   %-18s %6.2f s%s\n', found{k}, durs(k), tag);
end
if numel(found) > 1
    fprintf(['NOTE: multiple output objects found -- one is likely stale. ' ...
             'Tip: `clear out simOut` before each sim run to avoid this.\n']);
end

raw = raws{pick};
[t_raw, D] = unpack_log(raw);

% Make sure rows are time samples (structure-with-time can come out transposed)
if size(D,1) ~= numel(t_raw) && size(D,2) == numel(t_raw)
    D = D.';
end

% Guard against non-monotonic / duplicate time stamps (variable-step solvers)
[t_raw, iu] = unique(t_raw);
D = D(iu, :);

% --- show what we got so the column map / truncation can be diagnosed ---
fprintf('\n===== state_log diagnostics =====\n');
fprintf('%s: %d samples x %d columns.\n', LOG_VAR, size(D,1), size(D,2));
fprintf('Time: %.2f s -> %.2f s   (duration %.2f s)\n', t_raw(1), t_raw(end), t_raw(end)-t_raw(1));
fprintf('First sample: %s\n', mat2str(D(1,:),   3));
fprintf('Last  sample: %s\n', mat2str(D(end,:), 3));
fprintf('Per-column [min .. max] (find which columns span the warehouse, ~0..18):\n');
for c = 1:size(D,2)
    fprintf('   col %2d : [%8.3f .. %8.3f]\n', c, min(D(:,c)), max(D(:,c)));
end
fprintf('Currently mapping -> x:%d y:%d z:%d  phi:%d theta:%d psi:%d\n', ...
        COL_X, COL_Y, COL_Z, COL_PHI, COL_THETA, COL_PSI);
fprintf('=================================\n\n');

pos_raw = [D(:,COL_X),   D(:,COL_Y),     D(:,COL_Z)];
eul_raw = [D(:,COL_PSI), D(:,COL_THETA), D(:,COL_PHI)];   % ZYX order for eul2quat

% --- resample onto the scene's frame clock (matches your Phase 3 cadence) ---
dt     = 1 / scene.UpdateRate;        % 0.01 s at 100 Hz
t_play = (t_raw(1) : dt : t_raw(end)).';
pos    = interp1(t_raw, pos_raw, t_play, 'pchip');
eul    = interp1(t_raw, eul_raw, t_play, 'pchip');
quat   = eul2quat(eul, 'ZYX');        % [psi theta phi] -> [w x y z]

% velocity is visual-only here; finite difference is plenty
vel = [zeros(1,3); diff(pos)/dt];

n_frames = numel(t_play);
fprintf('Loaded %d logged samples, playing back %d frames.\n', numel(iu), n_frames);

%% ---------------- animate ----------------
figure('Name','Closed-loop flight (from Simulink)');
ax = show3D(scene);
view(45, 30); axis equal; hold(ax, 'on');

if SHOW_TRAIL
    trail = plot3(ax, pos(1,1), pos(1,2), pos(1,3), 'b-', 'LineWidth', 1.2);
end

setup(scene);

% teleport to the first logged pose so it doesn't jump from the origin
move(plat, [pos(1,:), vel(1,:), 0 0 0, quat(1,:), 0 0 0]);
show3D(scene, 'Parent', ax, 'FastUpdate', true);
drawnow;

disp('Flying...');
for i = 1:n_frames
    advance(scene);

    motion = [pos(i,:), vel(i,:), 0 0 0, quat(i,:), 0 0 0];
    move(plat, motion);

    if SHOW_TRAIL
        set(trail, 'XData', pos(1:i,1), 'YData', pos(1:i,2), 'ZData', pos(1:i,3));
    end

    show3D(scene, 'Parent', ax, 'FastUpdate', true);
    drawnow limitrate;
end
disp('Playback complete.');

% Optional: overlay the planned aisle-sweep waypoints to inspect tracking.
% Run phase2_planner.m first so `sweep_waypoints` exists, then uncomment:
% scatter3(ax, sweep_waypoints(:,1), sweep_waypoints(:,2), sweep_waypoints(:,3), ...
%          40, 'g', 'filled');

% ===================== local function =====================
function [t, D] = unpack_log(raw)
% Pull (time, data) from a log regardless of save format.
    if isa(raw, 'timeseries')
        t = raw.Time(:);
        D = raw.Data;
    elseif isstruct(raw) && isfield(raw, 'signals')      % "Structure with time"
        t = raw.time(:);
        D = squeeze(raw.signals.values);
    elseif isstruct(raw) && isfield(raw, 'time')         % bare struct fallback
        t = raw.time(:);
        D = raw.data;
    else
        error('Unrecognized log format -- expected timeseries or structure-with-time.');
    end
end