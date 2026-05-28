%Quadcopter physical parameters
% ── Physical properties ──────────────────────
m   = 0.5;          % total mass (kg)
g   = 9.81;         % gravity (m/s²)
L   = 0.175;        % arm length, centre to motor (m)

% ── Inertia tensor (diagonal, symmetric quad) ─
Ixx = 4.9e-3;      % roll  (kg·m²)
Iyy = 4.9e-3;      % pitch (kg·m²)
Izz = 8.8e-3;      % yaw   (kg·m²)

% ── Motor/propeller coefficients ─────────────
kT  = 2.9e-5;      % thrust coeff  Fi = kT*wi^2  (N·s²)
kD  = 1.1e-6;      % drag torque   Qi = kD*wi^2  (N·m·s²)
w_hover = sqrt(m*g / (4*kT));  % hover RPM Totalthrust=Weight
w_max = sqrt(2*m*g / (4*kT));
max_tilt = 20 * pi/180;   % 20 degree maximum tilt
% ── Initial conditions ───────────────────────
pos0 = [0; 0; 0];   % x y z  (m)
vel0 = [0; 0; 0];   % xdot ydot zdot  (m/s)
ang0 = [0; 0; 0];   % phi theta psi  (rad)
rate0= [0; 0; 0];   % p q r  (rad/s)

% ── Target ───────────────────────
x_ref = 2;
y_ref = 2;
z_ref = 2;      
phi_ref = 0;
theta_ref = 0;
psi_ref = 0;

% Trajectory parameters
v_max = 2.0;        % m/s maximum velocity
a_max = 1.5;        % m/s² maximum acceleration (links to max tilt: a_max = g*tan(max_tilt))

% Waypoint — where you want the drone to go
x_target = 5.0;
y_target = 10.0;
z_target = 3.0;

% Start position
x_start = pos0(1);
y_start = pos0(2);
z_start = pos0(3);

% Consistent constraint — max horizontal accel from max tilt
a_max_check = g * tan(max_tilt);   % = 9.81 * tan(20°) = 3.57 m/s²
% Use a_max < a_max_check to stay away from saturation with margin

% ── Sim settings ─────────────────────────────
Ts  = 0.01;        % sample time (s) — 0.1 kHz
Tf  = 5;           % sim duration (s)

disp("params loaded. hover speed: " + w_hover + " rad/s")

% The four inputs are w1^2, w2^2, w3^2, w4^2 
% Connect them into a Mux → 4×1 vector, then multiply by the allocation matrix:

%      [U1] = [kT    kT    kT    kT  ] [w1²]
%      [U2] = [kT*L  -kT*L  -kT*L  kT*L] [w2²]
%      [U3] =  [-kT*L -kT*L  kT*L  kT*L] [w3²]
%      [U4] =  [-kD   kD    -kD   kD  ] [w4²]

% In MATLAB workspace, define the matrix:
Alloc = [kT,      kT,    kT,    kT;
         kT*L,  -kT*L, -kT*L,  kT*L;
        -kT*L,  -kT*L,  kT*L,  kT*L;
        -kD,     kD,   -kD,    kD];

AllocInv = inv(Alloc);
% Test 1: Hover — set all motors to w_hover, run 5 seconds
% Expected: z stays at 0, x/y stay at 0, all angles stay at 0
%w_cmd = [w_hover; w_hover; w_hover; w_hover];

% Test 2: Throttle up — increase all motors 5% above hover
% Expected: z increases (drone climbs), x/y/angles stay near 0
%w_cmd = [1.05; 1.05; 1.05; 1.05] * w_hover;

% Test 3: Roll — increase M1+M4 (left pair), decrease M2+M3 (right pair)
% Expected: phi grows positively, drone drifts in y
% dw = w_hover * 0.05;
% %w_cmd = w_hover + [dw; -dw; -dw; dw];
% 
% % Test 4: Yaw — increase CW pair (M2+M4), decrease CCW pair (M1+M3)
% % Expected: psi grows, position stays near 0
% w_cmd = w_hover + [-dw; dw; -dw; dw];
 %%
simOut = sim('quad_sim');
t   = simOut.state_log.time;
st  = simOut.state_log.signals.values;
%Position
figure('Name', 'Position');
sgtitle('Position')   % first figure
subplot(3,1,1); plot(t, st(:,1), 'Color', '#E24B4A', 'LineWidth', 1.5)
yline(0, '--', 'Color', [0.7 0.7 0.7])
ylabel('m'); title('x (forward)'); grid on

subplot(3,1,2); plot(t, st(:,2), 'Color', '#1D9E75', 'LineWidth', 1.5)
yline(0, '--', 'Color', [0.7 0.7 0.7])
ylabel('m'); title('y (left)'); grid on

subplot(3,1,3); plot(t, st(:,3), 'Color', '#378ADD', 'LineWidth', 1.5)
yline(0, '--', 'Color', [0.7 0.7 0.7])
ylabel('m'); title('z (up)'); grid on

xlabel('time (s)')

%Angles
figure('Name', 'Angles');
sgtitle('Euler angles ')
subplot(3,1,1); plot(t, wrapTo180(st(:,7)  * 180/pi), 'Color', '#E24B4A', 'LineWidth', 1.5)
yline(0, '--', 'Color', [0.7 0.7 0.7])
ylabel('deg'); title('\phi (roll)'); ylim([-180 180]); yticks(-180:90:180); grid on

subplot(3,1,2); plot(t, wrapTo180(st(:,8)  * 180/pi), 'Color', '#1D9E75', 'LineWidth', 1.5)
yline(0, '--', 'Color', [0.7 0.7 0.7])
ylabel('deg'); title('\theta (pitch)'); ylim([-180 180]); yticks(-180:90:180); grid on

subplot(3,1,3); plot(t, wrapTo180(st(:,9)  * 180/pi), 'Color', '#378ADD', 'LineWidth', 1.5)
yline(0, '--', 'Color', [0.7 0.7 0.7])
ylabel('deg'); title('\psi (yaw)'); ylim([-180 180]); yticks(-180:90:180); grid on

xlabel('time (s)')

% Body rates
figure('Name','Rates')
subplot(3,1,1); plot(t, st(:,10), 'Color','#E24B4A', 'LineWidth',1.5)
yline(0,'--','Color',[.7 .7 .7]); ylabel('rad/s'); title('p (roll rate)'); grid on
subplot(3,1,2); plot(t, st(:,11), 'Color','#1D9E75', 'LineWidth',1.5)
yline(0,'--','Color',[.7 .7 .7]); ylabel('rad/s'); title('q (pitch rate)'); grid on
subplot(3,1,3); plot(t, st(:,12), 'Color','#378ADD', 'LineWidth',1.5)
yline(0,'--','Color',[.7 .7 .7]); ylabel('rad/s'); title('r (yaw rate)'); grid on
xlabel('time (s)')

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
        fprintf('%-15s  %10s  %12s\n', channels{i},'N/A','N/A')
        continue
    end
    if step_size > 0
        os = max(0,(max(sigs{i}) - tgts{i}) / abs(step_size) * 100);
    else
        os = max(0,(tgts{i} - min(sigs{i})) / abs(step_size) * 100);
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
%% Build A and B analytically
% A = zeros(12);
% 
% % Position kinematics — velocity feeds position
% A(1,4) = 1;  % ẋ → x
% A(2,5) = 1;  % ẏ → y
% A(3,6) = 1;  % ż → z
% 
% % Translational acceleration — gravity coupling
% A(4,8) =  g; % pitch θ → x-acceleration
% A(5,7) = -g; % roll  φ → y-acceleration
% 
% % Angle kinematics — body rates feed angles
% A(7,10) = 1; % p → φ̇
% A(8,11) = 1; % q → θ̇
% A(9,12) = 1; % r → ψ̇
% 
% %% Build B
% B = zeros(12, 4);
% B(6,  1) = 1/m;    % U1 → z̈
% B(10, 2) = 1/Ixx;  % U2 → ṗ
% B(11, 3) = 1/Iyy;  % U3 → q̇
% B(12, 4) = 1/Izz;  % U4 → ṙ
% 
% %% Controllability
% Co = ctrb(A, B);
% fprintf('Controllability rank : %d \n',  rank(ctrb(A,B)))
% 
% %% Observability (all states measured)
% C_out = eye(12);
% D_out = zeros(12, 4);
% Ob = obsv(A, C_out);
% fprintf('Observability rank:')
% disp(rank(Ob))        %=12
% 
% %% Eigenvalues — all should be zero
% fprintf('Eigenvalues of A : %s\n', num2str(eig(A)')) % twelve zeros → marginally stable
% 
% %% Build state-space
% sys = ss(A, B, C_out, D_out);
% fprintf('Poles: : %s\n', num2str(pole(sys)'))
%%
% Rate loop (roll and pitch identical, yaw uses Izz)
P_rate_roll  = tf(1, [Ixx 0]);
P_rate_pitch = tf(1, [Iyy 0]);
P_rate_yaw   = tf(1, [Izz 0]);

% Attitude loop
P_att = tf(1, [1 0]);
Q_att = tf(1, [1 0]);
R_att = tf(1, [1 0]);

% Altitude loop
z_alt = tf(1, [m 0 0]);
x = tf(g, [1 0 0]);
y = tf(g, [1 0 0]);

% Prefilters so the controller rises smoothly 0 → ref
%Cancels the zero for plant*PID

F_pos_num = Kp_xy / Kd_xy;               
F_pos_den = [1, Kp_xy / Kd_xy];

%%
wn_rate  = 40;


% With P only (Kd=0):
Kp_rate_roll  = wn_rate * Ixx;    % 40 * 4.9e-3 
Kp_rate_pitch = wn_rate * Iyy;
Kp_rate_yaw   = (wn_rate/2) * Izz;

% Kd = 0 for rate loop
Kd_rate_roll  = 0.1 * Kp_rate_roll;   % small, ~10% of Kp
Kd_rate_pitch = 0.1 * Kp_rate_pitch;
Kd_rate_yaw   = 0.1 * Kp_rate_yaw;


P_rate = tf(1, [Ixx 0]);
C_rate = tf(Kp_rate_roll, 1);    % pure P
sys_cl = feedback(P_rate * C_rate, 1);

pole(sys_cl)                      % should be at -40
stepinfo(sys_cl)                  % settling time should be ~0.13s
figure; step(sys_cl); grid on
%%
wn_att = 12;
Kp_att_roll  = wn_att;   
Kp_att_pitch = wn_att;
Kp_att_yaw   = wn_att/2;  % slower for yaw

P_att  = tf(1, [1 0]);
C_att  = tf(Kp_att_roll, 1);
sys_cl = feedback(P_att * C_att, 1);

pole(sys_cl)          % should be at -12
stepinfo(sys_cl)      % settling time ~0.5s
figure; step(sys_cl); grid on
%% For Z
wn_alt   = 3.5;
zeta_alt = 1.0;
Kp_alt   = wn_alt^2 * m;
Kd_alt   = 2 * zeta_alt * wn_alt * m;

P_alt    = tf(1, [m 0 0]);
C_alt    = tf([Kd_alt Kp_alt], 1);
F        = tf(Kp_alt/Kd_alt, [1 Kp_alt/Kd_alt]);

sys_cl   = feedback(P_alt * C_alt, 1);
sys_f    = F * sys_cl;

stepinfo(sys_f)
figure; step(sys_f); grid on
%% For x and y
wn_xy   = 3.5;
zeta_xy = 1.0;

Kp_xy   = wn_xy^2 / g;
Kd_xy   = (2 * zeta_xy * wn_xy) / g;

P_xy = tf(g, [1 0 0]);
C_xy    = tf([Kd_xy Kp_xy], 1);
F_xy        = tf(Kp_xy/Kd_xy, [1 Kp_xy/Kd_xy]);

sys_cl_xy   = feedback(P_xy * C_xy, 1);
sys_f    = F_xy * sys_cl_xy;

stepinfo(sys_f)
figure; step(sys_f); grid on

%%
P_pos  = tf(g, [1 0 0]);
C_pos  = tf([Kd_xy Kp_xy], 1);
F      = tf(F_pos_num, F_pos_den);

sys_cl    = feedback(P_pos * C_pos, 1);
sys_with_F = F * sys_cl;

figure
step(sys_cl, sys_with_F)
legend('Without prefilter', 'With prefilter')
grid on

%% Prefilter effect
figure
plot(simOut.prefilter_out.time, simOut.prefilter_out.signals.values)
hold on
plot(simOut.prefilter_out.time, ones(size(simOut.prefilter_out.time)) * x_ref, '--r')
title('Prefilter output vs x\_ref')
legend('Prefilter output','x\_ref step')
grid on

%%
figure
plot(simOut.theta_des.time, simOut.theta_des.signals.values * 180/pi)
yline(20, '--r')
yline(-20, '--r')
title('theta\_des after saturation')
ylabel('deg')
grid on