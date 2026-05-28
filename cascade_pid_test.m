run('params.m')

Tf_sim = 10; Ts_ctrl = 0.005;  % 200 Hz controller
t = 0:Ts_ctrl:Tf_sim;
N = length(t);

% Setpoints
z_ref    =  0;    % climb to 1 m
phi_ref  =  0;      % level roll
theta_ref = 0;      % level pitch
psi_ref  =  0;      % hold heading

% State: [x y z xd yd zd phi theta psi p q r]
X = zeros(12, N);
X(:,1) = [pos0; vel0; ang0; rate0];

% PID integrators (one per channel)
I = struct('alt',z_ref - pos0(3), 'rr',0, 'rp',0, 'ry',0, 'ar',0, 'ap',0, 'ay',0);
e_prev = struct('alt',0, 'rr',0, 'rp',0, 'ry',0, 'ar',0, 'ap',0, 'ay',0);

for k = 1:N-1
    x = X(:,k);
    z=x(3); phi=x(7); theta=x(8); psi=x(9); p=x(10); q=x(11); r=x(12);

    % ---- Altitude loop (outer) ----
    e_alt = z_ref - z;
    [dU1, I.alt] = pid_step(e_alt, e_prev.alt, I.alt, ...
                             Kp_alt, Ki_alt, Kd_alt, Ts_ctrl, -m*g, m*g);
    e_prev.alt = e_alt;
    U1 = U1_hover + dU1;          % feed-forward + correction
    U1 = max(0, min(2*m*g, U1));  % negative thrust and weight to thrust ratio limit

    % ---- Attitude angle loop → desired rates ----
    e_ar = phi_ref - phi;
    e_ap = theta_ref - theta;
    e_ay = psi_ref - psi;

    [pd_des, I.ar] = pid_step(e_ar, e_prev.ar, I.ar, Kp_att_roll, Ki_att_roll, Kd_att_roll, Ts_ctrl, -10, 10);
    [qd_des, I.ap] = pid_step(e_ap, e_prev.ap, I.ap, Kp_att_pitch, Ki_att_pitch, Kd_att_pitch, Ts_ctrl, -10, 10);
    [rd_des, I.ay] = pid_step(e_ay, e_prev.ay, I.ay, Kp_att_yaw, Ki_att_yaw, Kd_att_yaw, Ts_ctrl, -5, 5);
    e_prev.ar=e_ar; e_prev.ap=e_ap; e_prev.ay=e_ay;

    % ---- Rate loop (inner) → torques ----
    e_rr = pd_des - p;
    e_rp = qd_des - q;
    e_ry = rd_des - r;

    [U2, I.rr] = pid_step(e_rr, e_prev.rr, I.rr, Kp_rate_roll, Ki_rate_roll, Kd_rate_roll, Ts_ctrl, -0.5, 0.5);
    [U3, I.rp] = pid_step(e_rp, e_prev.rp, I.rp, Kp_rate_pitch, Ki_rate_pitch, Kd_rate_pitch, Ts_ctrl, -0.5, 0.5);
    [U4, I.ry] = pid_step(e_ry, e_prev.ry, I.ry, Kp_rate_yaw, Ki_rate_yaw, Kd_rate_yaw, Ts_ctrl, -0.3, 0.3);
    e_prev.rr=e_rr; e_prev.rp=e_rp; e_prev.ry=e_ry;

    % ---- Propagate plant (Euler integration of your EOM) ----
    X(:,k+1) = plant_step(X(:,k), U1, U2, U3, U4, Ts_ctrl, m, g, Ixx, Iyy, Izz);
end

% ---- Plot ----
figure('Name','Cascade PID — altitude + attitude')
subplot(2,2,1); plot(t,X(3,:),'b','LineWidth',1.5); yline(z_ref,'--r'); ylabel('m'); title('Altitude z'); grid on
subplot(2,2,2); plot(t,X(7,:)*180/pi,'r','LineWidth',1.5); yline(0,'--'); ylabel('deg'); title('Roll \phi'); grid on
subplot(2,2,3); plot(t,X(8,:)*180/pi,'g','LineWidth',1.5); yline(0,'--'); ylabel('deg'); title('Pitch \theta'); grid on
subplot(2,2,4); plot(t,X(9,:)*180/pi,'m','LineWidth',1.5); yline(0,'--'); ylabel('deg'); title('Yaw \psi'); grid on
xlabel('time (s)')
    
figure('Name','Rates')
subplot(3,1,1); plot(t, X(10,:), 'r', 'LineWidth', 1.5)
yline(0,'--','Color',[.7 .7 .7]); ylabel('rad/s'); title('p (roll rate)'); grid on
subplot(3,1,2); plot(t, X(11,:), 'g', 'LineWidth', 1.5)
yline(0,'--','Color',[.7 .7 .7]); ylabel('rad/s'); title('q (pitch rate)'); grid on
subplot(3,1,3); plot(t, X(12,:), 'b', 'LineWidth', 1.5)
yline(0,'--','Color',[.7 .7 .7]); ylabel('rad/s'); title('r (yaw rate)'); grid on
xlabel('time (s)')

step_size = phi_ref - ang0(1);

% Overshoot
if step_size > 0
    peak = max(X(7,:));
else
    peak = min(X(7,:));
end
overshoot = max(0, (peak - phi_ref) / abs(step_size) * 100);

% Settling time
band = 0.02 * abs(step_size);
outside = abs(X(7,:) - phi_ref) > band;

last_out = find(outside, 1, 'last');

if isempty(last_out)
    t_settle = 0;
elseif last_out == length(t)
    t_settle = NaN;
else
    t_settle = t(last_out + 1);
end

fprintf('Roll — overshoot: %.1f%%   settling time: %.3f s\n', overshoot, t_settle)
