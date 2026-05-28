t_vec  = 0:0.01:10;
start  = [0;0;0];
target = [5;3;1];
v_max  = 1.5;
a_max  = 1.0;
 
pos = zeros(3, length(t_vec));
vel = zeros(3, length(t_vec));

for k = 1:length(t_vec)
    [pos(:,k), vel(:,k)] = trap_traj(t_vec(k), start, target, v_max, a_max);
end

figure
subplot(2,1,1)
plot(t_vec, pos(1,:), 'r', t_vec, pos(2,:), 'g', t_vec, pos(3,:), 'b', 'LineWidth', 1.5)
legend('x','y','z'); ylabel('m'); title('Reference position'); grid on

subplot(2,1,2)
plot(t_vec, vel(1,:), 'r', t_vec, vel(2,:), 'g', t_vec, vel(3,:), 'b', 'LineWidth', 1.5)
legend('vx','vy','vz'); ylabel('m/s'); title('Reference velocity'); grid on
xlabel('time (s)')