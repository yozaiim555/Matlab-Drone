function [u, integral_new] = pid_step(err, err_prev, integral, Kp, Ki, Kd, Ts, u_min, u_max)
    derivative = (err - err_prev) / Ts;
    integral_new = integral + err * Ts;

    u = Kp * err + Ki * integral_new + Kd * derivative;

    % Anti-windup: clamp output AND unwind integrator
    if u > u_max
        u = u_max;
        integral_new = integral;   % freeze integrator at boundary
    elseif u < u_min
        u = u_min;
        integral_new = integral;
    end
end