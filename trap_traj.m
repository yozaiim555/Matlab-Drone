function [pos_ref, vel_ref] = trap_traj(t, start, target, v_max, a_max)
% Trapezoidal velocity profile trajectory generator
% Operates independently per axis
% Inputs:
%   t       — current time (scalar)
%   start   — [3x1] start position [x;y;z]
%   target  — [3x1] target position [x;y;z]
%   v_max   — maximum speed (m/s)
%   a_max   — maximum acceleration (m/s²)
% Outputs:
%   pos_ref — [3x1] current reference position
%   vel_ref — [3x1] current reference velocity

    pos_ref = zeros(3,1);
    vel_ref = zeros(3,1);

    for i = 1:3
        d = target(i) - start(i);      % total displacement on this axis
        if abs(d) < 1e-6
            pos_ref(i) = target(i);
            vel_ref(i) = 0;
            continue
        end

        % Direction of travel
        s = sign(d);
        D = abs(d);

        % Time to accelerate to v_max
        t_acc = v_max / a_max;

        % Distance covered during acceleration phase
        d_acc = 0.5 * a_max * t_acc^2;

        % Check if there is enough distance to reach v_max
        if 2 * d_acc > D
            % Triangular profile — never reaches v_max
            t_acc  = sqrt(D / a_max);
            t_flat = 0;
            v_peak = a_max * t_acc;
        else
            % Trapezoidal profile — reaches v_max
            d_flat = D - 2 * d_acc;
            t_flat = d_flat / v_max;
            v_peak = v_max;
        end

        % Total time for this axis
        t_total = 2 * t_acc + t_flat;

        if t >= t_total
            % Arrived
            pos_ref(i) = target(i);
            vel_ref(i) = 0;

        elseif t < t_acc
            % Acceleration phase
            pos_ref(i) = start(i) + s * 0.5 * a_max * t^2;
            vel_ref(i) = s * a_max * t;

        elseif t < t_acc + t_flat
            % Cruise phase
            t_c = t - t_acc;
            pos_ref(i) = start(i) + s * (0.5 * a_max * t_acc^2 + v_peak * t_c);
            vel_ref(i) = s * v_peak;

        else
            % Deceleration phase
            t_d = t - t_acc - t_flat;
            pos_ref(i) = start(i) + s * (D - 0.5 * a_max * (t_acc - t_d)^2);
            vel_ref(i) = s * (v_peak - a_max * t_d);
        end
    end
end