function X_new = plant_step(X, U1, U2, U3, U4, dt, m, g, Ixx, Iyy, Izz)
    phi=X(7); theta=X(8); psi=X(9); p=X(10); q=X(11); r=X(12);

    % Translational accelerations (ENU, z-up)
    xdd = U1/m * (cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
    ydd = U1/m * (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));
    zdd = -g + U1/m * cos(theta)*cos(phi);

    % Rotational accelerations (Euler, body frame)
    pdot = ((Iyy-Izz)*q*r + U2) / Ixx;
    qdot = ((Izz-Ixx)*p*r + U3) / Iyy;
    rdot = ((Ixx-Iyy)*p*q + U4) / Izz;

    % Euler angle rates
    phidot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    thetadot = q*cos(phi) - r*sin(phi);
    psidot   = (q*sin(phi) + r*cos(phi)) / cos(theta);

    % Euler integrate
    Xdot = [X(4); X(5); X(6); xdd; ydd; zdd;
            phidot; thetadot; psidot; pdot; qdot; rdot];
    X_new = X + dt * Xdot;
end