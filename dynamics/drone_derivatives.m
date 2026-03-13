function dS = drone_derivatives(S, u, m, g, k_wind, I)
    % S = [x; vx; y; vy; z; vz; phi; theta; psi; wx; wy; wz]
    phi = S(7); theta = S(8); psi = S(9);
    vx = S(2); vy = S(4); vz = S(6);
    wx = S(10); wy = S(11); wz = S(12);
    Ix = I(1); Iy = I(2); Iz = I(3);
    u1 = u(1); u2 = u(2); u3 = u(3); u4 = u(4);

    % --- Traslación ---
    thrust_z = u1 * cos(phi) * cos(theta);
    friccion_z = k_wind * vz * abs(vz);
    
    dvx = (1/m) * (cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi)) * u1;
    dvy = (1/m) * (cos(phi)*sin(theta)*sin(psi) - cos(psi)*sin(phi)) * u1;
    dvz = -g + (thrust_z - friccion_z) / m;

    % --- Rotación (Dinámica de momentos) ---
    dwx = ((Iy - Iz)/Ix)*wy*wz + (1/Ix)*u2;
    dwy = ((Iz - Ix)/Iy)*wx*wz + (1/Iy)*u3;
    dwz = ((Ix - Iy)/Iz)*wx*wy + (1/Iz)*u4;

    % --- Cinemática (Razon de cambio de ángulos) ---
    dphi   = wx + sin(phi)*(sin(theta)/cos(theta))*wy + cos(phi)*(sin(theta)/cos(theta))*wz;
    dtheta = cos(phi)*wy - sin(phi)*wz;
    dpsi   = (sin(phi)/cos(theta))*wy + (cos(phi)/cos(theta))*wz;

    % Retornar vector de derivadas
    dS = [vx; dvx; vy; dvy; vz; dvz; dphi; dtheta; dpsi; dwx; dwy; dwz];
end