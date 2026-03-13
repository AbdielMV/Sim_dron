% --- DYNAMIC VERTICAL ---
function [z_next, vz_next, x_next, vx_next, y_next, vy_next] = translational_dynamic_u(x_now, vx_now, y_now, vy_now, z_now, vz_now, u1, m, g, dt, k_wind, angles)
    
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    
    % --- DINÁMICA DE TRASLACIÓN COMPLETA ---
    % Proyección del empuje sobre el eje Z inercial:
    % F_vertical = u * cos(phi) * cos(theta)
    thrust_vertical = u1 * cos(phi) * cos(theta);
    
    % La fricción del viento se opone a la velocidad vertical, no depende de los ángulos del cuerpo
    friccion_z = k_wind * vz_now * abs(vz_now);

    acc_x = (1/m) * (cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi)) * u1;
    acc_y = (1/m) * (cos(phi)*sin(theta)*sin(psi) - cos(psi)*sin(phi)) * u1;
    acc_z = -g + (thrust_vertical - friccion_z) / m;
    
    x_next = x_now + (dt*vx_now);
    y_next = y_now + (dt*vy_now);
    z_next = z_now + (dt*vz_now);
    
    vx_next = vx_now + dt * acc_x;
    vy_next = vy_now + dt * acc_y;
    vz_next = vz_now + dt * acc_z;

end