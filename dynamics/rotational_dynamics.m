% --- ROTATIONAL DYNAMIC ---
function [ang_next, omega_next] = rotational_dynamics(ang_now, omega_now, u_rot_now, I, dt)
    % Desempaquetar estados actuales
    phi   = ang_now(1);
    theta = ang_now(2);
    psi = ang_now(3); % No se usa explícitamente en la dinámica, solo se integra
    
    w_x = omega_now(1); % omega_x
    w_y = omega_now(2); % omega_y
    w_z = omega_now(3); % omega_z
    
    Ix = I(1); Iy = I(2); Iz = I(3);
    u2 = u_rot_now(2); u3 = u_rot_now(3); u4 = u_rot_now(4);
    
    % --- 1. Dinámica (Momentos -> Aceleración Angular) ---
    % Ecuaciones inferiores de la imagen
    w_x_dot = ((Iy - Iz)/Ix)*w_y*w_z + (1/Ix)*u2;
    w_y_dot = ((Iz - Ix)/Iy)*w_x*w_z + (1/Iy)*u3;
    w_z_dot = ((Ix - Iy)/Iz)*w_x*w_y + (1/Iz)*u4;
    
    % --- 2. Cinemática (Velocidades Cuerpo -> Razon de cambio Ángulos Euler) ---
    % Ecuaciones superiores de la imagen
    % Nota: c(theta) es cos(theta), s(phi) es sin(phi), etc.
    
    % phi_dot (Roll rate)
    phi_dot   = w_x + sin(phi)*(sin(theta)/cos(theta))*w_y + cos(phi)*(sin(theta)/cos(theta))*w_z;
    
    % theta_dot (Pitch rate)
    theta_dot = cos(phi)*w_y - sin(phi)*w_z;
    
    % psi_dot (Yaw rate)
    psi_dot   = (sin(phi)/cos(theta))*w_y + (cos(phi)/cos(theta))*w_z;
    
    % --- 3. Integración de Euler (Discreto) ---
    omega_next = [w_x + dt*w_x_dot; 
                  w_y + dt*w_y_dot; 
                  w_z + dt*w_z_dot];
              
    ang_next   = [phi + dt*phi_dot; 
                  theta + dt*theta_dot; 
                  psi + dt*psi_dot]; 
end