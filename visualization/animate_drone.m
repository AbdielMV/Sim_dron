%% ==================== ANIMACIÓN 3D Y EXPORTACIÓN A VIDEO ====================
fprintf('Preparando simulación 3D usando el método de hgtransform...\n');
step_anim = 50; 
fps = 1 / (dt * step_anim);
video_filename = 'Simulacion_RHONN.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fps;
v.Quality = 100;
open(v);

% 1. Preparar Figura y Ejes
fig = figure('Name', 'Simulación 3D del UAV', 'Color', 'w', 'WindowState', 'maximized');
ax = axes('Parent', fig); 
hold(ax, 'on'); grid(ax, 'on'); 

% --- VISTA ISOMÉTRICA CLÁSICA ---
view(ax, [45, 30]); % Ángulo isométrico
camproj(ax, 'orthographic'); % Proyección isométrica real (sin fuga)
% --------------------------------

axis(ax, 'equal'); 
axis(ax, 'vis3d');

plot3(ax, x, y, z, 'b-', 'LineWidth', 1.5);

% Límites fijos para ver toda la trayectoria desde el inicio
margen = 2.0;
xlim(ax, [min(x)-margen, max(x)+margen]);
ylim(ax, [min(y)-margen, max(y)+margen]);
zlim(ax, [min(z)-0.5, max(z)+margen]);
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');

% Configuraciones críticas para isometría
axis(ax, 'equal'); 
axis(ax, 'vis3d');

% Dibujar trayectoria del controlador
plot3(ax, x, y, z, 'b-', 'LineWidth', 1.5);
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');

% 2. Crear los grupos de transformación jerárquica
animeDrone          = hgtransform('Parent', ax);
animeDroneBody      = hgtransform('Parent', animeDrone);
animeDronePropeller = hgtransform('Parent', animeDrone);

% 3. Cargar y Simplificar STLs
try
    % Cuerpo
    fv_body = stlread('DroneBody.STL');
    if isprop(fv_body, 'Points'), f_b = fv_body.ConnectivityList; v_b = fv_body.Points;
    else, f_b = fv_body.faces; v_b = fv_body.vertices; end
    [f_b, v_b] = reducepatch(f_b, v_b, 0.1);
    
    patch('Faces', f_b, 'Vertices', v_b, 'FaceColor', [0.3 0.3 0.3], ...
          'EdgeColor', 'none', 'Parent', animeDroneBody);
          
    % Propelas (El archivo de tu compañero ya tiene las 4 integradas)
    fv_prop = stlread('DronePropeller.STL');
    if isprop(fv_prop, 'Points'), f_p = fv_prop.ConnectivityList; v_p = fv_prop.Points;
    else, f_p = fv_prop.faces; v_p = fv_prop.vertices; end
    [f_p, v_p] = reducepatch(f_p, v_p, 0.1);
    
    patch('Faces', f_p, 'Vertices', v_p, 'FaceColor', [0.8 0.1 0.1], ...
          'EdgeColor', 'none', 'Parent', animeDronePropeller);
catch
    error('Archivos STL no encontrados. Verifica los nombres y la ruta.');
end

camlight('headlight'); material('dull');

% ==========================================================
% 4. ESCALA VISUAL (AQUÍ CONTROLAS EL TAMAÑO EN PANTALLA)
% ==========================================================
factor_visual = 2; % Si lo quieres más grande, ponle 20 o 30.

stl_body = [-0.25, -0.15, -0.30] * factor_visual;
stl_propeller = [-0.345, 0.125, -0.365] * factor_visual;
escala_final = 0.00125 * factor_visual;

H_Body = makehgtform('xrotate', pi/2, 'yrotate', pi/2, 'translate', stl_body, 'scale', escala_final);
set(animeDroneBody, 'Matrix', H_Body);

H_Propeller = makehgtform('xrotate', pi/2, 'yrotate', pi/2, 'translate', stl_propeller, 'scale', escala_final);
set(animeDronePropeller, 'Matrix', H_Propeller);

% 5. Bucle de Animación
for k = 1:step_anim:N
    pos_k = [x(k), y(k), z(k)];
    phi_k = ang(1, k); theta_k = ang(2, k); psi_k = ang(3, k); 
    
    % Matriz de transformación general del Dron (Padre)
    H = makehgtform('translate', pos_k, 'zrotate', psi_k, 'yrotate', theta_k, 'xrotate', phi_k);
    set(animeDrone, 'Matrix', H);
    
    % --- CÁMARA DE SEGUIMIENTO ---
    % camtarget(ax, pos_k); % Apuntar siempre al dron
    % offset_camara = [-2.0, -2.0, 1.5]; % Posición de la cámara (Atrás, Izquierda, Arriba)
    % campos(ax, pos_k + offset_camara);
    % camva(ax, 25); % Zoom
    
    title(ax, sprintf('Simulación UAV | Tiempo: %.2f s', t(k)));
    drawnow;
    writeVideo(v, getframe(fig));
end
close(v);
fprintf('¡Simulación terminada! Video guardado como %s\n', video_filename);

%{
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
%}