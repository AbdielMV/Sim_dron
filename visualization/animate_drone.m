%% ==================== ANIMACIÓN 3D Y EXPORTACIÓN A VIDEO ====================
fprintf('Preparando simulación 3D usando el método de hgtransform...\n');
step_anim = 50; 
fps = 1 / (dt * step_anim);
video_filename = 'Simulacion_RHONN.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = fps;
v.Quality = 100;
open(v);

% =========================================================
% 1. Preparar Figura y Ejes
% =========================================================
fig = figure('Name', 'Simulación 3D del UAV', 'Color', 'w', 'WindowState', 'maximized');
ax = axes('Parent', fig); 
hold(ax, 'on'); grid(ax, 'on'); 

% --- VISTA ISOMÉTRICA CLÁSICA ---
view(ax, [45, 30]); % Ángulo isométrico
camproj(ax, 'orthographic'); % Proyección isométrica real (sin fuga)
% --------------------------------

% Límites fijos para ver toda la trayectoria desde el inicio
margen = 2.0;
% Aseguramos que los límites contemplen tanto la trayectoria real como la referencia (usando ref_total)
xlim(ax, [min([x, ref_total(1,:)])-margen, max([x, ref_total(1,:)])+margen]);
ylim(ax, [min([y, ref_total(2,:)])-margen, max([y, ref_total(2,:)])+margen]);
zlim(ax, [min([z, ref_total(3,:)])-0.5, max([z, ref_total(3,:)])+margen]);
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');

% Configuraciones críticas para isometría
axis(ax, 'equal'); 
axis(ax, 'vis3d');

% --- GRAFICAR TRAYECTORIAS ---
% 1. Dibujar trayectoria de referencia (Línea punteada roja usando ref_total)
plot3(ax, ref_total(1,:), ref_total(2,:), ref_total(3,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Referencia');

% 2. Dibujar trayectoria del controlador/dron (Línea sólida azul)
plot3(ax, x, y, z, 'b-', 'LineWidth', 1.5, 'DisplayName', 'UAV Real');

% Activar la leyenda
legend(ax, 'show', 'Location', 'best');

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
