%% =============== METRICAS DE DESEMPEÑO DE LA RED ===========
%{
error = rad2deg(e1_ident_roll);
y_result = ang(1,:);

% 1. Mean Squared Error
mse_val = mean(error.^2);

% 2. Root Mean Squared Error
rmse_val = sqrt(mse_val);

% 3. Mean Absolute Error
mae_val = mean(abs(error));

% 4. Integral del Error Absoluto (IAE) - Útil para rendimiento en el tiempo
iae_val = trapz(t, abs(error));

fprintf('--- Reporte de Identificación ---\n');
fprintf('MSE:  %.6f\n', mse_val);
fprintf('RMSE: %.6f\n', rmse_val);
fprintf('MAE:  %.6f\n', mae_val);

S = stepinfo(y_result, t, 'SettlingTimeThreshold', 0.05);

fprintf('Sobreimpulso: %.2f%%\n', S.Overshoot);
fprintf('Tiempo de asentamiento: %.2f s\n', S.SettlingTime);
fprintf('Tiempo de subida: %.2f s\n', S.RiseTime);
%}

%% ========================================================================
% CONFIGURACIÓN GLOBAL DE GRÁFICAS (FORMATO ARTÍCULO/TESIS)
% =========================================================================
tamano_letra = 14; % <-- Cambia este número al tamaño que te pidan (ej. 12, 14, 16)
grosor_linea = 1.5; % Un poco más grueso ayuda mucho en .eps

% Aplicar tamaño de letra a todo (Ejes, Títulos, Textos y Leyendas)
set(groot, 'defaultAxesFontSize', tamano_letra);
set(groot, 'defaultTextFontSize', tamano_letra);
set(groot, 'defaultLegendFontSize', tamano_letra - 2); % Leyenda un poco más chica

% Cambiar el tipo de letra (Opcional, 'Times New Roman' es estándar)
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');

% Configurar el fondo blanco para que al exportar no salga gris
set(groot, 'defaultFigureColor', 'w');

%% ==================== GRÁFICAS COMPLETAS ====================
% --- 1. Dinámica Traslacional (z, v) ---
figure('Name','Dinámica Traslacional Posiciones');
subplot(3,1,1);
plot(t, ref_total(1,1:length(t)), 'LineWidth', 1.2, 'Color', '#FF0000'); hold on; grid on;
plot(t, x, 'LineWidth', 1.2, 'Color', '#031891');
plot(t, xn, 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle', '--');
legend('Referencia','x (Real)','xn (Red)','Location','best');
ylabel('x (m)'); title('Dinámica X Discreta');

subplot(3,1,2);
plot(t, ref_total(2,1:length(t)), 'LineWidth', 1.2, 'Color', '#FF0000'); hold on; grid on;
plot(t, y, 'LineWidth', 1.2, 'Color', '#031891'); 
plot(t, yn, 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle', '--');
legend('Referencia','y (Real)','yn (Red)','Location','best');
ylabel('y (m)'); title('Dinámica Y Discreta');

subplot(3,1,3);
plot(t, target_z(1,1:length(t)), 'LineWidth', 1.2, 'Color', '#FF0000'); hold on; grid on;
plot(t, z, 'LineWidth', 1.2, 'Color', '#031891'); 
plot(t, zn, 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle', '--');
legend('Referencia','z (Real)','zn (Red)','Location','best');
ylabel('z (m)'); title('Dinámica Z Discreta');
xlabel(' Tiempo (s)')

%subplot(3,1,2);
% --- 2. Dinámica Traslacional Velocidades (z, v) ---
figure('Name','Dinámica Traslacional Velocidades')
subplot(3,1,1);
plot(t, vx, 'LineWidth', 1.2, 'Color', '#031891'); hold on; grid on;
plot(t, vxn, 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle', '--');
legend('vx (Real)','vxn (Red)','Location','best');
ylabel('vx (m/s)'); title('Dinámica X Discreta');

subplot(3,1,2);
plot(t, vy, 'LineWidth', 1.2, 'Color', '#031891'); hold on; grid on;
plot(t, vyn, 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle', '--');
legend('vy (Real)','vyn (Red)','Location','best');
ylabel('vy (m/s)'); title('Dinámica Y Discreta');

subplot(3,1,3);
plot(t, vz, 'LineWidth', 1.2, 'Color', '#031891'); hold on; grid on;
plot(t, vzn, 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle', '--');
legend('vz (Real)','vzn (Red)','Location','best');
ylabel('vz (m/s)'); xlabel('Tiempo (s)'); title('Dinámica Z Discreta');

% --- 4. Dinámica Rotacional (Solo Dron) ---
% A) Ángulos de Euler
figure('Name', 'Dinámica Rotacional: Ángulos');
subplot(3,1,1);
plot(t, ref_roll_rhonn(1,1:length(t)), 'LineWidth', 1.2, 'Color', '#FF0000'); hold on; grid on;
plot(t, ang(1,:), 'LineWidth', 1.2, 'Color', '#031891');
plot(t, ang_nn(1,:), 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle','--');
ylabel('\phi (rad)'); title('Roll (Alabeo)');
legend('Referencia','\phi (Real)','\phi n (Red)','Location','best');

subplot(3,1,2);
plot(t, ref_pitch_rhonn(1,1:length(t)), 'LineWidth', 1.2, 'Color', '#FF0000'); hold on; grid on;
plot(t, ang(2,:), 'LineWidth', 1.2, 'Color', '#031891');
plot(t, ang_nn(2,:), 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle','--');
ylabel('\theta (rad)'); title('Pitch (Cabeceo)');
legend('Referencia','\theta (Real)','\theta n (Red)','Location','best');

subplot(3,1,3);
plot(t, ref_yaw(1,1:length(t)), 'LineWidth', 1.2, 'Color', '#FF0000'); hold on; grid on;
plot(t, ang(3,:), 'LineWidth', 1.2, 'Color', '#031891');
plot(t, ang_nn(3,:), 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle','--');
ylabel('\psi (rad)'); xlabel('Tiempo (s)'); title('Yaw (Guiñada)');
legend('Referencia','\psi (Real)','\psi n (Red)','Location','best');

% B) Velocidades Angulares
figure('Name', 'Dinámica Rotacional: Velocidades Angulares');
subplot(3,1,1);
plot(t, omega(1,:), 'LineWidth', 1.2, 'Color', '#031891'); hold on; grid on;
plot(t, omega_nn(1,:), 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle','--');
ylabel('p (rad/s)'); title('\omega_x (Velocidad Roll)');
legend('wx (Real)','wxn n (Red)','Location','best');

subplot(3,1,2);
plot(t, omega(2,:), 'LineWidth', 1.2, 'Color', '#031891'); hold on; grid on;
plot(t, omega_nn(2,:), 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle','--');
ylabel('q (rad/s)'); title('\omega_y (Velocidad Pitch)');
legend('wy (Real)','wyn n (Red)','Location','best');

subplot(3,1,3);
plot(t, omega(3,:), 'LineWidth', 1.2, 'Color', '#031891'); hold on; grid on;
plot(t, omega_nn(3,:), 'LineWidth', 1.2, 'Color', '#04b304', 'LineStyle','--');
ylabel('r (rad/s)'); xlabel('Tiempo (s)'); title('\omega_z (Velocidad Yaw)');
legend('wz (Real)','wzn n (Red)','Location','best');

% --- 2. Errores y Esfuerzo de Control X---
figure('Name','Errores y Control X');
subplot(3,2,1);
plot(t, e1_x_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error x (m)'); title('Error de Posición');

subplot(3,2,2);
plot(t, e2_x_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error v (m/s)'); title('Error de Velocidad');

subplot(3,2,3);
plot(t, e1_ident_x_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error x (m)'); title('Error de Identificación (Posición)');

subplot(3,2,4);
plot(t, e2_ident_x_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error v (m/s)'); title('Error de Identificación (Velocidad)');

subplot(3,2,5);
plot(t, u_neural_rot(1,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar(Vertical)');

subplot(3,2,6);
plot(t, U(1,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado(Vertical)');

% --- 2. Errores y Esfuerzo de Control Y---
figure('Name','Errores y Control Y');
subplot(3,2,1);
plot(t, e1_y_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error y (m)'); title('Error de Posición');

subplot(3,2,2);
plot(t, e2_y_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error v (m/s)'); title('Error de Velocidad');

subplot(3,2,3);
plot(t, e1_ident_y_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error y (m)'); title('Error de Identificación (Posición)');

subplot(3,2,4);
plot(t, e2_ident_y_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error v (m/s)'); title('Error de Identificación (Velocidad)');

subplot(3,2,5);
plot(t, u_neural_rot(1,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar(Vertical)');

subplot(3,2,6);
plot(t, U(1,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado(Vertical)');

% --- 2. Errores y Esfuerzo de Control ALTURA---
figure('Name','Errores y Control Z (Altura)');
subplot(3,2,1);
plot(t, e1_z_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error z (m)'); title('Error de Posición');

subplot(3,2,2);
plot(t, e2_z_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error v (m/s)'); title('Error de Velocidad');

subplot(3,2,3);
plot(t, e1_ident_z_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error z (m)'); title('Error de Identificación (Posición)');

subplot(3,2,4);
plot(t, e2_ident_z_dynamic, 'LineWidth', 1.2); grid on;
ylabel('Error v (m/s)'); title('Error de Identificación (Velocidad)');

subplot(3,2,5);
plot(t, u_neural_rot(1,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar(Vertical)');

subplot(3,2,6);
plot(t, U(1,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado(Vertical)');

% --- 5a. Errores y Esfuerzo de Control ROLL---
figure('Name','Errores y Control ROLL');
subplot(3,2,1);
plot(t, rad2deg(e1_roll), 'LineWidth', 1.2); grid on;
ylabel('Error phi (°)'); title('Error de Posición');

subplot(3,2,2);
plot(t, rad2deg(e2_roll), 'LineWidth', 1.2); grid on;
ylabel('Error v_phi (°/s)'); title('Error de Velocidad');

subplot(3,2,3);
plot(t, rad2deg(e1_ident_roll), 'LineWidth', 1.2); grid on;
ylabel('Error phi (°)'); title('Error de Identificación (Posición)');

subplot(3,2,4);
plot(t, rad2deg(e2_ident_roll), 'LineWidth', 1.2); grid on;
ylabel('Error v_phi (°/s)'); title('Error de Identificación (Velocidad)');

subplot(3,2,5);
plot(t, u_neural_rot(2,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar(Roll)');

subplot(3,2,6);
plot(t, U(2,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado(Roll)');

% --- 5b. Errores y Esfuerzo de Control PITCH---
figure('Name','Errores y Control PITCH');
subplot(3,2,1);
plot(t, rad2deg(e1_pitch), 'LineWidth', 1.2); grid on;
ylabel('Error theta (°)'); title('Error de Posición');

subplot(3,2,2);
plot(t, rad2deg(e2_pitch), 'LineWidth', 1.2); grid on;
ylabel('Error v_theta (°/s)'); title('Error de Velocidad');

subplot(3,2,3);
plot(t, rad2deg(e1_ident_pitch), 'LineWidth', 1.2); grid on;
ylabel('Error theta (°)'); title('Error de Identificación (Posición)');

subplot(3,2,4);
plot(t, rad2deg(e2_ident_pitch), 'LineWidth', 1.2); grid on;
ylabel('Error v_theta (°/s)'); title('Error de Identificación (Velocidad)');

subplot(3,2,5);
plot(t, u_neural_rot(3,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar(Pitch)');

subplot(3,2,6);
plot(t, U(3,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado(Pitch)');

% --- 5c. Errores y Esfuerzo de Control YAW---
figure('Name','Errores y Control YAW');
subplot(3,2,1);
plot(t, e1_yaw, 'LineWidth', 1.2); grid on;
ylabel('Error psi (rad)'); title('Error de Posición');

subplot(3,2,2);
plot(t, e2_yaw, 'LineWidth', 1.2); grid on;
ylabel('Error v_psi (rad/s)'); title('Error de Velocidad');

subplot(3,2,3);
plot(t, rad2deg(e1_ident_yaw), 'LineWidth', 1.2); grid on;
ylabel('Error psi (°)'); title('Error de Identificación (Posición)');

subplot(3,2,4);
plot(t, rad2deg(e2_ident_yaw), 'LineWidth', 1.2); grid on;
ylabel('Error v_psi (°/s)'); title('Error de Identificación (Velocidad)');

subplot(3,2,5);
plot(t, u_neural_rot(4,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar(Yaw)');

subplot(3,2,6);
plot(t, U(4,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado(Yaw)');

figure('Name','Velocidad de los motores')
subplot(2,2,1)
plot(t, omega_motors(1,:),'LineWidth', 1.2); grid on;
ylabel('RPM'); xlabel('Tiempo (s)'); title('W1');

subplot(2,2,2)
plot(t, omega_motors(2,:),'LineWidth', 1.2); grid on;
ylabel('RPM'); xlabel('Tiempo (s)'); title('W2');

subplot(2,2,3)
plot(t, omega_motors(3,:),'LineWidth', 1.2); grid on;
ylabel('RPM'); xlabel('Tiempo (s)'); title('W3');

subplot(2,2,4)
plot(t, omega_motors(4,:),'LineWidth', 1.2); grid on;
ylabel('RPM'); xlabel('Tiempo (s)'); title('W4');

% --- 6. Referencias del PID vs Control Rhonn---
figure('Name','Referencias del PID vs Control RHONN')
subplot(2,1,1);
plot(t_ref(1:end-1),ref_roll, 'LineWidth',2); hold on; grid on;
plot(t_ref(1:end-1),ref_roll_rhonn, 'LineWidth', 1.2, 'Color', 'r');
legend('Referencia PID','Referencia RHONN','Location','best');
title('Roll')
subplot(2,1,2);
plot(t_ref(1:end-1),ref_pitch, 'LineWidth',2); hold on; grid on;
plot(t_ref(1:end-1),ref_pitch_rhonn, 'LineWidth', 1.2, 'Color', 'r');
legend('Referencia PID','Referencia RHONN','Location','best');
title('Pitch')

% Graficar las referencias de Roll y Pitch dadas por el PID
figure('Name','Referencias del PID')
subplot(2,2,1);
plot(t_ref(1:end-2),ux_des, 'LineWidth',2);
title('Roll')
ylabel('Rad');
subplot(2,2,2);
plot(t_ref(1:end-2),e1_x_dynamic, 'LineWidth',2);
xlabel('Tiempo');
title('e1 X');
subplot(2,2,3);
plot(t_ref(1:end-2),ux_des, 'LineWidth',2);
title('Pitch')
ylabel('Rad');
subplot(2,2,4);
plot(t_ref(1:end-2),e1_y_dynamic, 'LineWidth',2);
title('e1 Y')
xlabel('Tiempo')

% =========================================================================
% EXPORTACIÓN AUTOMÁTICA DE TODAS LAS GRÁFICAS A FORMATO .EPS
% =========================================================================
fprintf('\nGuardando gráficas en formato .eps...\n');

% --- Definir la carpeta de destino ---
% Opcional 1: Ruta relativa (Crea la carpeta donde tienes tu código actual)
% carpeta_destino = 'Graficas_Exportadas';

% Opcional 2: Ruta absoluta (Descomenta y cambia esto si quieres una ruta específica en tu PC)
carpeta_destino = 'D:\Pictures\Graficas_Tesis';

% Si la carpeta no existe, MATLAB la crea automáticamente para evitar errores
if ~exist(carpeta_destino, 'dir')
    mkdir(carpeta_destino);
end
% ---------------------------------------------

% 1. Encontrar todas las figuras abiertas
lista_figuras = findobj(allchild(0), 'flat', 'Type', 'figure');

% 2. Bucle para guardar cada una
for i = 1:length(lista_figuras)
    fig = lista_figuras(i);
    
    % Obtener el nombre de la figura
    nombre_figura = get(fig, 'Name');
    if isempty(nombre_figura)
        nombre_figura = sprintf('Figura_%d', fig.Number);
    end
    
    % Limpiar el nombre para que sea un archivo válido
    nombre_archivo = strrep(nombre_figura, ' ', '_');
    nombre_archivo = strrep(nombre_archivo, ':', '');
    nombre_archivo = strrep(nombre_archivo, '(', '');
    nombre_archivo = strrep(nombre_archivo, ')', '');
    nombre_archivo = strrep(nombre_archivo, '/', '_');
    
    % Ajustar proporciones
    set(fig, 'PaperPositionMode', 'auto');
    
    % --- NUEVO: Construir la ruta completa y guardar ---
    % fullfile une la carpeta y el archivo con la diagonal correcta (\ o /)
    ruta_completa = fullfile(carpeta_destino, [nombre_archivo, '.eps']);
    
    % Guardar la figura en la ruta especificada
    print(fig, ruta_completa, '-depsc', '-r300');
    
    fprintf(' Guardada: %s\n', ruta_completa);
end
fprintf('¡Exportación terminada! Revisa la carpeta: %s\n', carpeta_destino);

% Restablecer las configuraciones globales
reset(groot);

%{
% Pesos Sinapticos Z
figure('Name','EFK Weights Z')
subplot(2,3,1);
plot(t, w1_z_dynamic(1,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w11 (Posición)');

subplot(2,3,2);
plot(t, w1_z_dynamic(2,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w12 (Posición)');

% subplot(2,3,3);
% plot(t, w1_z_dynamic(3,:), 'LineWidth', 1.2); grid on;
% ylabel('Value'); title('w12 (Posición)');

subplot(2,3,4);
plot(t, w2_z_dynamic(1,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w21 (Velocidad)');

subplot(2,3,5);
plot(t, w2_z_dynamic(2,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w22 (Velocidad)'); xlabel('Tiempo (s)');

subplot(2,3,6);
plot(t, w2_z_dynamic(3,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w23 (Bias Velocidad)'); xlabel('Tiempo (s)');

% Pesos Sinapticos ROLL
figure('Name','EFK Weights ROLL')
subplot(2,3,1);
plot(t, w1_roll(1,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w11 (Posición)');

subplot(2,3,2);
plot(t, w1_roll(2,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w12 (Posición)');

subplot(2,3,3);
plot(t, w1_roll(3,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w12 (Posición)');

subplot(2,3,4);
plot(t, w2_roll(1,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w21 (Velocidad)');

subplot(2,3,5);
plot(t, w2_roll(2,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w22 (Velocidad)'); xlabel('Tiempo (s)');

subplot(2,3,6);
plot(t, w2_roll(3,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w23 (Bias Velocidad)'); xlabel('Tiempo (s)');

% Pesos Sinapticos PITCH
figure('Name','EFK Weights PITCH')
subplot(2,3,1);
plot(t, w1_pitch(1,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w11 (Posición)');

subplot(2,3,2);
plot(t, w1_pitch(2,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w12 (Posición)');

subplot(2,3,3);
plot(t, w1_pitch(3,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w12 (Posición)');

subplot(2,3,4);
plot(t, w2_pitch(1,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w21 (Velocidad)');

subplot(2,3,5);
plot(t, w2_pitch(2,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w22 (Velocidad)'); xlabel('Tiempo (s)');

subplot(2,3,6);
plot(t, w2_pitch(3,:), 'LineWidth', 1.2); grid on;
ylabel('Value'); title('w23 (Bias Velocidad)'); xlabel('Tiempo (s)');
%}