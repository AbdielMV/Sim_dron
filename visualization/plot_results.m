% =========================================================================
% SCRIPT DE VISUALIZACIÓN DE RESULTADOS (6 DoF) - UAV MULTI-RATE
% =========================================================================
disp('Generando gráficas...');

% 1. Ajuste de vectores de referencia y neuronales para coincidir con el vector de tiempo 't'
len_t = length(t);

ref_x_plot = target_x(1:len_t);
ref_y_plot = target_y(1:len_t);
ref_z_plot = target_z(1:len_t);

ref_roll_plot  = ref_roll_rhonn(1:len_t);
ref_pitch_plot = ref_pitch_rhonn(1:len_t);
ref_yaw_plot   = ref_yaw(1:len_t);

xn_plot = xn(1:len_t);
yn_plot = yn(1:len_t);
zn_plot = zn(1:len_t);
vxn_plot = vxn(1:len_t);
vyn_plot = vyn(1:len_t);
vzn_plot = vzn(1:len_t);

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

% --- 1. Dinámica Traslacional (Posiciones) ---
figure('Name','Dinámica Traslacional Posiciones');
subplot(3,1,1);
plot(t, ref_x_plot, 'LineWidth', grosor_linea, 'Color', '#FF0000'); hold on; grid on;
plot(t, x, 'LineWidth', grosor_linea, 'Color', '#031891');
plot(t, xn_plot, 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle', '--');
legend('Referencia','x (Real)','xn (Red)','Location','best');
ylabel('x (m)'); title('Dinámica X Discreta');

subplot(3,1,2);
plot(t, ref_y_plot, 'LineWidth', grosor_linea, 'Color', '#FF0000'); hold on; grid on;
plot(t, y, 'LineWidth', grosor_linea, 'Color', '#031891'); 
plot(t, yn_plot, 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle', '--');
legend('Referencia','y (Real)','yn (Red)','Location','best');
ylabel('y (m)'); title('Dinámica Y Discreta');

subplot(3,1,3);
plot(t, ref_z_plot, 'LineWidth', grosor_linea, 'Color', '#FF0000'); hold on; grid on;
plot(t, z, 'LineWidth', grosor_linea, 'Color', '#031891'); 
plot(t, zn_plot, 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle', '--');
legend('Referencia','z (Real)','zn (Red)','Location','best');
ylabel('z (m)'); xlabel(' Tiempo (s)'); title('Dinámica Z Discreta');

% --- 2. Dinámica Traslacional Velocidades ---
figure('Name','Dinámica Traslacional Velocidades')
subplot(3,1,1);
plot(t, vx, 'LineWidth', grosor_linea, 'Color', '#031891'); hold on; grid on;
plot(t, vxn_plot, 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle', '--');
legend('vx (Real)','vxn (Red)','Location','best');
ylabel('vx (m/s)'); title('Velocidad X Discreta');

subplot(3,1,2);
plot(t, vy, 'LineWidth', grosor_linea, 'Color', '#031891'); hold on; grid on;
plot(t, vyn_plot, 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle', '--');
legend('vy (Real)','vyn (Red)','Location','best');
ylabel('vy (m/s)'); title('Velocidad Y Discreta');

subplot(3,1,3);
plot(t, vz, 'LineWidth', grosor_linea, 'Color', '#031891'); hold on; grid on;
plot(t, vzn_plot, 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle', '--');
legend('vz (Real)','vzn (Red)','Location','best');
ylabel('vz (m/s)'); xlabel('Tiempo (s)'); title('Velocidad Z Discreta');

% --- 3. Dinámica Rotacional (Ángulos) ---
figure('Name', 'Dinámica Rotacional: Ángulos');
subplot(3,1,1);
plot(t, ref_roll_plot, 'LineWidth', grosor_linea, 'Color', '#FF0000'); hold on; grid on;
plot(t, ang(1,:), 'LineWidth', grosor_linea, 'Color', '#031891');
plot(t, ang_nn(1,1:len_t), 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle','--');
ylabel('\phi (rad)'); title('Roll (Alabeo)');
legend('Referencia','\phi (Real)','\phi n (Red)','Location','best');

subplot(3,1,2);
plot(t, ref_pitch_plot, 'LineWidth', grosor_linea, 'Color', '#FF0000'); hold on; grid on;
plot(t, ang(2,:), 'LineWidth', grosor_linea, 'Color', '#031891');
plot(t, ang_nn(2,1:len_t), 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle','--');
ylabel('\theta (rad)'); title('Pitch (Cabeceo)');
legend('Referencia','\theta (Real)','\theta n (Red)','Location','best');

subplot(3,1,3);
plot(t, ref_yaw_plot, 'LineWidth', grosor_linea, 'Color', '#FF0000'); hold on; grid on;
plot(t, ang(3,:), 'LineWidth', grosor_linea, 'Color', '#031891');
plot(t, ang_nn(3,1:len_t), 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle','--');
ylabel('\psi (rad)'); xlabel('Tiempo (s)'); title('Yaw (Guiñada)');
legend('Referencia','\psi (Real)','\psi n (Red)','Location','best');

% --- 4. Dinámica Rotacional (Velocidades Angulares) ---
figure('Name', 'Dinámica Rotacional: Velocidades Angulares');
subplot(3,1,1);
plot(t, omega(1,:), 'LineWidth', grosor_linea, 'Color', '#031891'); hold on; grid on;
plot(t, omega_nn(1,1:len_t), 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle','--');
ylabel('p (rad/s)'); title('\omega_x (Velocidad Roll)');
legend('\omega_x (Real)','\omega_{xn} (Red)','Location','best');

subplot(3,1,2);
plot(t, omega(2,:), 'LineWidth', grosor_linea, 'Color', '#031891'); hold on; grid on;
plot(t, omega_nn(2,1:len_t), 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle','--');
ylabel('q (rad/s)'); title('\omega_y (Velocidad Pitch)');
legend('\omega_y (Real)','\omega_{yn} (Red)','Location','best');

subplot(3,1,3);
plot(t, omega(3,:), 'LineWidth', grosor_linea, 'Color', '#031891'); hold on; grid on;
plot(t, omega_nn(3,1:len_t), 'LineWidth', grosor_linea, 'Color', '#04b304', 'LineStyle','--');
ylabel('r (rad/s)'); xlabel('Tiempo (s)'); title('\omega_z (Velocidad Yaw)');
legend('\omega_z (Real)','\omega_{zn} (Red)','Location','best');

% --- 5a. Errores y Esfuerzo de Control X ---
figure('Name','Errores y Control X');
subplot(3,2,1); plot(t, e1_x_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error x (m)'); title('Error de Posición');
subplot(3,2,2); plot(t, e2_x_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error v (m/s)'); title('Error de Velocidad');
subplot(3,2,3); plot(t, e1_ident_x_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error x (m)'); title('Error de Ident. (Pos)');
subplot(3,2,4); plot(t, e2_ident_x_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error v (m/s)'); title('Error de Ident. (Vel)');
subplot(3,2,5); plot(t, u_neural_rot(1,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar');
subplot(3,2,6); plot(t, U(1,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado');

% --- 5b. Errores y Esfuerzo de Control Y ---
figure('Name','Errores y Control Y');
subplot(3,2,1); plot(t, e1_y_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error y (m)'); title('Error de Posición');
subplot(3,2,2); plot(t, e2_y_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error v (m/s)'); title('Error de Velocidad');
subplot(3,2,3); plot(t, e1_ident_y_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error y (m)'); title('Error de Ident. (Pos)');
subplot(3,2,4); plot(t, e2_ident_y_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error v (m/s)'); title('Error de Ident. (Vel)');
subplot(3,2,5); plot(t, u_neural_rot(1,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar');
subplot(3,2,6); plot(t, U(1,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado');

% --- 5c. Errores y Esfuerzo de Control ALTURA (Z) ---
figure('Name','Errores y Control Z (Altura)');
subplot(3,2,1); plot(t, e1_z_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error z (m)'); title('Error de Posición');
subplot(3,2,2); plot(t, e2_z_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error v (m/s)'); title('Error de Velocidad');
subplot(3,2,3); plot(t, e1_ident_z_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error z (m)'); title('Error de Ident. (Pos)');
subplot(3,2,4); plot(t, e2_ident_z_dynamic(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error v (m/s)'); title('Error de Ident. (Vel)');
subplot(3,2,5); plot(t, u_neural_rot(1,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar');
subplot(3,2,6); plot(t, U(1,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado');

% --- 6a. Errores y Esfuerzo de Control ROLL ---
figure('Name','Errores y Control ROLL');
subplot(3,2,1); plot(t, rad2deg(e1_roll(1:len_t)), 'LineWidth', grosor_linea); grid on; ylabel('Error phi (°)'); title('Error de Posición');
subplot(3,2,2); plot(t, rad2deg(e2_roll(1:len_t)), 'LineWidth', grosor_linea); grid on; ylabel('Error v_phi (°/s)'); title('Error de Velocidad');
subplot(3,2,3); plot(t, rad2deg(e1_ident_roll(1:len_t)), 'LineWidth', grosor_linea); grid on; ylabel('Error phi (°)'); title('Error de Ident. (Pos)');
subplot(3,2,4); plot(t, rad2deg(e2_ident_roll(1:len_t)), 'LineWidth', grosor_linea); grid on; ylabel('Error v_phi (°/s)'); title('Error de Ident. (Vel)');
subplot(3,2,5); plot(t, u_neural_rot(2,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar(Roll)');
subplot(3,2,6); plot(t, U(2,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado(Roll)');

% --- 6b. Errores y Esfuerzo de Control PITCH ---
figure('Name','Errores y Control PITCH');
subplot(3,2,1); plot(t, rad2deg(e1_pitch(1:len_t)), 'LineWidth', grosor_linea); grid on; ylabel('Error theta (°)'); title('Error de Posición');
subplot(3,2,2); plot(t, rad2deg(e2_pitch(1:len_t)), 'LineWidth', grosor_linea); grid on; ylabel('Error v_theta (°/s)'); title('Error de Velocidad');
subplot(3,2,3); plot(t, rad2deg(e1_ident_pitch(1:len_t)), 'LineWidth', grosor_linea); grid on; ylabel('Error theta (°)'); title('Error de Ident. (Pos)');
subplot(3,2,4); plot(t, rad2deg(e2_ident_pitch(1:len_t)), 'LineWidth', grosor_linea); grid on; ylabel('Error v_theta (°/s)'); title('Error de Ident. (Vel)');
subplot(3,2,5); plot(t, u_neural_rot(3,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar(Pitch)');
subplot(3,2,6); plot(t, U(3,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado(Pitch)');

% --- 6c. Errores y Esfuerzo de Control YAW ---
figure('Name','Errores y Control YAW');
subplot(3,2,1); plot(t, e1_yaw(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error psi (rad)'); title('Error de Posición');
subplot(3,2,2); plot(t, e2_yaw(1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('Error v_psi (rad/s)'); title('Error de Velocidad');
subplot(3,2,3); plot(t, rad2deg(e1_ident_yaw(1:len_t)), 'LineWidth', grosor_linea); grid on; ylabel('Error psi (°)'); title('Error de Ident. (Pos)');
subplot(3,2,4); plot(t, rad2deg(e2_ident_yaw(1:len_t)), 'LineWidth', grosor_linea); grid on; ylabel('Error v_psi (°/s)'); title('Error de Ident. (Vel)');
subplot(3,2,5); plot(t, u_neural_rot(4,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Sin Saturar(Yaw)');
subplot(3,2,6); plot(t, U(4,1:len_t), 'LineWidth', grosor_linea); grid on; ylabel('u (N)'); xlabel('Tiempo (s)'); title('u Saturado(Yaw)');

% --- 7. Velocidad de los Motores ---
RPM_motors = omega_motors * (60 / (2*pi));
figure('Name','Velocidad de los motores')
subplot(2,2,1); plot(t, RPM_motors(1,1:len_t),'LineWidth', grosor_linea); grid on; ylabel('RPM'); xlabel('Tiempo (s)'); title('W1');
subplot(2,2,2); plot(t, RPM_motors(2,1:len_t),'LineWidth', grosor_linea); grid on; ylabel('RPM'); xlabel('Tiempo (s)'); title('W2');
subplot(2,2,3); plot(t, RPM_motors(3,1:len_t),'LineWidth', grosor_linea); grid on; ylabel('RPM'); xlabel('Tiempo (s)'); title('W3');
subplot(2,2,4); plot(t, RPM_motors(4,1:len_t),'LineWidth', grosor_linea); grid on; ylabel('RPM'); xlabel('Tiempo (s)'); title('W4');

%% =========================================================================
% EXPORTACIÓN AUTOMÁTICA DE TODAS LAS GRÁFICAS A FORMATO .EPS
% =========================================================================
fprintf('\nGuardando gráficas en formato .eps...\n');

% --- Definir la carpeta de destino ---
% Opcional 1: Ruta relativa
% carpeta_destino = 'Graficas_Exportadas';

% Opcional 2: Ruta absoluta
carpeta_destino = 'D:\Pictures\Graficas_Tesis';

% Si la carpeta no existe, MATLAB la crea
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
    
    % --- Construir la ruta completa y guardar ---
    ruta_completa = fullfile(carpeta_destino, [nombre_archivo, '.eps']);
    
    % Guardar la figura en la ruta especificada
    print(fig, ruta_completa, '-depsc', '-r300');
    
    fprintf(' Guardada: %s\n', ruta_completa);
end
fprintf('¡Exportación terminada! Revisa la carpeta: %s\n', carpeta_destino);

% Restablecer las configuraciones globales
reset(groot);

%% =========================================================================
% PESOS SINÁPTICOS (OPCIONAL)
% =========================================================================
%{
% Pesos Sinapticos Z
figure('Name','EFK Weights Z')
subplot(2,3,1); plot(t, w1_z_dynamic(1,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w11 (Posición)');
subplot(2,3,2); plot(t, w1_z_dynamic(2,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w12 (Posición)');
subplot(2,3,4); plot(t, w2_z_dynamic(1,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w21 (Velocidad)');
subplot(2,3,5); plot(t, w2_z_dynamic(2,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w22 (Velocidad)'); xlabel('Tiempo (s)');
subplot(2,3,6); plot(t, w2_z_dynamic(3,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w23 (Bias Velocidad)'); xlabel('Tiempo (s)');

% Pesos Sinapticos ROLL
figure('Name','EFK Weights ROLL')
subplot(2,3,1); plot(t, w1_roll(1,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w11 (Posición)');
subplot(2,3,2); plot(t, w1_roll(2,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w12 (Posición)');
subplot(2,3,3); plot(t, w1_roll(3,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w12 (Posición)');
subplot(2,3,4); plot(t, w2_roll(1,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w21 (Velocidad)');
subplot(2,3,5); plot(t, w2_roll(2,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w22 (Velocidad)'); xlabel('Tiempo (s)');
subplot(2,3,6); plot(t, w2_roll(3,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w23 (Bias Velocidad)'); xlabel('Tiempo (s)');

% Pesos Sinapticos PITCH
figure('Name','EFK Weights PITCH')
subplot(2,3,1); plot(t, w1_pitch(1,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w11 (Posición)');
subplot(2,3,2); plot(t, w1_pitch(2,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w12 (Posición)');
subplot(2,3,3); plot(t, w1_pitch(3,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w12 (Posición)');
subplot(2,3,4); plot(t, w2_pitch(1,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w21 (Velocidad)');
subplot(2,3,5); plot(t, w2_pitch(2,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w22 (Velocidad)'); xlabel('Tiempo (s)');
subplot(2,3,6); plot(t, w2_pitch(3,:), 'LineWidth', 1.2); grid on; ylabel('Value'); title('w23 (Bias Velocidad)'); xlabel('Tiempo (s)');
%}