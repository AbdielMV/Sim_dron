%% =============== METRICAS DE DESEMPEÑO DE LA RED ===========
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

%% ==================== GRÁFICAS COMPLETAS ====================
% --- 1. Dinámica Traslacional (z, v) ---
figure('Name','Dinámica Traslacional Posiciones');
subplot(3,1,1);
plot(t, x, 'LineWidth', 1.2); hold on; grid on;
plot(t, ref_total(1,1:length(t)), 'LineWidth', 1.2);
plot(t, xn, 'LineWidth', 1.2, 'LineStyle', '--');
legend('x (Real)','Referencia','xn (Red)','Location','best');
ylabel('x (m)'); title('Dinámica X Discreta');

subplot(3,1,2);
plot(t, y, 'LineWidth', 1.2); hold on; grid on;
plot(t, ref_total(2,1:length(t)), 'LineWidth', 1.2);
plot(t, yn, 'LineWidth', 1.2, 'LineStyle', '--');
legend('y (Real)','Referencia','yn (Red)','Location','best');
ylabel('y (m)'); title('Dinámica Y Discreta');

subplot(3,1,3);
plot(t, z, 'LineWidth', 1.2); hold on; grid on;
plot(t, target_z(1,1:length(t)), 'LineWidth', 1.2);
plot(t, zn, 'LineWidth', 1.2, 'LineStyle', '--');
legend('z (Real)','Referencia','zn (Red)','Location','best');
ylabel('z (m)'); title('Dinámica Z Discreta');
xlabel(' Tiempo (s)')

subplot(3,1,2);

figure('Name','Dinámica Traslacional Velocidades')
subplot(3,1,1);
plot(t, vx, 'LineWidth', 1.2); hold on; grid on;
plot(t, vxn, 'LineWidth', 1.2, 'LineStyle', '--');
legend('v (Real)','vn (Red)','Location','best');
ylabel('v (m/s)'); xlabel('Tiempo (s)');

subplot(3,1,2);
plot(t, vy, 'LineWidth', 1.2); hold on; grid on;
plot(t, vyn, 'LineWidth', 1.2, 'LineStyle', '--');
legend('v (Real)','vn (Red)','Location','best');
ylabel('v (m/s)'); xlabel('Tiempo (s)');

subplot(3,1,3);
plot(t, vz, 'LineWidth', 1.2); hold on; grid on;
plot(t, vzn, 'LineWidth', 1.2, 'LineStyle', '--');
legend('v (Real)','vn (Red)','Location','best');
ylabel('v (m/s)'); xlabel('Tiempo (s)');
xlabel('Tiempo (s)');

% --- 4. Dinámica Rotacional (Solo Dron) ---
% A) Ángulos de Euler
figure('Name', 'Dinámica Rotacional: Ángulos');
subplot(3,1,1);
plot(t, ang(1,:), 'LineWidth', 1.2, 'Color', 'b'); hold on; grid on;
plot(t, ang_nn(1,:), 'LineWidth', 1.2, 'Color', 'g', 'LineStyle','--');
plot(t, ref_roll_rhonn(1,1:length(t)), 'LineWidth', 1.2, 'Color', 'r');
ylabel('\phi (rad)'); title('Roll (Alabeo)');
legend('\phi (Real)','\phi n (Red)','Referencia','Location','best');

subplot(3,1,2);
plot(t, ang(2,:), 'LineWidth', 1.2, 'Color', 'g'); hold on; grid on;
plot(t, ang_nn(2,:), 'LineWidth', 1.2, 'Color', 'y', 'LineStyle','--');
plot(t, ref_pitch_rhonn(1,1:length(t)), 'LineWidth', 1.2, 'Color', 'r');
ylabel('\theta (rad)'); title('Pitch (Cabeceo)');
legend('\theta (Real)','\theta n (Red)','Referencia','Location','best');

subplot(3,1,3);
plot(t, ang(3,:), 'LineWidth', 1.2, 'Color', 'b'); hold on; grid on;
plot(t, ang_nn(3,:), 'LineWidth', 1.2, 'Color', 'g', 'LineStyle','--');
plot(t, ref_yaw(1,1:length(t)), 'LineWidth', 1.2, 'Color', 'r');
ylabel('\psi (rad)'); xlabel('Tiempo (s)'); title('Yaw (Guiñada)');
legend('\psi (Real)','\psi n (Red)','Referencia','Location','best');

% B) Velocidades Angulares
figure('Name', 'Dinámica Rotacional: Velocidades Angulares');
subplot(3,1,1);
plot(t, omega(1,:), 'LineWidth', 1.2, 'Color', 'm'); hold on; grid on;
plot(t, omega_nn(1,:), 'LineWidth', 1.2, 'Color', 'g','LineStyle','--');
ylabel('p (rad/s)'); title('\omega_x (Velocidad Roll)');
legend('wx (Real)','wxn n (Red)','Location','best');

subplot(3,1,2);
plot(t, omega(2,:), 'LineWidth', 1.2, 'Color', 'c'); hold on; grid on;
plot(t, omega_nn(2,:), 'LineWidth', 1.2, 'Color', 'g','LineStyle','--');
ylabel('q (rad/s)'); title('\omega_y (Velocidad Pitch)');
legend('wy (Real)','wyn n (Red)','Location','best');

subplot(3,1,3);
plot(t, omega(3,:), 'LineWidth', 1.2, 'Color', 'y'); hold on; grid on;
plot(t, omega_nn(3,:), 'LineWidth', 1.2, 'Color', 'b','LineStyle','--');
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
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Sin Saturar(Vertical)');

subplot(3,2,6);
plot(t, U(1,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Saturado(Vertical)');

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
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Sin Saturar(Vertical)');

subplot(3,2,6);
plot(t, U(1,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Saturado(Vertical)');

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
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Sin Saturar(Vertical)');

subplot(3,2,6);
plot(t, U(1,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Saturado(Vertical)');

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
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Sin Saturar(Roll)');

subplot(3,2,6);
plot(t, U(2,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Saturado(Roll)');

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
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Sin Saturar(Pitch)');

subplot(3,2,6);
plot(t, U(3,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Saturado(Pitch)');

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
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Sin Saturar(Yaw)');

subplot(3,2,6);
plot(t, U(4,:), 'LineWidth', 1.2); grid on;
ylabel('u (N)'); xlabel('Tiempo (s)'); title('Ley de Control Saturado(Yaw)');

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