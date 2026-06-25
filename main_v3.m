% =========================================================================
% SIMULADOR UAV - CONTROL NEURONAL RHONN EKF (6 DoF)
% =========================================================================
clear; close all; clc;

% Agregar subcarpetas al path de MATLAB
addpath('parameters', 'dynamics', 'control_rhonn', 'utils', 'visualization', 'efk', 'rhonn_model');

% 1. Cargar Parámetros e Inicialización 
run('init_system.m'); 

% 2. Generación de Referencias Globales
[t_ref, ref_total, dref_total, ddref_total] = build_ref(dt, Tf, 1);
target_x  = ref_total(1, :); 
target_y  = ref_total(2, :); 
target_z  = ref_total(3, :); 
vx_target = dref_total(1, :); 
vy_target = dref_total(2, :);

% Límite físico de inclinación (Gimbal Lock Protection)
max_inclinacion = deg2rad(35); 

% =========================================================================
% 3. BUCLE PRINCIPAL DE SIMULACIÓN
% =========================================================================
fprintf('Iniciando simulación...\n');

for k = 1:N
    
    % =====================================================================
    % --- PASO 1: Dinámica Física Real (RK4 Multi-Tasa) ---
    % =====================================================================

    % El dron agarra un objeto entre t=15s y t=20s, lo que cambia su masa y su inercia
    if t(k) >=15 && t(k) <= 20
        m_k = m_real(k) * 3; % Simulación de cambio de masa (ejemplo)
        Ix_k = Ix_real(k) * 1.6; % Ajuste de inercia para simular cambio de masa
        Iy_k = Iy_real(k) * 1.2; % Ajuste de inercia para simular cambio de masa
        Iz_k = Iz_real(k) * 1.4; % Ajuste de inercia para simular cambio de masa
    else
    m_k = m_real(k);
    Iz_k = Iz_real(k);
    Ix_k = Ix_real(k);
    Iy_k = Iy_real(k);
    end

    Inertia_k = [Ix_k, Iy_k, Iz_k]; % Actualización de la inercia para el paso actual

    for j = 1:M
        k1_rk = drone_derivatives(S, U(:,k), m_k, g, k_wind, Inertia_k);
        k2_rk = drone_derivatives(S + 0.5*dt_cont*k1_rk, U(:,k), m_k, g, k_wind, Inertia_k);
        k3_rk = drone_derivatives(S + 0.5*dt_cont*k2_rk, U(:,k), m_k, g, k_wind, Inertia_k);
        k4_rk = drone_derivatives(S + dt_cont*k3_rk, U(:,k), m_k, g, k_wind, Inertia_k);
        
        S = S + (dt_cont/6)*(k1_rk + 2*k2_rk + 2*k3_rk + k4_rk);
    end
    
    x(k+1) = S(1);  vx(k+1) = S(2);
    y(k+1) = S(3);  vy(k+1) = S(4);
    z(k+1) = S(5);  vz(k+1) = S(6);
    ang(:,k+1)   = S(7:9);    
    omega(:,k+1) = S(10:12);  

    % =====================================================================
    % --- PASO 2: Generación de Referencias Actuales ---
    % =====================================================================
    % Referencia de Yaw (Gira lentamente a lo largo del tiempo)
    ref_yaw = 0.2 * t_ref * pi;
    % ref_yaw(k)   = 0.2 * t_ref(k) * pi; 
    % ref_yaw(k+1) = 0.2 * t_ref(k+1) * pi; 
    % ref_yaw(k+2) = 0.2 * t_ref(k+2) * pi; 

    % =====================================================================
    % --- PASO 3: Identificadores Neuronales y EKF ---
    % =====================================================================
    % 3.1 Modelos RHONN (Predicciones)
    [xn(k+1), vxn(k+1), H_x_dynamic, Iwu_x_dynamic, Iwx_x_dynamic] = rhonn_model_x_dynamic(...
        x(k), vx(k), ang(1,k), ang(3,k), ang(2,k), ux_des(1,k), w1_x_dynamic(:,k), w2_x_dynamic(:,k), dt, m);

    [yn(k+1), vyn(k+1), H_y_dynamic, Iwu_y_dynamic, Iwx_y_dynamic] = rhonn_model_y_dynamic(...
        y(k), vy(k), ang(1,k), ang(3,k), ang(2,k), uy_des(1,k), w1_y_dynamic(:,k), w2_y_dynamic(:,k), dt, m);

    [zn(k+1), vzn(k+1), H_z_dynamic, Iwu_z_dynamic, Iwx_z_dynamic] = rhonn_model_z_dynamic(...
        z(k), vz(k), ang(1,k), ang(2,k), U(1,k), w1_z_dynamic(:,k), w2_z_dynamic(:,k), dt, g, m);

    [ang_nn(1,k+1), omega_nn(1,k+1), H_roll, Iwu_roll, Iwx_roll] = rhonn_model_roll_dynamic(...
        ang(1,k), omega(1,k), ang(2,k), omega(2,k), omega(3,k), U(2,k), w1_roll(:,k), w2_roll(:,k), dt);

    [ang_nn(2,k+1), omega_nn(2,k+1), H_pitch, Iwu_pitch, Iwx_pitch] = rhonn_model_pitch_dynamic(...
        ang(2,k), omega(2,k), ang(1,k), omega(1,k), omega(3,k), U(3,k), w1_pitch(:,k), w2_pitch(:,k), dt);

    [ang_nn(3,k+1), omega_nn(3,k+1), H_yaw, Iwu_yaw, Iwx_yaw] = rhonn_model_yaw_dynamic(...
        ang(3,k), omega(3,k), ang(1,k), ang(2,k), omega(1,k), omega(2,k), U(4,k), w1_yaw(:,k), w2_yaw(:,k), dt);

    % 3.2 Entrenamiento EKF (Actualización de Pesos)
    [w1_x_dynamic(:,k+1), w2_x_dynamic(:,k+1), p1_x_dynamic(:,:,k+1), p2_x_dynamic(:,:,k+1), e1_ident_x_dynamic(k), e2_ident_x_dynamic(k)] = efk_training_x_dynamic(...
        H_x_dynamic, x(k), vx(k), xn(k), vxn(k), w1_x_dynamic(:,k), w2_x_dynamic(:,k), p1_x_dynamic(:,:,k), p2_x_dynamic(:,:,k), Q1_x_dynamic(:,:,1), Q2_x_dynamic(:,:,1), R1_x_dynamic, R2_x_dynamic);
    
    [w1_y_dynamic(:,k+1), w2_y_dynamic(:,k+1), p1_y_dynamic(:,:,k+1), p2_y_dynamic(:,:,k+1), e1_ident_y_dynamic(k), e2_ident_y_dynamic(k)] = efk_training_y_dynamic(...
        H_y_dynamic, y(k), vy(k), yn(k), vyn(k), w1_y_dynamic(:,k), w2_y_dynamic(:,k), p1_y_dynamic(:,:,k), p2_y_dynamic(:,:,k), Q1_y_dynamic(:,:,1), Q2_y_dynamic(:,:,1), R1_y_dynamic, R2_y_dynamic);

    [w1_z_dynamic(:,k+1), w2_z_dynamic(:,k+1), p1_z_dynamic(:,:,k+1), p2_z_dynamic(:,:,k+1), e1_ident_z_dynamic(k), e2_ident_z_dynamic(k)] = efk_training_z_dynamic(...
        H_z_dynamic, z(k), vz(k), zn(k), vzn(k), w1_z_dynamic(:,k), w2_z_dynamic(:,k), p1_z_dynamic(:,:,k), p2_z_dynamic(:,:,k), Q1_z_dynamic(:,:,1), Q2_z_dynamic(:,:,1), R1_z_dynamic, R2_z_dynamic);

    [w1_roll(:,k+1), w2_roll(:,k+1), p1_roll(:,:,k+1), p2_roll(:,:,k+1), e1_ident_roll(k), e2_ident_roll(k)] = efk_training_roll(...
        H_roll, ang(1,k), omega(1,k), ang_nn(1,k), omega_nn(1,k), w1_roll(:,k), w2_roll(:,k), p1_roll(:,:,k), p2_roll(:,:,k), Q1_roll(:,:,1), Q2_roll(:,:,1), R1_roll, R2_roll);

    [w1_pitch(:,k+1), w2_pitch(:,k+1), p1_pitch(:,:,k+1), p2_pitch(:,:,k+1), e1_ident_pitch(k), e2_ident_pitch(k)] = efk_training_pitch(...
        H_pitch, ang(2,k), omega(2,k), ang_nn(2,k), omega_nn(2,k), w1_pitch(:,k), w2_pitch(:,k), p1_pitch(:,:,k), p2_pitch(:,:,k), Q1_pitch(:,:,1), Q2_pitch(:,:,1), R1_pitch, R2_pitch);

    [w1_yaw(:,k+1), w2_yaw(:,k+1), p1_yaw(:,:,k+1), p2_yaw(:,:,k+1), e1_ident_yaw(k), e2_ident_yaw(k)] = efk_training_yaw(...
        H_yaw, ang(3,k), omega(3,k), ang_nn(3,k), omega_nn(3,k), w1_yaw(:,k), w2_yaw(:,k), p1_yaw(:,:,k), p2_yaw(:,:,k), Q1_yaw(:,:,1), Q2_yaw(:,:,1), R1_yaw, R2_yaw);

    % =====================================================================
    % --- PASO 4: Control de Traslación (Lazo Externo Z, X, Y) ---
    % =====================================================================
    [e1_z_dynamic(k), e2_z_dynamic(k), u_neural_rot(1,k+1)] = control_rhonn_feedback_z_dynamic(...
        z(k), vz(k), ang(1,k), ang(2,k), zn(k+1), w1_z_dynamic(:,k), w2_z_dynamic(:,k), dt, Iwx_z_dynamic, Iwu_z_dynamic, target_z(k), target_z(k+1), target_z(k+2), m, g);

    [e1_x_dynamic(k), e2_x_dynamic(k), ux_des(k+1)] = control_rhonn_feedback_x_dynamic(...
        x(k), vx(k), ang(1,k), ang(3,k), ang(2,k), xn(k+1), w1_x_dynamic(:,k), w2_x_dynamic(:,k), dt, Iwx_x_dynamic, Iwu_x_dynamic, target_x(k), target_x(k+1), target_x(k+2), m, g, ex_sum);
    ex_sum = ex_sum + e1_x_dynamic(k); 
    
    [e1_y_dynamic(k), e2_y_dynamic(k), uy_des(k+1)] = control_rhonn_feedback_y_dynamic(...
        y(k), vy(k), ang(1,k), ang(3,k), ang(2,k), yn(k+1), w1_y_dynamic(:,k), w2_y_dynamic(:,k), dt, Iwx_y_dynamic, Iwu_y_dynamic, target_y(k), target_y(k+1), target_y(k+2), m, g, ey_sum);
    ey_sum = ey_sum + e1_y_dynamic(k); 

    % =====================================================================
    % --- PASO 5: Mapeo de Fuerzas a Ángulos (Compensador + Limites) ---
    % =====================================================================
    [ref_phi_calc_rhonn, ref_theta_calc_rhonn] = compensator(ux_des(k+1), uy_des(k+1), ang(3,k), U(1,k));

    % Protección contra Gimbal Lock y maniobras agresivas
    ref_phi_calc_rhonn   = max(min(ref_phi_calc_rhonn, max_inclinacion), -max_inclinacion);
    ref_theta_calc_rhonn = max(min(ref_theta_calc_rhonn, max_inclinacion), -max_inclinacion);
    %ref_roll_rhonn(vector_indices)  = ref_phi_calc_rhonn;
    %ref_pitch_rhonn(vector_indices) = ref_theta_calc_rhonn;

    % Asignación a vectores de referencia
    vector_indices = k : min(k+2, length(ref_roll_rhonn));
    if t(k) > 0 && t(k) < 15
        ref_roll_rhonn(vector_indices)  = ref_phi_calc_rhonn*0;
        ref_pitch_rhonn(vector_indices) = ref_theta_calc_rhonn*0;
    elseif t(k) >= 15 && t(k) < 25
        ref_roll_rhonn(vector_indices)  = (ref_phi_calc_rhonn/ref_phi_calc_rhonn)*deg2rad(20);
        ref_pitch_rhonn(vector_indices) = (ref_theta_calc_rhonn/ref_theta_calc_rhonn)*deg2rad(-20);
    else
        ref_roll_rhonn(vector_indices)  = ref_phi_calc_rhonn*0;
        ref_pitch_rhonn(vector_indices) = ref_theta_calc_rhonn*0;
    end

    % =============================================================================
    % --- PASO 6: Control de Rotación (Lazo Interno Roll, Pitch, Yaw) ---
    % =============================================================================
    [e1_roll(k), e2_roll(k), u_neural_rot(2,k+1)] = control_rhonn_feedback_roll(...
        ang(1,k), omega(1,k), ang(2,k), omega(2,k), omega(3,k), ang_nn(1,k), omega_nn(1,k), ang_nn(2,k), omega_nn(2,k), omega_nn(3,k), w1_roll(:,k), w2_roll(:,k), dt, Iwx_roll, Iwu_roll, ref_roll_rhonn(k), ref_roll_rhonn(k+1), ref_roll_rhonn(k+2));

    [e1_pitch(k), e2_pitch(k), u_neural_rot(3,k+1)] = control_rhonn_feedback_pitch(...
        ang(2,k), omega(2,k), ang(1,k), omega(1,k), omega(3,k), ang_nn(2,k), omega_nn(2,k), ang_nn(1,k), omega_nn(1,k), omega_nn(3,k), w1_pitch(:,k), w2_pitch(:,k), dt, Iwx_pitch, Iwu_pitch, ref_pitch_rhonn(k), ref_pitch_rhonn(k+1), ref_pitch_rhonn(k+2));

    [e1_yaw(k), e2_yaw(k), u_neural_rot(4,k+1)] = control_rhonn_feedback_yaw(...
        ang(3,k), omega(3,k), ang(1,k), ang(2,k), omega(1,k), omega(2,k), ang_nn(3,k), omega_nn(3,k), ang_nn(1,k), ang_nn(2,k), omega_nn(1,k), omega_nn(2,k), w1_yaw(:,k), w2_yaw(:,k), dt, Iwx_yaw, Iwu_yaw, ref_yaw(k), ref_yaw(k+1), ref_yaw(k+2));

    %u_neural_rot(1,k+1) = m_k*g; % Desactivando control de Z (Mantener Altura)
    %u_neural_rot(2,k+1) = 0; % Desactivando control de Roll
    %u_neural_rot(3,k+1) = 0; % Desactivando control de Pitch
    %u_neural_rot(4,k+1) = 0; % Desactivando control de Yaw

    % =====================================================================
    % --- PASO 7: Mezcla y Saturación de Motores ---
    % =====================================================================
    [U(:,k+1), omega_motors(:,k+1)] = saturacion_motors(u_neural_rot(:,k+1), cT, d, cQ);

end

% =========================================================================
% 4. POST-PROCESAMIENTO (Ajuste de longitudes para graficar)
% =========================================================================
e1_x_dynamic = [e1_x_dynamic, e1_x_dynamic(end)]; e2_x_dynamic = [e2_x_dynamic, e2_x_dynamic(end)];
e1_ident_x_dynamic = [e1_ident_x_dynamic, e1_ident_x_dynamic(end)]; e2_ident_x_dynamic = [e2_ident_x_dynamic, e2_ident_x_dynamic(end)];

e1_y_dynamic = [e1_y_dynamic, e1_y_dynamic(end)]; e2_y_dynamic = [e2_y_dynamic, e2_y_dynamic(end)];
e1_ident_y_dynamic = [e1_ident_y_dynamic, e1_ident_y_dynamic(end)]; e2_ident_y_dynamic = [e2_ident_y_dynamic, e2_ident_y_dynamic(end)];

e1_z_dynamic = [e1_z_dynamic, e1_z_dynamic(end)]; e2_z_dynamic = [e2_z_dynamic, e2_z_dynamic(end)];
e1_ident_z_dynamic = [e1_ident_z_dynamic, e1_ident_z_dynamic(end)]; e2_ident_z_dynamic = [e2_ident_z_dynamic, e2_ident_z_dynamic(end)];

e1_roll = [e1_roll, e1_roll(end)]; e2_roll = [e2_roll, e2_roll(end)];
e1_ident_roll = [e1_ident_roll, e1_ident_roll(end)]; e2_ident_roll = [e2_ident_roll, e2_ident_roll(end)];

e1_pitch = [e1_pitch, e1_pitch(end)]; e2_pitch = [e2_pitch, e2_pitch(end)];
e1_ident_pitch = [e1_ident_pitch, e1_ident_pitch(end)]; e2_ident_pitch = [e2_ident_pitch, e2_ident_pitch(end)];

e1_yaw = [e1_yaw, e1_yaw(end)]; e2_yaw = [e2_yaw, e2_yaw(end)];
e1_ident_yaw = [e1_ident_yaw, e1_ident_yaw(end)]; e2_ident_yaw = [e2_ident_yaw, e2_ident_yaw(end)];

% =========================================================================
% 5. VISUALIZACIÓN Y RESULTADOS
% =========================================================================
plot_results; 
%animate_drone;