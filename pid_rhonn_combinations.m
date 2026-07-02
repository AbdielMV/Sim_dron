% =========================================================================
% SIMULADOR UAV - CONTROL NEURONAL RHONN EKF (6 DoF)
% =========================================================================
clear; close all; clc;

% Agregar subcarpetas al path de MATLAB
addpath('parameters', 'dynamics', 'control_rhonn', 'utils', 'visualization', 'efk', 'rhonn_model');

% 1. Cargar Parámetros e Inicialización 
run('init_system.m'); 

% 2. Generación de Referencias Globales
[t_ref, ref_total, dref_total, ddref_total] = build_ref(dt, Tf, 2);
target_x  = ref_total(1, :); 
target_y  = ref_total(2, :); 
target_z  = ref_total(3, :); 
vx_target = dref_total(1, :); 
vy_target = dref_total(2, :); 
vz_target = dref_total(3, :);

% 1. Estados físicos (Clonamos los originales de init_system.m)
S_sim1 = [x(1); vx(1); y(1); vy(1); z(1); vz(1); ang(1,1); ang(2,1); ang(3,1); omega(1,1); omega(2,1); omega(3,1)];
x_sim1 = x; y_sim1 = y; z_sim1 = z;
vx_sim1 = vx; vy_sim1 = vy; vz_sim1 = vz;
ang_sim1 = ang; omega_sim1 = omega;

% 2. Predicciones del Identificador Neuronal
xn_sim1 = zeros(1, N+1); vxn_sim1 = zeros(1, N+1);
yn_sim1 = zeros(1, N+1); vyn_sim1 = zeros(1, N+1);
zn_sim1 = zeros(1, N+1); vzn_sim1 = zeros(1, N+1);
ang_nn_sim1 = zeros(3, N+1); omega_nn_sim1 = zeros(3, N+1);

% 3. Pesos y Covarianzas del EKF (Clonando desde los valores base)
w1_x_dynamic_sim1 = w1_x_dynamic; w2_x_dynamic_sim1 = w2_x_dynamic; 
p1_x_dynamic_sim1 = p1_x_dynamic; p2_x_dynamic_sim1 = p2_x_dynamic;

w1_y_dynamic_sim1 = w1_y_dynamic; w2_y_dynamic_sim1 = w2_y_dynamic; 
p1_y_dynamic_sim1 = p1_y_dynamic; p2_y_dynamic_sim1 = p2_y_dynamic;

w1_z_dynamic_sim1 = w1_z_dynamic; w2_z_dynamic_sim1 = w2_z_dynamic; 
p1_z_dynamic_sim1 = p1_z_dynamic; p2_z_dynamic_sim1 = p2_z_dynamic;

w1_roll_sim1 = w1_roll; w2_roll_sim1 = w2_roll; 
p1_roll_sim1 = p1_roll; p2_roll_sim1 = p2_roll;

w1_pitch_sim1 = w1_pitch; w2_pitch_sim1 = w2_pitch; 
p1_pitch_sim1 = p1_pitch; p2_pitch_sim1 = p2_pitch;

w1_yaw_sim1 = w1_yaw; w2_yaw_sim1 = w2_yaw; 
p1_yaw_sim1 = p1_yaw; p2_yaw_sim1 = p2_yaw;

% 4. Errores de Identificación (e1, e2 ident)
e1_ident_x_dynamic_sim1 = zeros(1, N+1); e2_ident_x_dynamic_sim1 = zeros(1, N+1);
e1_ident_y_dynamic_sim1 = zeros(1, N+1); e2_ident_y_dynamic_sim1 = zeros(1, N+1);
e1_ident_z_dynamic_sim1 = zeros(1, N+1); e2_ident_z_dynamic_sim1 = zeros(1, N+1);
e1_ident_roll_sim1 = zeros(1, N+1); e2_ident_roll_sim1 = zeros(1, N+1);
e1_ident_pitch_sim1 = zeros(1, N+1); e2_ident_pitch_sim1 = zeros(1, N+1);
e1_ident_yaw_sim1 = zeros(1, N+1); e2_ident_yaw_sim1 = zeros(1, N+1);

% 5. Errores de Control (e1, e2)
e1_x_dynamic_sim1 = zeros(1, N+1); e2_x_dynamic_sim1 = zeros(1, N+1);
e1_y_dynamic_sim1 = zeros(1, N+1); e2_y_dynamic_sim1 = zeros(1, N+1);
e1_z_dynamic_sim1 = zeros(1, N+1); e2_z_dynamic_sim1 = zeros(1, N+1);
e1_roll_sim1 = zeros(1, N+1); e2_roll_sim1 = zeros(1, N+1);
e1_pitch_sim1 = zeros(1, N+1); e2_pitch_sim1 = zeros(1, N+1);
e1_yaw_sim1 = zeros(1, N+1); e2_yaw_sim1 = zeros(1, N+1);

% 6. Control, Compensador y Motores
ux_des_sim1 = zeros(1, N+1); 
uy_des_sim1 = zeros(1, N+1);
u_neural_rot_sim1 = zeros(4, N+1);
U_sim1 = zeros(4, N+1);
U_sim1(1,1) = m * g; % Empuje inicial en t=0 para mantener altura
omega_motors_sim1 = zeros(4, N+1);

ref_roll_rhonn_sim1 = zeros(1, N+1);
ref_pitch_rhonn_sim1 = zeros(1, N+1);

% 7. Variables de Memoria (Derivadas PID e Integrales RHONN)
e_z_prev_sim1 = 0;
e_x_prev_sim1 = 0;
e_y_prev_sim1 = 0;
e_roll_prev_sim1 = 0;
e_pitch_prev_sim1 = 0;   
ex_sum_sim1 = 0;
ey_sum_sim1 = 0;

% =========================================================================
% 3. BUCLE PRINCIPAL DE SIMULACIÓN
% =========================================================================
fprintf('Iniciando simulación control neuronal\n');

% Variables de memoria
e_z_prev_sim1 = 0;
e_x_prev_sim1 = 0;
e_y_prev_sim1 = 0;
e_roll_prev_sim1 = 0;
e_pitch_prev_sim1 = 0;   

% =========================================================================
% 3. BUCLE PRINCIPAL DE SIMULACIÓN 1 (PID TRASLACIÓN / RHONN ORIENTACIÓN)
% =========================================================================
fprintf('Iniciando simulación 1: PID Traslación / RHONN Orientación\n');

% Variables de memoria (etiquetadas para sim1)
e_z_prev_sim1 = 0;
e_x_prev_sim1 = 0;
e_y_prev_sim1 = 0;
e_roll_prev_sim1 = 0;
e_pitch_prev_sim1 = 0;   
ex_sum_sim1 = 0;
ey_sum_sim1 = 0;

for k = 1:N
    
    % =====================================================================
    % --- PASO 1: Dinámica Física Real (RK4 Multi-Tasa) ---
    % =====================================================================

    % Perturbaciones de 8 a 9 y en el tiempo 15 a 20
    if t(k) >= 8 && t(k) < 9
        tau_x_dist_sim1 = 0.0; % Simulación de torque/perturbación en X
        tau_y_dist_sim1 = 0.0; % Simulación de torque/perturbación en Y
        tau_z_dist_sim1 = 0.0; % Simulación de torque/perturbación en Z
        m_k_sim1 = m_real(k) * 1; % Simulación cambio de masa
        Ix_k_sim1 = Ix_real(k) * 1; % Ajuste de inercia en X
        Iy_k_sim1 = Iy_real(k) * 1; % Ajuste de inercia en Y
        Iz_k_sim1 = Iz_real(k) * 1; % Ajuste de inercia en Z
    elseif t(k) >=15 && t(k) <= 20
        tau_x_dist_sim1 = 0.0;
        tau_y_dist_sim1 = 0.0;
        tau_z_dist_sim1 = 0.0;
        m_k_sim1 = m_real(k) * 1;
        Ix_k_sim1 = Ix_real(k) * 1; 
        Iy_k_sim1 = Iy_real(k) * 1; 
        Iz_k_sim1 = Iz_real(k) * 1;
    else
        tau_x_dist_sim1 = 0.0;
        tau_y_dist_sim1 = 0.0;
        tau_z_dist_sim1 = 0.0;
        m_k_sim1 = m_real(k);
        Ix_k_sim1 = Ix_real(k); 
        Iy_k_sim1 = Iy_real(k); 
        Iz_k_sim1 = Iz_real(k);
    end

    Inertia_k_sim1 = [Ix_k_sim1, Iy_k_sim1, Iz_k_sim1]; 
    tau_dist_k_sim1 = [tau_x_dist_sim1; tau_y_dist_sim1; tau_z_dist_sim1]; 

    for j = 1:M
        k1_rk = drone_derivatives(S_sim1, U_sim1(:,k), m_k_sim1, g, k_wind, Inertia_k_sim1, tau_dist_k_sim1);
        k2_rk = drone_derivatives(S_sim1 + 0.5*dt_cont*k1_rk, U_sim1(:,k), m_k_sim1, g, k_wind, Inertia_k_sim1, tau_dist_k_sim1);
        k3_rk = drone_derivatives(S_sim1 + 0.5*dt_cont*k2_rk, U_sim1(:,k), m_k_sim1, g, k_wind, Inertia_k_sim1, tau_dist_k_sim1);
        k4_rk = drone_derivatives(S_sim1 + dt_cont*k3_rk, U_sim1(:,k), m_k_sim1, g, k_wind, Inertia_k_sim1, tau_dist_k_sim1);
        S_sim1 = S_sim1 + (dt_cont/6)*(k1_rk + 2*k2_rk + 2*k3_rk + k4_rk);
    end
    
    x_sim1(k+1) = S_sim1(1);  vx_sim1(k+1) = S_sim1(2);
    y_sim1(k+1) = S_sim1(3);  vy_sim1(k+1) = S_sim1(4);
    z_sim1(k+1) = S_sim1(5);  vz_sim1(k+1) = S_sim1(6);
    ang_sim1(:,k+1)   = S_sim1(7:9);    
    omega_sim1(:,k+1) = S_sim1(10:12);  

    % =====================================================================
    % --- PASO 2: Generación de Referencias Actuales ---
    % =====================================================================
    ref_yaw_sim1 = (0.2 * t_ref * pi)*0;

    % =====================================================================
    % --- PASO 3: Identificadores Neuronales y EKF ---
    % =====================================================================
    % 3.1 Modelos RHONN (Predicciones)
    [xn_sim1(k+1), vxn_sim1(k+1), H_x_dynamic_sim1, Iwu_x_dynamic_sim1, Iwx_x_dynamic_sim1] = rhonn_model_x_dynamic(...
        x_sim1(k), vx_sim1(k), ang_sim1(1,k), ang_sim1(3,k), ang_sim1(2,k), ux_des_sim1(1,k), w1_x_dynamic_sim1(:,k), w2_x_dynamic_sim1(:,k), dt, m);

    [yn_sim1(k+1), vyn_sim1(k+1), H_y_dynamic_sim1, Iwu_y_dynamic_sim1, Iwx_y_dynamic_sim1] = rhonn_model_y_dynamic(...
        y_sim1(k), vy_sim1(k), ang_sim1(1,k), ang_sim1(3,k), ang_sim1(2,k), uy_des_sim1(1,k), w1_y_dynamic_sim1(:,k), w2_y_dynamic_sim1(:,k), dt, m);

    [zn_sim1(k+1), vzn_sim1(k+1), H_z_dynamic_sim1, Iwu_z_dynamic_sim1, Iwx_z_dynamic_sim1] = rhonn_model_z_dynamic(...
        z_sim1(k), vz_sim1(k), ang_sim1(1,k), ang_sim1(2,k), U_sim1(1,k), w1_z_dynamic_sim1(:,k), w2_z_dynamic_sim1(:,k), dt, g, m);

    [ang_nn_sim1(1,k+1), omega_nn_sim1(1,k+1), H_roll_sim1, Iwu_roll_sim1, Iwx_roll_sim1] = rhonn_model_roll_dynamic(...
        ang_sim1(1,k), omega_sim1(1,k), ang_sim1(2,k), omega_sim1(2,k), omega_sim1(3,k), U_sim1(2,k), w1_roll_sim1(:,k), w2_roll_sim1(:,k), dt);

    [ang_nn_sim1(2,k+1), omega_nn_sim1(2,k+1), H_pitch_sim1, Iwu_pitch_sim1, Iwx_pitch_sim1] = rhonn_model_pitch_dynamic(...
        ang_sim1(2,k), omega_sim1(2,k), ang_sim1(1,k), omega_sim1(1,k), omega_sim1(3,k), U_sim1(3,k), w1_pitch_sim1(:,k), w2_pitch_sim1(:,k), dt);

    [ang_nn_sim1(3,k+1), omega_nn_sim1(3,k+1), H_yaw_sim1, Iwu_yaw_sim1, Iwx_yaw_sim1] = rhonn_model_yaw_dynamic(...
        ang_sim1(3,k), omega_sim1(3,k), ang_sim1(1,k), ang_sim1(2,k), omega_sim1(1,k), omega_sim1(2,k), U_sim1(4,k), w1_yaw_sim1(:,k), w2_yaw_sim1(:,k), dt);

    % 3.2 Entrenamiento EKF (Actualización de Pesos)
    [w1_x_dynamic_sim1(:,k+1), w2_x_dynamic_sim1(:,k+1), p1_x_dynamic_sim1(:,:,k+1), p2_x_dynamic_sim1(:,:,k+1), e1_ident_x_dynamic_sim1(k), e2_ident_x_dynamic_sim1(k)] = efk_training_x_dynamic(...
        H_x_dynamic_sim1, x_sim1(k), vx_sim1(k), xn_sim1(k), vxn_sim1(k), w1_x_dynamic_sim1(:,k), w2_x_dynamic_sim1(:,k), p1_x_dynamic_sim1(:,:,k), p2_x_dynamic_sim1(:,:,k), Q1_x_dynamic(:,:,1), Q2_x_dynamic(:,:,1), R1_x_dynamic, R2_x_dynamic);
    
    [w1_y_dynamic_sim1(:,k+1), w2_y_dynamic_sim1(:,k+1), p1_y_dynamic_sim1(:,:,k+1), p2_y_dynamic_sim1(:,:,k+1), e1_ident_y_dynamic_sim1(k), e2_ident_y_dynamic_sim1(k)] = efk_training_y_dynamic(...
        H_y_dynamic_sim1, y_sim1(k), vy_sim1(k), yn_sim1(k), vyn_sim1(k), w1_y_dynamic_sim1(:,k), w2_y_dynamic_sim1(:,k), p1_y_dynamic_sim1(:,:,k), p2_y_dynamic_sim1(:,:,k), Q1_y_dynamic(:,:,1), Q2_y_dynamic(:,:,1), R1_y_dynamic, R2_y_dynamic);

    [w1_z_dynamic_sim1(:,k+1), w2_z_dynamic_sim1(:,k+1), p1_z_dynamic_sim1(:,:,k+1), p2_z_dynamic_sim1(:,:,k+1), e1_ident_z_dynamic_sim1(k), e2_ident_z_dynamic_sim1(k)] = efk_training_z_dynamic(...
        H_z_dynamic_sim1, z_sim1(k), vz_sim1(k), zn_sim1(k), vzn_sim1(k), w1_z_dynamic_sim1(:,k), w2_z_dynamic_sim1(:,k), p1_z_dynamic_sim1(:,:,k), p2_z_dynamic_sim1(:,:,k), Q1_z_dynamic(:,:,1), Q2_z_dynamic(:,:,1), R1_z_dynamic, R2_z_dynamic);

    [w1_roll_sim1(:,k+1), w2_roll_sim1(:,k+1), p1_roll_sim1(:,:,k+1), p2_roll_sim1(:,:,k+1), e1_ident_roll_sim1(k), e2_ident_roll_sim1(k)] = efk_training_roll(...
        H_roll_sim1, ang_sim1(1,k), omega_sim1(1,k), ang_nn_sim1(1,k), omega_nn_sim1(1,k), w1_roll_sim1(:,k), w2_roll_sim1(:,k), p1_roll_sim1(:,:,k), p2_roll_sim1(:,:,k), Q1_roll(:,:,1), Q2_roll(:,:,1), R1_roll, R2_roll);

    [w1_pitch_sim1(:,k+1), w2_pitch_sim1(:,k+1), p1_pitch_sim1(:,:,k+1), p2_pitch_sim1(:,:,k+1), e1_ident_pitch_sim1(k), e2_ident_pitch_sim1(k)] = efk_training_pitch(...
        H_pitch_sim1, ang_sim1(2,k), omega_sim1(2,k), ang_nn_sim1(2,k), omega_nn_sim1(2,k), w1_pitch_sim1(:,k), w2_pitch_sim1(:,k), p1_pitch_sim1(:,:,k), p2_pitch_sim1(:,:,k), Q1_pitch(:,:,1), Q2_pitch(:,:,1), R1_pitch, R2_pitch);

    [w1_yaw_sim1(:,k+1), w2_yaw_sim1(:,k+1), p1_yaw_sim1(:,:,k+1), p2_yaw_sim1(:,:,k+1), e1_ident_yaw_sim1(k), e2_ident_yaw_sim1(k)] = efk_training_yaw(...
        H_yaw_sim1, ang_sim1(3,k), omega_sim1(3,k), ang_nn_sim1(3,k), omega_nn_sim1(3,k), w1_yaw_sim1(:,k), w2_yaw_sim1(:,k), p1_yaw_sim1(:,:,k), p2_yaw_sim1(:,:,k), Q1_yaw(:,:,1), Q2_yaw(:,:,1), R1_yaw, R2_yaw);

    % =====================================================================
    % --- PASO 4: Control de Traslación (Lazo Externo Z, X, Y) ---
    % =====================================================================
    [e1_z_dynamic_sim1(k), e2_z_dynamic_sim1(k), u_neural_rot_sim1(1,k+1)] = control_rhonn_feedback_z_dynamic(...
        z_sim1(k+1), vz_sim1(k+1), ang_sim1(1,k), ang_sim1(2,k), zn_sim1(k+1), w1_z_dynamic_sim1(:,k), w2_z_dynamic_sim1(:,k), dt, Iwx_z_dynamic_sim1, Iwu_z_dynamic_sim1, target_z(k), target_z(k+1), target_z(k+2), m, g);

    [e1_x_dynamic_sim1(k), e2_x_dynamic_sim1(k), ux_des_sim1(k+1)] = control_rhonn_feedback_x_dynamic(...
        x_sim1(k+1), vx_sim1(k+1), ang_sim1(1,k), ang_sim1(3,k), ang_sim1(2,k), xn_sim1(k+1), w1_x_dynamic_sim1(:,k), w2_x_dynamic_sim1(:,k), dt, Iwx_x_dynamic_sim1, Iwu_x_dynamic_sim1, target_x(k), target_x(k+1), target_x(k+2), m, g, ex_sum_sim1);
    ex_sum_sim1 = ex_sum_sim1 + e1_x_dynamic_sim1(k); 

    [e1_y_dynamic_sim1(k), e2_y_dynamic_sim1(k), uy_des_sim1(k+1)] = control_rhonn_feedback_y_dynamic(...
        y_sim1(k+1), vy_sim1(k+1), ang_sim1(1,k), ang_sim1(3,k), ang_sim1(2,k), yn_sim1(k+1), w1_y_dynamic_sim1(:,k), w2_y_dynamic_sim1(:,k), dt, Iwx_y_dynamic_sim1, Iwu_y_dynamic_sim1, target_y(k), target_y(k+1), target_y(k+2), m, g, ey_sum_sim1);
    ey_sum_sim1 = ey_sum_sim1 + e1_y_dynamic_sim1(k); 
    
    % Ganancias PID
    kp_pos_sim1 = 4.0; % Ganancia proporcional
    kd_pos_sim1 = 3.5; % Ganancia derivativa
    
    % 1. Cálculo de Errores de Posición y Velocidad
    e_x_sim1 = target_x(k) - x_sim1(k+1);
    de_x_sim1 = (e_x_sim1 - e_x_prev_sim1) / dt;
    
    e_y_sim1 = target_y(k) - y_sim1(k+1);
    de_y_sim1 = (e_y_sim1 - e_y_prev_sim1) / dt;
    
    % Memoria previa
    e_x_prev_sim1 = e_x_sim1; e_y_prev_sim1 = e_y_sim1;

    % 2. Esfuerzo de Control PD (Fuerza Deseada)
    ux_des_sim1(k+1) = m * (kp_pos_sim1 * e_x_sim1 + kd_pos_sim1 * de_x_sim1);
    uy_des_sim1(k+1) = m * (kp_pos_sim1 * e_y_sim1 + kd_pos_sim1 * de_y_sim1);

    % =====================================================================
    % --- PASO 5: Mapeo Algebraico y Predicción DSTD (Super Twisting) ---
    % =====================================================================
    % --- Instante actual (k) ---
    [ref_phi_k_sim1, ref_theta_k_sim1] = compensator(ux_des_sim1(k+1), uy_des_sim1(k+1), ang_sim1(3,k), U_sim1(1,k));

    ref_roll_rhonn_sim1(k)   = ref_phi_k_sim1;
    ref_roll_rhonn_sim1(k+1) = ref_phi_k_sim1 + (dt*ref_phi_k_sim1);
    ref_roll_rhonn_sim1(k+2) = ref_phi_k_sim1 + (2*dt*ref_phi_k_sim1);

    ref_pitch_rhonn_sim1(k)   = ref_theta_k_sim1;
    ref_pitch_rhonn_sim1(k+1) = ref_theta_k_sim1 + (dt*ref_theta_k_sim1);
    ref_pitch_rhonn_sim1(k+2) = ref_theta_k_sim1 + (2*dt*ref_theta_k_sim1);

    % =============================================================================
    % --- PASO 6: Control de Rotación (Lazo Interno Roll, Pitch, Yaw) ---
    % =============================================================================
    [e1_roll_sim1(k), e2_roll_sim1(k), u_neural_rot_sim1(2,k+1)] = control_rhonn_feedback_roll(...
        ang_sim1(1,k+1), omega_sim1(1,k+1), ang_sim1(2,k), omega_sim1(2,k), omega_sim1(3,k), ang_nn_sim1(1,k+1), omega_nn_sim1(1,k), ang_nn_sim1(2,k), omega_nn_sim1(2,k), omega_nn_sim1(3,k), w1_roll_sim1(:,k), w2_roll_sim1(:,k), dt, Iwx_roll_sim1, Iwu_roll_sim1, ref_roll_rhonn_sim1(k), ref_roll_rhonn_sim1(k+1), ref_roll_rhonn_sim1(k+2));

    [e1_pitch_sim1(k), e2_pitch_sim1(k), u_neural_rot_sim1(3,k+1)] = control_rhonn_feedback_pitch(...
        ang_sim1(2,k+1), omega_sim1(2,k+1), ang_sim1(1,k), omega_sim1(1,k), omega_sim1(3,k), ang_nn_sim1(2,k+1), omega_nn_sim1(2,k), ang_nn_sim1(1,k), omega_nn_sim1(1,k), omega_nn_sim1(3,k), w1_pitch_sim1(:,k), w2_pitch_sim1(:,k), dt, Iwx_pitch_sim1, Iwu_pitch_sim1, ref_pitch_rhonn_sim1(k), ref_pitch_rhonn_sim1(k+1), ref_pitch_rhonn_sim1(k+2));

    [e1_yaw_sim1(k), e2_yaw_sim1(k), u_neural_rot_sim1(4,k+1)] = control_rhonn_feedback_yaw(...
        ang_sim1(3,k+1), omega_sim1(3,k+1), ang_sim1(1,k), ang_sim1(2,k), omega_sim1(1,k), omega_sim1(2,k), ang_nn_sim1(3,k+1), omega_nn_sim1(3,k), ang_nn_sim1(1,k), ang_nn_sim1(2,k), omega_nn_sim1(1,k), omega_nn_sim1(2,k), w1_yaw_sim1(:,k), w2_yaw_sim1(:,k), dt, Iwx_yaw_sim1, Iwu_yaw_sim1, ref_yaw_sim1(k), ref_yaw_sim1(k+1), ref_yaw_sim1(k+2));

    % =====================================================================
    % --- PASO 7: Mezcla y Saturación de Motores ---
    % =====================================================================
    [U_sim1(:,k+1), omega_motors_sim1(:,k+1)] = saturacion_motors(u_neural_rot_sim1(:,k+1), cT, d, cQ);

end
fprintf('Finalizada simulación 1\n');

% =========================================================================
% --- INICIALIZACIÓN DE VARIABLES ÚNICAS: SIMULACIÓN 2 ---
% =========================================================================
fprintf('Asignando memoria para Simulación 2...\n');

S_sim2 = [x(1); vx(1); y(1); vy(1); z(1); vz(1); ang(1,1); ang(2,1); ang(3,1); omega(1,1); omega(2,1); omega(3,1)];
x_sim2 = x; y_sim2 = y; z_sim2 = z;
vx_sim2 = vx; vy_sim2 = vy; vz_sim2 = vz;
ang_sim2 = ang; omega_sim2 = omega;

xn_sim2 = zeros(1, N+1); vxn_sim2 = zeros(1, N+1);
yn_sim2 = zeros(1, N+1); vyn_sim2 = zeros(1, N+1);
zn_sim2 = zeros(1, N+1); vzn_sim2 = zeros(1, N+1);
ang_nn_sim2 = zeros(3, N+1); omega_nn_sim2 = zeros(3, N+1);

w1_x_dynamic_sim2 = w1_x_dynamic; w2_x_dynamic_sim2 = w2_x_dynamic; p1_x_dynamic_sim2 = p1_x_dynamic; p2_x_dynamic_sim2 = p2_x_dynamic;
w1_y_dynamic_sim2 = w1_y_dynamic; w2_y_dynamic_sim2 = w2_y_dynamic; p1_y_dynamic_sim2 = p1_y_dynamic; p2_y_dynamic_sim2 = p2_y_dynamic;
w1_z_dynamic_sim2 = w1_z_dynamic; w2_z_dynamic_sim2 = w2_z_dynamic; p1_z_dynamic_sim2 = p1_z_dynamic; p2_z_dynamic_sim2 = p2_z_dynamic;
w1_roll_sim2 = w1_roll; w2_roll_sim2 = w2_roll; p1_roll_sim2 = p1_roll; p2_roll_sim2 = p2_roll;
w1_pitch_sim2 = w1_pitch; w2_pitch_sim2 = w2_pitch; p1_pitch_sim2 = p1_pitch; p2_pitch_sim2 = p2_pitch;
w1_yaw_sim2 = w1_yaw; w2_yaw_sim2 = w2_yaw; p1_yaw_sim2 = p1_yaw; p2_yaw_sim2 = p2_yaw;

e1_ident_x_dynamic_sim2 = zeros(1, N+1); e2_ident_x_dynamic_sim2 = zeros(1, N+1);
e1_ident_y_dynamic_sim2 = zeros(1, N+1); e2_ident_y_dynamic_sim2 = zeros(1, N+1);
e1_ident_z_dynamic_sim2 = zeros(1, N+1); e2_ident_z_dynamic_sim2 = zeros(1, N+1);
e1_ident_roll_sim2 = zeros(1, N+1); e2_ident_roll_sim2 = zeros(1, N+1);
e1_ident_pitch_sim2 = zeros(1, N+1); e2_ident_pitch_sim2 = zeros(1, N+1);
e1_ident_yaw_sim2 = zeros(1, N+1); e2_ident_yaw_sim2 = zeros(1, N+1);

e1_x_dynamic_sim2 = zeros(1, N+1); e2_x_dynamic_sim2 = zeros(1, N+1);
e1_y_dynamic_sim2 = zeros(1, N+1); e2_y_dynamic_sim2 = zeros(1, N+1);
e1_z_dynamic_sim2 = zeros(1, N+1); e2_z_dynamic_sim2 = zeros(1, N+1);
e1_roll_sim2 = zeros(1, N+1); e2_roll_sim2 = zeros(1, N+1);
e1_pitch_sim2 = zeros(1, N+1); e2_pitch_sim2 = zeros(1, N+1);
e1_yaw_sim2 = zeros(1, N+1); e2_yaw_sim2 = zeros(1, N+1);

ux_des_sim2 = zeros(1, N+1); uy_des_sim2 = zeros(1, N+1);
u_neural_rot_sim2 = zeros(4, N+1);
U_sim2 = zeros(4, N+1); U_sim2(1,1) = m * g;
omega_motors_sim2 = zeros(4, N+1);
ref_roll_rhonn_sim2 = zeros(1, N+1); ref_pitch_rhonn_sim2 = zeros(1, N+1);

e_z_prev_sim2 = 0; e_x_prev_sim2 = 0; e_y_prev_sim2 = 0;
e_roll_prev_sim2 = 0; e_pitch_prev_sim2 = 0;   
ex_sum_sim2 = 0; ey_sum_sim2 = 0;

% =========================================================================
% 3. BUCLE PRINCIPAL DE SIMULACIÓN 2
% =========================================================================
fprintf('Iniciando simulación 2...\n');

for k = 1:N
    
    % =====================================================================
    % --- PASO 1: Dinámica Física Real (RK4 Multi-Tasa) ---
    % =====================================================================

    if t(k) >= 8 && t(k) < 9
        tau_x_dist_sim2 = 0.0; tau_y_dist_sim2 = 0.0; tau_z_dist_sim2 = 0.0;
        m_k_sim2 = m_real(k) * 1; 
        Ix_k_sim2 = Ix_real(k) * 1; Iy_k_sim2 = Iy_real(k) * 1; Iz_k_sim2 = Iz_real(k) * 1;
    elseif t(k) >=15 && t(k) <= 20
        tau_x_dist_sim2 = 0.0; tau_y_dist_sim2 = 0.0; tau_z_dist_sim2 = 0.0;
        m_k_sim2 = m_real(k) * 1;
        Ix_k_sim2 = Ix_real(k) * 1; Iy_k_sim2 = Iy_real(k) * 1; Iz_k_sim2 = Iz_real(k) * 1;
    else
        tau_x_dist_sim2 = 0.0; tau_y_dist_sim2 = 0.0; tau_z_dist_sim2 = 0.0;
        m_k_sim2 = m_real(k);
        Ix_k_sim2 = Ix_real(k); Iy_k_sim2 = Iy_real(k); Iz_k_sim2 = Iz_real(k);
    end

    Inertia_k_sim2 = [Ix_k_sim2, Iy_k_sim2, Iz_k_sim2]; 
    tau_dist_k_sim2 = [tau_x_dist_sim2; tau_y_dist_sim2; tau_z_dist_sim2]; 

    for j = 1:M
        k1_rk = drone_derivatives(S_sim2, U_sim2(:,k), m_k_sim2, g, k_wind, Inertia_k_sim2, tau_dist_k_sim2);
        k2_rk = drone_derivatives(S_sim2 + 0.5*dt_cont*k1_rk, U_sim2(:,k), m_k_sim2, g, k_wind, Inertia_k_sim2, tau_dist_k_sim2);
        k3_rk = drone_derivatives(S_sim2 + 0.5*dt_cont*k2_rk, U_sim2(:,k), m_k_sim2, g, k_wind, Inertia_k_sim2, tau_dist_k_sim2);
        k4_rk = drone_derivatives(S_sim2 + dt_cont*k3_rk, U_sim2(:,k), m_k_sim2, g, k_wind, Inertia_k_sim2, tau_dist_k_sim2);

        S_sim2 = S_sim2 + (dt_cont/6)*(k1_rk + 2*k2_rk + 2*k3_rk + k4_rk);
    end
    
    x_sim2(k+1) = S_sim2(1);  vx_sim2(k+1) = S_sim2(2);
    y_sim2(k+1) = S_sim2(3);  vy_sim2(k+1) = S_sim2(4);
    z_sim2(k+1) = S_sim2(5);  vz_sim2(k+1) = S_sim2(6);
    ang_sim2(:,k+1)   = S_sim2(7:9);    
    omega_sim2(:,k+1) = S_sim2(10:12);  

    % =====================================================================
    % --- PASO 2: Generación de Referencias Actuales ---
    % =====================================================================
    ref_yaw_sim2 = (0.2 * t_ref * pi)*0;

    % =====================================================================
    % --- PASO 3: Identificadores Neuronales y EKF ---
    % =====================================================================
    [xn_sim2(k+1), vxn_sim2(k+1), H_x_dynamic_sim2, Iwu_x_dynamic_sim2, Iwx_x_dynamic_sim2] = rhonn_model_x_dynamic(...
        x_sim2(k), vx_sim2(k), ang_sim2(1,k), ang_sim2(3,k), ang_sim2(2,k), ux_des_sim2(1,k), w1_x_dynamic_sim2(:,k), w2_x_dynamic_sim2(:,k), dt, m);

    [yn_sim2(k+1), vyn_sim2(k+1), H_y_dynamic_sim2, Iwu_y_dynamic_sim2, Iwx_y_dynamic_sim2] = rhonn_model_y_dynamic(...
        y_sim2(k), vy_sim2(k), ang_sim2(1,k), ang_sim2(3,k), ang_sim2(2,k), uy_des_sim2(1,k), w1_y_dynamic_sim2(:,k), w2_y_dynamic_sim2(:,k), dt, m);

    [zn_sim2(k+1), vzn_sim2(k+1), H_z_dynamic_sim2, Iwu_z_dynamic_sim2, Iwx_z_dynamic_sim2] = rhonn_model_z_dynamic(...
        z_sim2(k), vz_sim2(k), ang_sim2(1,k), ang_sim2(2,k), U_sim2(1,k), w1_z_dynamic_sim2(:,k), w2_z_dynamic_sim2(:,k), dt, g, m);

    [ang_nn_sim2(1,k+1), omega_nn_sim2(1,k+1), H_roll_sim2, Iwu_roll_sim2, Iwx_roll_sim2] = rhonn_model_roll_dynamic(...
        ang_sim2(1,k), omega_sim2(1,k), ang_sim2(2,k), omega_sim2(2,k), omega_sim2(3,k), U_sim2(2,k), w1_roll_sim2(:,k), w2_roll_sim2(:,k), dt);

    [ang_nn_sim2(2,k+1), omega_nn_sim2(2,k+1), H_pitch_sim2, Iwu_pitch_sim2, Iwx_pitch_sim2] = rhonn_model_pitch_dynamic(...
        ang_sim2(2,k), omega_sim2(2,k), ang_sim2(1,k), omega_sim2(1,k), omega_sim2(3,k), U_sim2(3,k), w1_pitch_sim2(:,k), w2_pitch_sim2(:,k), dt);

    [ang_nn_sim2(3,k+1), omega_nn_sim2(3,k+1), H_yaw_sim2, Iwu_yaw_sim2, Iwx_yaw_sim2] = rhonn_model_yaw_dynamic(...
        ang_sim2(3,k), omega_sim2(3,k), ang_sim2(1,k), ang_sim2(2,k), omega_sim2(1,k), omega_sim2(2,k), U_sim2(4,k), w1_yaw_sim2(:,k), w2_yaw_sim2(:,k), dt);

    [w1_x_dynamic_sim2(:,k+1), w2_x_dynamic_sim2(:,k+1), p1_x_dynamic_sim2(:,:,k+1), p2_x_dynamic_sim2(:,:,k+1), e1_ident_x_dynamic_sim2(k), e2_ident_x_dynamic_sim2(k)] = efk_training_x_dynamic(...
        H_x_dynamic_sim2, x_sim2(k), vx_sim2(k), xn_sim2(k), vxn_sim2(k), w1_x_dynamic_sim2(:,k), w2_x_dynamic_sim2(:,k), p1_x_dynamic_sim2(:,:,k), p2_x_dynamic_sim2(:,:,k), Q1_x_dynamic(:,:,1), Q2_x_dynamic(:,:,1), R1_x_dynamic, R2_x_dynamic);
    
    [w1_y_dynamic_sim2(:,k+1), w2_y_dynamic_sim2(:,k+1), p1_y_dynamic_sim2(:,:,k+1), p2_y_dynamic_sim2(:,:,k+1), e1_ident_y_dynamic_sim2(k), e2_ident_y_dynamic_sim2(k)] = efk_training_y_dynamic(...
        H_y_dynamic_sim2, y_sim2(k), vy_sim2(k), yn_sim2(k), vyn_sim2(k), w1_y_dynamic_sim2(:,k), w2_y_dynamic_sim2(:,k), p1_y_dynamic_sim2(:,:,k), p2_y_dynamic_sim2(:,:,k), Q1_y_dynamic(:,:,1), Q2_y_dynamic(:,:,1), R1_y_dynamic, R2_y_dynamic);

    [w1_z_dynamic_sim2(:,k+1), w2_z_dynamic_sim2(:,k+1), p1_z_dynamic_sim2(:,:,k+1), p2_z_dynamic_sim2(:,:,k+1), e1_ident_z_dynamic_sim2(k), e2_ident_z_dynamic_sim2(k)] = efk_training_z_dynamic(...
        H_z_dynamic_sim2, z_sim2(k), vz_sim2(k), zn_sim2(k), vzn_sim2(k), w1_z_dynamic_sim2(:,k), w2_z_dynamic_sim2(:,k), p1_z_dynamic_sim2(:,:,k), p2_z_dynamic_sim2(:,:,k), Q1_z_dynamic(:,:,1), Q2_z_dynamic(:,:,1), R1_z_dynamic, R2_z_dynamic);

    [w1_roll_sim2(:,k+1), w2_roll_sim2(:,k+1), p1_roll_sim2(:,:,k+1), p2_roll_sim2(:,:,k+1), e1_ident_roll_sim2(k), e2_ident_roll_sim2(k)] = efk_training_roll(...
        H_roll_sim2, ang_sim2(1,k), omega_sim2(1,k), ang_nn_sim2(1,k), omega_nn_sim2(1,k), w1_roll_sim2(:,k), w2_roll_sim2(:,k), p1_roll_sim2(:,:,k), p2_roll_sim2(:,:,k), Q1_roll(:,:,1), Q2_roll(:,:,1), R1_roll, R2_roll);

    [w1_pitch_sim2(:,k+1), w2_pitch_sim2(:,k+1), p1_pitch_sim2(:,:,k+1), p2_pitch_sim2(:,:,k+1), e1_ident_pitch_sim2(k), e2_ident_pitch_sim2(k)] = efk_training_pitch(...
        H_pitch_sim2, ang_sim2(2,k), omega_sim2(2,k), ang_nn_sim2(2,k), omega_nn_sim2(2,k), w1_pitch_sim2(:,k), w2_pitch_sim2(:,k), p1_pitch_sim2(:,:,k), p2_pitch_sim2(:,:,k), Q1_pitch(:,:,1), Q2_pitch(:,:,1), R1_pitch, R2_pitch);

    [w1_yaw_sim2(:,k+1), w2_yaw_sim2(:,k+1), p1_yaw_sim2(:,:,k+1), p2_yaw_sim2(:,:,k+1), e1_ident_yaw_sim2(k), e2_ident_yaw_sim2(k)] = efk_training_yaw(...
        H_yaw_sim2, ang_sim2(3,k), omega_sim2(3,k), ang_nn_sim2(3,k), omega_nn_sim2(3,k), w1_yaw_sim2(:,k), w2_yaw_sim2(:,k), p1_yaw_sim2(:,:,k), p2_yaw_sim2(:,:,k), Q1_yaw(:,:,1), Q2_yaw(:,:,1), R1_yaw, R2_yaw);

    % =====================================================================
    % --- PASO 4: Control de Traslación (Lazo Externo Z, X, Y) ---
    % =====================================================================
    [e1_z_dynamic_sim2(k), e2_z_dynamic_sim2(k), u_neural_rot_sim2(1,k+1)] = control_rhonn_feedback_z_dynamic(...
        z_sim2(k+1), vz_sim2(k+1), ang_sim2(1,k), ang_sim2(2,k), zn_sim2(k+1), w1_z_dynamic_sim2(:,k), w2_z_dynamic_sim2(:,k), dt, Iwx_z_dynamic_sim2, Iwu_z_dynamic_sim2, target_z(k), target_z(k+1), target_z(k+2), m, g);

    [e1_x_dynamic_sim2(k), e2_x_dynamic_sim2(k), ux_des_sim2(k+1)] = control_rhonn_feedback_x_dynamic(...
        x_sim2(k+1), vx_sim2(k+1), ang_sim2(1,k), ang_sim2(3,k), ang_sim2(2,k), xn_sim2(k+1), w1_x_dynamic_sim2(:,k), w2_x_dynamic_sim2(:,k), dt, Iwx_x_dynamic_sim2, Iwu_x_dynamic_sim2, target_x(k), target_x(k+1), target_x(k+2), m, g, ex_sum_sim2);
    ex_sum_sim2 = ex_sum_sim2 + e1_x_dynamic_sim2(k); 

    [e1_y_dynamic_sim2(k), e2_y_dynamic_sim2(k), uy_des_sim2(k+1)] = control_rhonn_feedback_y_dynamic(...
        y_sim2(k+1), vy_sim2(k+1), ang_sim2(1,k), ang_sim2(3,k), ang_sim2(2,k), yn_sim2(k+1), w1_y_dynamic_sim2(:,k), w2_y_dynamic_sim2(:,k), dt, Iwx_y_dynamic_sim2, Iwu_y_dynamic_sim2, target_y(k), target_y(k+1), target_y(k+2), m, g, ey_sum_sim2);
    ey_sum_sim2 = ey_sum_sim2 + e1_y_dynamic_sim2(k); 
    
    % Ganancias PID
    kp_pos_sim2 = 4.0; % Ganancia proporcional
    kd_pos_sim2 = 3.5; % Ganancia derivativa
    
    % 1. Cálculo de Errores de Posición y Velocidad
    e_x_sim2 = target_x(k) - x_sim2(k+1);
    de_x_sim2 = (e_x_sim2 - e_x_prev_sim2) / dt;
    
    e_y_sim2 = target_y(k) - y_sim2(k+1);
    de_y_sim2 = (e_y_sim2 - e_y_prev_sim2) / dt;
    
    % Memoria previa
    e_x_prev_sim2 = e_x_sim2; e_y_prev_sim2 = e_y_sim2;

    % 2. Esfuerzo de Control PD (Fuerza Deseada)
    ux_des_sim2(k+1) = m * (kp_pos_sim2 * e_x_sim2 + kd_pos_sim2 * de_x_sim2);
    uy_des_sim2(k+1) = m * (kp_pos_sim2 * e_y_sim2 + kd_pos_sim2 * de_y_sim2);

    % =====================================================================
    % --- PASO 5: Mapeo Algebraico y Predicción DSTD (Super Twisting) ---
    % =====================================================================
    
    % --- Instante actual (k) ---
    [ref_phi_k_sim2, ref_theta_k_sim2] = compensator(ux_des_sim2(k+1), uy_des_sim2(k+1), ang_sim2(3,k), U_sim2(1,k));

    ref_phi_k_sim2 = ref_phi_k_sim2;
    ref_theta_k_sim2 = ref_theta_k_sim2;

    ref_roll_rhonn_sim2(k)   = ref_phi_k_sim2;
    ref_roll_rhonn_sim2(k+1) = ref_phi_k_sim2 + (dt*ref_phi_k_sim2);
    ref_roll_rhonn_sim2(k+2) = ref_phi_k_sim2 + (2*dt*ref_phi_k_sim2);

    ref_pitch_rhonn_sim2(k)   = ref_theta_k_sim2;
    ref_pitch_rhonn_sim2(k+1) = ref_theta_k_sim2 + (dt*ref_theta_k_sim2);
    ref_pitch_rhonn_sim2(k+2) = ref_theta_k_sim2 + (2*dt*ref_theta_k_sim2);

    % =============================================================================
    % --- PASO 6: Control de Rotación (Lazo Interno Roll, Pitch, Yaw) ---
    % =============================================================================
    [e1_roll_sim2(k), e2_roll_sim2(k), u_neural_rot_sim2(2,k+1)] = control_rhonn_feedback_roll(...
        ang_sim2(1,k+1), omega_sim2(1,k+1), ang_sim2(2,k), omega_sim2(2,k), omega_sim2(3,k), ang_nn_sim2(1,k+1), omega_nn_sim2(1,k), ang_nn_sim2(2,k), omega_nn_sim2(2,k), omega_nn_sim2(3,k), w1_roll_sim2(:,k), w2_roll_sim2(:,k), dt, Iwx_roll_sim2, Iwu_roll_sim2, ref_roll_rhonn_sim2(k), ref_roll_rhonn_sim2(k+1), ref_roll_rhonn_sim2(k+2));

    [e1_pitch_sim2(k), e2_pitch_sim2(k), u_neural_rot_sim2(3,k+1)] = control_rhonn_feedback_pitch(...
        ang_sim2(2,k+1), omega_sim2(2,k+1), ang_sim2(1,k), omega_sim2(1,k), omega_sim2(3,k), ang_nn_sim2(2,k+1), omega_nn_sim2(2,k), ang_nn_sim2(1,k), omega_nn_sim2(1,k), omega_nn_sim2(3,k), w1_pitch_sim2(:,k), w2_pitch_sim2(:,k), dt, Iwx_pitch_sim2, Iwu_pitch_sim2, ref_pitch_rhonn_sim2(k), ref_pitch_rhonn_sim2(k+1), ref_pitch_rhonn_sim2(k+2));

    [e1_yaw_sim2(k), e2_yaw_sim2(k), u_neural_rot_sim2(4,k+1)] = control_rhonn_feedback_yaw(...
        ang_sim2(3,k+1), omega_sim2(3,k+1), ang_sim2(1,k), ang_sim2(2,k), omega_sim2(1,k), omega_sim2(2,k), ang_nn_sim2(3,k+1), omega_nn_sim2(3,k), ang_nn_sim2(1,k), ang_nn_sim2(2,k), omega_nn_sim2(1,k), omega_nn_sim2(2,k), w1_yaw_sim2(:,k), w2_yaw_sim2(:,k), dt, Iwx_yaw_sim2, Iwu_yaw_sim2, ref_yaw_sim2(k), ref_yaw_sim2(k+1), ref_yaw_sim2(k+2));

    % Ganancias PID
    kp_roll_sim2 = 4.0; % Ganancia proporcional
    kd_roll_sim2 = 3.5; % Ganancia derivativa

    kp_pitch_sim2 = 4.0; % Ganancia proporcional
    kd_pitch_sim2 = 3.5; % Ganancia derivativa

    % 1. Cálculo de Errores de Posición y Velocidad
    e_roll_sim2 = ref_roll_rhonn_sim2(k) - ang_sim2(1,k+1);
    de_roll_sim2 = (e_roll_sim2 - e_roll_prev_sim2) / dt;
    
    e_pitch_sim2 = ref_pitch_rhonn_sim2(k) - ang_sim2(2,k+1);
    de_pitch_sim2 = (e_pitch_sim2 - e_pitch_prev_sim2) / dt;
    
    % Memoria previa
    e_roll_prev_sim2 = e_roll_sim2; e_pitch_prev_sim2 = e_pitch_sim2;

    % 2. Esfuerzo de Control PD (Fuerza Deseada)
    u_neural_rot_sim2(2,k+1) = m * (kp_roll_sim2 * e_roll_sim2 + kd_roll_sim2 * de_roll_sim2);
    u_neural_rot_sim2(3,k+1) = m * (kp_pitch_sim2 * e_pitch_sim2 + kd_pitch_sim2 * de_pitch_sim2);

    % =====================================================================
    % --- PASO 7: Mezcla y Saturación de Motores ---
    % =====================================================================
    [U_sim2(:,k+1), omega_motors_sim2(:,k+1)] = saturacion_motors(u_neural_rot_sim2(:,k+1), cT, d, cQ);

end
fprintf('Finalizado simulación 2\n');

% =========================================================================
% 4. POST-PROCESAMIENTO (Ajuste de longitudes para graficar)
% =========================================================================
fprintf('Ajustando longitudes de vectores para graficación...\n');

% -------------------------------------------------------------------------
% --- SIMULACIÓN 1 (PID Traslación / RHONN Orientación) ---
% -------------------------------------------------------------------------
e1_x_dynamic_sim1 = [e1_x_dynamic_sim1, e1_x_dynamic_sim1(end)]; e2_x_dynamic_sim1 = [e2_x_dynamic_sim1, e2_x_dynamic_sim1(end)];
e1_ident_x_dynamic_sim1 = [e1_ident_x_dynamic_sim1, e1_ident_x_dynamic_sim1(end)]; e2_ident_x_dynamic_sim1 = [e2_ident_x_dynamic_sim1, e2_ident_x_dynamic_sim1(end)];

e1_y_dynamic_sim1 = [e1_y_dynamic_sim1, e1_y_dynamic_sim1(end)]; e2_y_dynamic_sim1 = [e2_y_dynamic_sim1, e2_y_dynamic_sim1(end)];
e1_ident_y_dynamic_sim1 = [e1_ident_y_dynamic_sim1, e1_ident_y_dynamic_sim1(end)]; e2_ident_y_dynamic_sim1 = [e2_ident_y_dynamic_sim1, e2_ident_y_dynamic_sim1(end)];

e1_z_dynamic_sim1 = [e1_z_dynamic_sim1, e1_z_dynamic_sim1(end)]; e2_z_dynamic_sim1 = [e2_z_dynamic_sim1, e2_z_dynamic_sim1(end)];
e1_ident_z_dynamic_sim1 = [e1_ident_z_dynamic_sim1, e1_ident_z_dynamic_sim1(end)]; e2_ident_z_dynamic_sim1 = [e2_ident_z_dynamic_sim1, e2_ident_z_dynamic_sim1(end)];

e1_roll_sim1 = [e1_roll_sim1, e1_roll_sim1(end)]; e2_roll_sim1 = [e2_roll_sim1, e2_roll_sim1(end)];
e1_ident_roll_sim1 = [e1_ident_roll_sim1, e1_ident_roll_sim1(end)]; e2_ident_roll_sim1 = [e2_ident_roll_sim1, e2_ident_roll_sim1(end)];

e1_pitch_sim1 = [e1_pitch_sim1, e1_pitch_sim1(end)]; e2_pitch_sim1 = [e2_pitch_sim1, e2_pitch_sim1(end)];
e1_ident_pitch_sim1 = [e1_ident_pitch_sim1, e1_ident_pitch_sim1(end)]; e2_ident_pitch_sim1 = [e2_ident_pitch_sim1, e2_ident_pitch_sim1(end)];

e1_yaw_sim1 = [e1_yaw_sim1, e1_yaw_sim1(end)]; e2_yaw_sim1 = [e2_yaw_sim1, e2_yaw_sim1(end)];
e1_ident_yaw_sim1 = [e1_ident_yaw_sim1, e1_ident_yaw_sim1(end)]; e2_ident_yaw_sim1 = [e2_ident_yaw_sim1, e2_ident_yaw_sim1(end)];

% -------------------------------------------------------------------------
% --- SIMULACIÓN 2 (RHONN Traslación / PID Orientación) ---
% -------------------------------------------------------------------------
e1_x_dynamic_sim2 = [e1_x_dynamic_sim2, e1_x_dynamic_sim2(end)]; e2_x_dynamic_sim2 = [e2_x_dynamic_sim2, e2_x_dynamic_sim2(end)];
e1_ident_x_dynamic_sim2 = [e1_ident_x_dynamic_sim2, e1_ident_x_dynamic_sim2(end)]; e2_ident_x_dynamic_sim2 = [e2_ident_x_dynamic_sim2, e2_ident_x_dynamic_sim2(end)];

e1_y_dynamic_sim2 = [e1_y_dynamic_sim2, e1_y_dynamic_sim2(end)]; e2_y_dynamic_sim2 = [e2_y_dynamic_sim2, e2_y_dynamic_sim2(end)];
e1_ident_y_dynamic_sim2 = [e1_ident_y_dynamic_sim2, e1_ident_y_dynamic_sim2(end)]; e2_ident_y_dynamic_sim2 = [e2_ident_y_dynamic_sim2, e2_ident_y_dynamic_sim2(end)];

e1_z_dynamic_sim2 = [e1_z_dynamic_sim2, e1_z_dynamic_sim2(end)]; e2_z_dynamic_sim2 = [e2_z_dynamic_sim2, e2_z_dynamic_sim2(end)];
e1_ident_z_dynamic_sim2 = [e1_ident_z_dynamic_sim2, e1_ident_z_dynamic_sim2(end)]; e2_ident_z_dynamic_sim2 = [e2_ident_z_dynamic_sim2, e2_ident_z_dynamic_sim2(end)];

e1_roll_sim2 = [e1_roll_sim2, e1_roll_sim2(end)]; e2_roll_sim2 = [e2_roll_sim2, e2_roll_sim2(end)];
e1_ident_roll_sim2 = [e1_ident_roll_sim2, e1_ident_roll_sim2(end)]; e2_ident_roll_sim2 = [e2_ident_roll_sim2, e2_ident_roll_sim2(end)];

e1_pitch_sim2 = [e1_pitch_sim2, e1_pitch_sim2(end)]; e2_pitch_sim2 = [e2_pitch_sim2, e2_pitch_sim2(end)];
e1_ident_pitch_sim2 = [e1_ident_pitch_sim2, e1_ident_pitch_sim2(end)]; e2_ident_pitch_sim2 = [e2_ident_pitch_sim2, e2_ident_pitch_sim2(end)];

e1_yaw_sim2 = [e1_yaw_sim2, e1_yaw_sim2(end)]; e2_yaw_sim2 = [e2_yaw_sim2, e2_yaw_sim2(end)];
e1_ident_yaw_sim2 = [e1_ident_yaw_sim2, e1_ident_yaw_sim2(end)]; e2_ident_yaw_sim2 = [e2_ident_yaw_sim2, e2_ident_yaw_sim2(end)];

% =========================================================================
% 5. VISUALIZACIÓN Y RESULTADOS COMPARATIVOS
% =========================================================================
fprintf('Generando figuras comparativas de rendimiento...\n');

% Configuración general de graficación
t_plot = t(1:N+1);
lw_ref = 2;      % Grosor para las referencias
lw_real = 1.5;   % Grosor para las respuestas reales

% Paleta de colores profesionales
c_ref = 'k--';       % Referencias (Negro punteado)
c_sim1 = '#0072BD';  % Simulación 1 (Azul): PID Traslación / RHONN Orientación
c_sim2 = '#D95319';  % Simulación 2 (Naranja): RHONN Traslación / PID Orientación

% -------------------------------------------------------------------------
% FIGURA 1: DINÁMICA DE TRASLACIÓN (X, Y, Z)
% -------------------------------------------------------------------------
figure('Name', 'Figura 1: Dinámica de Traslación Global', 'Position', [100, 100, 850, 700]);

subplot(3,1,1);
plot(t_plot, target_x(1:N+1), c_ref, 'LineWidth', lw_ref); hold on;
plot(t_plot, x_sim1(1:N+1), 'Color', c_sim1, 'LineWidth', lw_real);
plot(t_plot, x_sim2(1:N+1), 'Color', c_sim2, 'LineWidth', lw_real);
ylabel('X [m]'); title('Posición Espacial en el Eje X'); grid on;
legend('Referencia', 'Sim 1 (Lazo Ext: PID)', 'Sim 2 (Lazo Ext: RHONN)', 'Location', 'best');

subplot(3,1,2);
plot(t_plot, target_y(1:N+1), c_ref, 'LineWidth', lw_ref); hold on;
plot(t_plot, y_sim1(1:N+1), 'Color', c_sim1, 'LineWidth', lw_real);
plot(t_plot, y_sim2(1:N+1), 'Color', c_sim2, 'LineWidth', lw_real);
ylabel('Y [m]'); title('Posición Espacial en el Eje Y'); grid on;

subplot(3,1,3);
plot(t_plot, target_z(1:N+1), c_ref, 'LineWidth', lw_ref); hold on;
plot(t_plot, z_sim1(1:N+1), 'Color', c_sim1, 'LineWidth', lw_real);
plot(t_plot, z_sim2(1:N+1), 'Color', c_sim2, 'LineWidth', lw_real);
ylabel('Z [m]'); xlabel('Tiempo [s]'); title('Posición Espacial en el Eje Z (Altitud)'); grid on;

% -------------------------------------------------------------------------
% FIGURA 2: DINÁMICA DE ORIENTACIÓN (Roll, Pitch, Yaw)
% -------------------------------------------------------------------------
figure('Name', 'Figura 2: Dinámica de Orientación (Ángulos de Euler)', 'Position', [150, 150, 850, 700]);

subplot(3,1,1);
plot(t_plot, rad2deg(ref_roll_rhonn_sim1(1:N+1)), 'k:', 'LineWidth', 1.2); hold on;
plot(t_plot, rad2deg(ang_sim1(1,1:N+1)), 'Color', c_sim1, 'LineWidth', lw_real);
plot(t_plot, rad2deg(ref_roll_rhonn_sim2(1:N+1)), 'r:', 'LineWidth', 1.2);
plot(t_plot, rad2deg(ang_sim2(1,1:N+1)), 'Color', c_sim2, 'LineWidth', lw_real);
ylabel('Roll \phi [°]'); title('Respuesta de Actitud: Roll (Alabeo)'); grid on;
legend('Ref Sim 1', 'Sim 1 (Lazo Int: RHONN)', 'Ref Sim 2', 'Sim 2 (Lazo Int: PID)', 'Location', 'best');

subplot(3,1,2);
plot(t_plot, rad2deg(ref_pitch_rhonn_sim1(1:N+1)), 'k:', 'LineWidth', 1.2); hold on;
plot(t_plot, rad2deg(ang_sim1(2,1:N+1)), 'Color', c_sim1, 'LineWidth', lw_real);
plot(t_plot, rad2deg(ref_pitch_rhonn_sim2(1:N+1)), 'r:', 'LineWidth', 1.2);
plot(t_plot, rad2deg(ang_sim2(2,1:N+1)), 'Color', c_sim2, 'LineWidth', lw_real);
ylabel('Pitch \theta [°]'); title('Respuesta de Actitud: Pitch (Cabeceo)'); grid on;

subplot(3,1,3);
plot(t_plot, rad2deg(ang_sim1(3,1:N+1)), 'Color', c_sim1, 'LineWidth', lw_real); hold on;
plot(t_plot, rad2deg(ang_sim2(3,1:N+1)), 'Color', c_sim2, 'LineWidth', lw_real);
ylabel('Yaw \psi [°]'); xlabel('Tiempo [s]'); title('Respuesta de Actitud: Yaw (Guiñada)'); grid on;

% -------------------------------------------------------------------------
% FIGURA 3: VELOCIDAD DE LOS ROTORES (RPM)
% -------------------------------------------------------------------------
figure('Name', 'Figura 3: Régimen de Operación de Motores (RPM)', 'Position', [200, 200, 950, 600]);

for i = 1:4
    subplot(2,2,i);
    plot(t_plot, omega_motors_sim1(i,1:N+1), 'Color', c_sim1, 'LineWidth', lw_real); hold on;
    plot(t_plot, omega_motors_sim2(i,1:N+1), 'Color', c_sim2, 'LineWidth', lw_real);
    ylabel(sprintf('\Omega %d [rpm]', i)); xlabel('Tiempo [s]'); 
    title(sprintf('Velocidad de Giro - Motor %d', i)); grid on;
    if i == 1
        legend('Sim 1 (RHONN Actitud)', 'Sim 2 (PID Actitud)', 'Location', 'best');
    end
end

% -------------------------------------------------------------------------
% FIGURA 4: ERRORES DE SEGUIMIENTO PUROS (e1)
% -------------------------------------------------------------------------
figure('Name', 'Figura 4: Errores de Seguimiento Puros (e1)', 'Position', [250, 250, 950, 600]);

% e1 Traslación X
subplot(2,2,1);
plot(t_plot, e1_x_dynamic_sim1(1:N+1), 'Color', c_sim1, 'LineWidth', lw_real); hold on;
plot(t_plot, e1_x_dynamic_sim2(1:N+1), 'Color', c_sim2, 'LineWidth', lw_real);
ylabel('Error X [m]'); title('Error de Seguimiento Dinámico (X)'); grid on;
legend('Sim 1 (Lazo Ext: PID)', 'Sim 2 (Lazo Ext: RHONN)', 'Location', 'best');

% e1 Traslación Y
subplot(2,2,2);
plot(t_plot, e1_y_dynamic_sim1(1:N+1), 'Color', c_sim1, 'LineWidth', lw_real); hold on;
plot(t_plot, e1_y_dynamic_sim2(1:N+1), 'Color', c_sim2, 'LineWidth', lw_real);
ylabel('Error Y [m]'); title('Error de Seguimiento Dinámico (Y)'); grid on;

% e1 Orientación Roll
subplot(2,2,3);
plot(t_plot, rad2deg(e1_roll_sim1(1:N+1)), 'Color', c_sim1, 'LineWidth', lw_real); hold on;
plot(t_plot, rad2deg(e1_roll_sim2(1:N+1)), 'Color', c_sim2, 'LineWidth', lw_real);
ylabel('Error Roll [°]'); xlabel('Tiempo [s]'); title('Error de Seguimiento en Actitud (Roll)'); grid on;

% e1 Orientación Pitch
subplot(2,2,4);
plot(t_plot, rad2deg(e1_pitch_sim1(1:N+1)), 'Color', c_sim1, 'LineWidth', lw_real); hold on;
plot(t_plot, rad2deg(e1_pitch_sim2(1:N+1)), 'Color', c_sim2, 'LineWidth', lw_real);
ylabel('Error Pitch [°]'); xlabel('Tiempo [s]'); title('Error de Seguimiento en Actitud (Pitch)'); grid on;