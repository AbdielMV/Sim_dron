% =========================================================================
% SIMULADOR UAV - CONTROL NEURONAL RHONN EKF
% =========================================================================
clear all; close all; clc;

% Agregar subcarpetas al path de MATLAB para que encuentre las funciones
addpath('parameters', 'dynamics', 'control_rhonn', 'utils', 'visualization', 'efk', 'rhonn_model');

% 1. Cargar Parámetros e Inicialización (Masa, Inercias, EKF, Condiciones Iniciales)
% (Asumimos que esta función devuelve todas las variables necesarias)
run('init_system.m'); 

% 2. Generación de Referencias
[t_ref, ref_total, dref_total, ddref_total] = build_ref(dt, Tf, 1);
ref_roll = ones(1,N+2)*deg2rad(0);
ref_pitch = ones(1,N+2)*deg2rad(0);
ref_yaw = ones(1,N+2)*deg2rad(0);
target_x = ref_total(1, :); % Solo la componente X para el control de posición X
target_y = ref_total(2, :); % Solo la componente Y para el control de posición Y
vx_target = dref_total(1, :);
vy_target = dref_total(2, :);
target_z = ref_total(3, :); % Solo la componente Z para el control de altura Z

% ==========================================
% 3. BUCLE PRINCIPAL DE SIMULACIÓN
% ==========================================
fprintf('Iniciando simulación. Cambio de masa en t=%.2fs...\n', t(cuarto_tiempo));

for k = 1:N
    
    % --- PASO 1: Dinámica Física Real ---
    m_k = m_real(k);

    [z(k+1), vz(k+1), x(k+1), vx(k+1), y(k+1), vy(k+1)] = translational_dynamic_u(x(k), vx(k), y(k), vy(k), z(k), vz(k), U(1,k), m_k, g, dt, k_wind, ang(:,k));

    [ang(:,k+1), omega(:,k+1)] = rotational_dynamics(ang(:,k), omega(:,k), U(:,k), Inertia, dt);

    % --- PASO 2: MODELO RHONN (Usa Masa NOMINAL Fija) ---

    [xn(k+1), vxn(k+1), H_x_dynamic, Iwu_x_dynamic, Iwx_x_dynamic] = rhonn_model_x_dynamic(x(k), vx(k), ...
        ang(1,k), ang(3,k), ang(2,k), U(1,k), w1_x_dynamic(:,k), w2_x_dynamic(:,k), dt, m);
    
    [yn(k+1), vyn(k+1), H_y_dynamic, Iwu_y_dynamic, Iwx_y_dynamic] = rhonn_model_y_dynamic(x(k), vx(k), ...
        ang(1,k), ang(3,k), ang(2,k), U(1,k), w1_y_dynamic(:,k), w2_y_dynamic(:,k), dt, m);

    [zn(k+1), vzn(k+1), H_z_dynamic, Iwu_z_dynamic, Iwx_z_dynamic] = rhonn_model_z_dynamic(z(k), vz(k), ... 
        ang(1,k), ang(2,k), U(1,k), w1_z_dynamic(:,k), w2_z_dynamic(:,k), dt, g, m);

    [ang_nn(1,k+1), omega_nn(1,k+1), H_roll, Iwu_roll, Iwx_roll] = rhonn_model_roll_dynamic(ang(1,k), ...
        omega(1,k), ang(2,k), omega(2,k), omega(3,k), U(2,k), w1_roll(:,k), w2_roll(:,k), dt);

    [ang_nn(2,k+1), omega_nn(2,k+1), H_pitch, Iwu_pitch, Iwx_pitch] = rhonn_model_pitch_dynamic(ang(2,k), ...
        omega(2,k), ang(1,k), omega(1,k), omega(3,k), U(3,k), w1_pitch(:,k), w2_pitch(:,k), dt);

    [ang_nn(3,k+1), omega_nn(3,k+1), H_yaw, Iwu_yaw, Iwx_yaw] = rhonn_model_yaw_dynamic(ang(3,k), ...
        omega(3,k), ang(1,k), ang(2,k), omega(1,k), omega(2,k), U(4,k), w1_yaw(:,k), w2_yaw(:,k), dt);


    % --- PASO 3: ENTRENAMIENTO EKF ---

    [w1_x_dynamic(:,k+1),w2_x_dynamic(:,k+1),p1_x_dynamic(:,:,k+1),p2_x_dynamic(:,:,k+1), ...
        e1_ident_x_dynamic(k),e2_ident_x_dynamic(k)] = efk_training_x_dynamic(H_x_dynamic,x(k),vx(k),xn(k),vxn(k),w1_x_dynamic(:,k), ...
        w2_x_dynamic(:,k),p1_x_dynamic(:,:,k),p2_x_dynamic(:,:,k),Q1_x_dynamic(:,:,1), ...
        Q2_x_dynamic(:,:,1), R1_x_dynamic, R2_x_dynamic);
    
    [w1_y_dynamic(:,k+1),w2_y_dynamic(:,k+1),p1_y_dynamic(:,:,k+1),p2_y_dynamic(:,:,k+1), ...
        e1_ident_y_dynamic(k),e2_ident_y_dynamic(k)] = efk_training_y_dynamic(H_y_dynamic,y(k),vy(k),yn(k),vyn(k),w1_y_dynamic(:,k), ...
        w2_y_dynamic(:,k),p1_y_dynamic(:,:,k),p2_y_dynamic(:,:,k),Q1_y_dynamic(:,:,1), ...
        Q2_y_dynamic(:,:,1), R1_y_dynamic, R2_y_dynamic);

    [w1_z_dynamic(:,k+1),w2_z_dynamic(:,k+1),p1_z_dynamic(:,:,k+1),p2_z_dynamic(:,:,k+1), ...
        e1_ident_z_dynamic(k),e2_ident_z_dynamic(k)] = efk_training_z_dynamic(H_z_dynamic,z(k),vz(k),zn(k),vzn(k),w1_z_dynamic(:,k), ...
        w2_z_dynamic(:,k),p1_z_dynamic(:,:,k),p2_z_dynamic(:,:,k),Q1_z_dynamic(:,:,1), ...
        Q2_z_dynamic(:,:,1), R1_z_dynamic, R2_z_dynamic);

    [w1_roll(:,k+1),w2_roll(:,k+1),p1_roll(:,:,k+1),p2_roll(:,:,k+1), ...
        e1_ident_roll(k),e2_ident_roll(k)] = efk_training_roll(H_roll,ang(1,k),omega(1,k),ang_nn(1,k),omega_nn(1,k), ...
        w1_roll(:,k), w2_roll(:,k), p1_roll(:,:,k), p2_roll(:,:,k), Q1_roll(:,:,1), Q2_roll(:,:,1), R1_roll, R2_roll);

    [w1_pitch(:,k+1),w2_pitch(:,k+1),p1_pitch(:,:,k+1),p2_pitch(:,:,k+1), ...
        e1_ident_pitch(k),e2_ident_pitch(k)] = efk_training_pitch(H_pitch,ang(2,k),omega(2,k),ang_nn(2,k),omega_nn(2,k), ...
        w1_pitch(:,k), w2_pitch(:,k), p1_pitch(:,:,k), p2_pitch(:,:,k), Q1_pitch(:,:,1), Q2_pitch(:,:,1), R1_pitch, R2_pitch);

    [w1_yaw(:,k+1),w2_yaw(:,k+1),p1_yaw(:,:,k+1),p2_yaw(:,:,k+1), ...
        e1_ident_yaw(k),e2_ident_yaw(k)] = efk_training_yaw(H_yaw,ang(3,k),omega(3,k),ang_nn(3,k),omega_nn(3,k), ...
        w1_yaw(:,k), w2_yaw(:,k), p1_yaw(:,:,k), p2_yaw(:,:,k), Q1_yaw(:,:,1), Q2_yaw(:,:,1), R1_yaw, R2_yaw);

    % ==========================================================
    % --- PASO 4.5: CONTROL DE POSICIÓN X-Y (Lazo Externo) ---
    % ==========================================================
    
    % % 1. Definir la Referencia (Destino constante)
    % target_x = 5.0; % Queremos ir a X = 5 metros
    % target_y = 5.0; % Queremos ir a Y = 5 metros
    % vx_target = 0;  % Queremos frenar al llegar
    % vy_target = 0;

    %{
    % 1. Configuración del Círculo
    t_actual = (k-1) * dt;
    Radio = 10.0;             % Amplitud (radio) de 10 metros
    Frecuencia = 0.05;        % Hz (0.05 Hz = 1 vuelta cada 20 segundos, muy lento y estable)
    Omega = 2 * pi * Frecuencia;

    % Posición deseada actual (Ecuaciones paramétricas)
    target_x = Radio * cos(Omega * t_actual);
    target_y = Radio * sin(Omega * t_actual);
    
    % Velocidad deseada actual (Derivadas analíticas para no atrasarse)
    vx_target = -Radio * Omega * sin(Omega * t_actual);
    vy_target =  Radio * Omega * cos(Omega * t_actual);
    %}

    %{
    % Referencia de Yaw (Gira lentamente)
    ref_yaw(k) = 0.2*t_ref(k)*pi; % Referencia de Yaw (Gira lentamente)
    ref_yaw(k+1) = 0.2*t_ref(k+1)*pi; % Referencia de Yaw (Gira lentamente)
    ref_yaw(k+2) = 0.2*t_ref(k+2)*pi; % Referencia de Yaw (Gira lentamente)
    %}

    %{
    % Establecer la referencia de posición actual (puede ser fija o dinámica)
    target_x = ref_total(1, k);
    target_y = ref_total(2, k);
    
    vx_target = dref_total(1, k);
    vy_target = dref_total(2, k);
    %}
    
    % 2. Llamar al controlador PID que calcula los ángulos necesarios
    [ref_phi_calc(k), ref_theta_calc(k)] = control_posicion_xy_pid(...
        x(k), y(k), vx(k), vy(k), ...       % Estados físicos actuales
        target_x(k), target_y(k), ...             % Posición deseada
        vx_target(k), vy_target(k), ...           % Velocidad deseada (Feedforward)
        ang(3,k));                          % Yaw (psi) actual para compensación

    % 3. Inyectar estos ángulos como la "referencia" para el lazo interno (RHONN)
    vector_indices = k : min(k+2, length(ref_roll));
    ref_roll(vector_indices)  = ref_phi_calc(k);
    ref_pitch(vector_indices) = ref_theta_calc(k);

    % ============CONTROL TRASLACIONAL (X-Y) - Lazo Externo con RHONN + EFK =================
    [e1_z_dynamic(k), e2_z_dynamic(k), u_neural_rot(1,k+1)] = control_rhonn_feedback_z_dynamic(...
        z(k), vz(k), ang(1,k), ang(2,k), zn(k+1), w1_z_dynamic(:,k), w2_z_dynamic(:,k), dt, Iwx_z_dynamic, Iwu_z_dynamic, ...
        target_z(k), target_z(k+1), target_z(k+2), m, g);

    [e1_x_dynamic(k), e2_x_dynamic(k), ux_des(k+1)] = control_rhonn_feedback_x_dynamic(...
        x(k), vx(k), ang(1,k), ang(3,k), ang(2,k), xn(k+1), w1_x_dynamic(:,k), w2_x_dynamic(:,k), dt, Iwx_x_dynamic, Iwu_x_dynamic, ...
        target_x(k), target_x(k+1), target_x(k+2), m, g);
    
    [e1_y_dynamic(k), e2_y_dynamic(k), uy_des(k+1)] = control_rhonn_feedback_y_dynamic(...
        y(k), vy(k), ang(1,k), ang(3,k), ang(2,k), yn(k+1), w1_y_dynamic(:,k), w2_y_dynamic(:,k), dt, Iwx_y_dynamic, Iwu_y_dynamic, ...
        target_y(k), target_y(k+1), target_y(k+2), m, g);
    
    [ref_phi_calc_rhonn, ref_theta_calc_rhonn] = compensator(ux_des(k+1), uy_des(k+1), ang(3,k));
    vector_indices = k : min(k+2, length(ref_roll));
    ref_roll_rhonn(vector_indices)  = ref_phi_calc_rhonn;
    ref_pitch_rhonn(vector_indices) = ref_theta_calc_rhonn;

    % ============CONTROL ROTACIÓN (Ángulos) - Lazo Interno con RHONN + EKF================
    [e1_roll(k), e2_roll(k), u_neural_rot(2,k+1)] = control_rhonn_feedback_roll(ang(1,k), omega(1,k), ang(2,k), omega(2,k),...
        omega(3,k), ang_nn(1,k), omega_nn(1,k), ang_nn(2,k), omega_nn(2,k), omega_nn(3,k), w1_roll(:,k), w2_roll(:,k), dt, ...
        Iwx_roll, Iwu_roll, ref_roll_rhonn(k), ref_roll_rhonn(k+1), ref_roll_rhonn(k+2));

    [e1_pitch(k), e2_pitch(k), u_neural_rot(3,k+1)] = control_rhonn_feedback_pitch(ang(2,k), omega(2,k), ang(1,k), omega(1,k),...
        omega(3,k), ang_nn(2,k), omega_nn(2,k), ang_nn(1,k), omega_nn(1,k), omega_nn(3,k), w1_pitch(:,k), w2_pitch(:,k), dt, ...
        Iwx_pitch, Iwu_pitch, ref_pitch_rhonn(k), ref_pitch_rhonn(k+1), ref_pitch_rhonn(k+2));

    [e1_yaw(k), e2_yaw(k), u_neural_rot(4,k+1)] = control_rhonn_feedback_yaw(ang(3,k), omega(3,k), ang(1,k), ang(2,k),...
        omega(1,k), omega(2,k), ang_nn(3,k), omega_nn(3,k), ang_nn(1,k), ang_nn(2,k), omega_nn(1,k), omega_nn(2,k), w1_yaw(:,k), w2_yaw(:,k), dt, ...
        Iwx_yaw, Iwu_yaw, ref_yaw(k), ref_yaw(k+1), ref_yaw(k+2));

    % --- PASO 6: Saturación de los motores ---

    [U(:,k+1), omega_motors(:,k+1)] = saturacion_motors(u_neural_rot(:,k+1), cT, d, cQ);

end

% Ajuste de longitud de errores (para trazar junto con t)
e1_x_dynamic = [e1_x_dynamic, e1_x_dynamic(end)];
e2_x_dynamic = [e2_x_dynamic, e2_x_dynamic(end)];
e1_ident_x_dynamic = [e1_ident_x_dynamic, e1_ident_x_dynamic(end)];
e2_ident_x_dynamic = [e2_ident_x_dynamic, e2_ident_x_dynamic(end)];

e1_y_dynamic = [e1_y_dynamic, e1_y_dynamic(end)];
e2_y_dynamic = [e2_y_dynamic, e2_y_dynamic(end)];
e1_ident_y_dynamic = [e1_ident_y_dynamic, e1_ident_y_dynamic(end)];
e2_ident_y_dynamic = [e2_ident_y_dynamic, e2_ident_y_dynamic(end)];

e1_z_dynamic = [e1_z_dynamic, e1_z_dynamic(end)];
e2_z_dynamic = [e2_z_dynamic, e2_z_dynamic(end)];
e1_ident_z_dynamic = [e1_ident_z_dynamic, e1_ident_z_dynamic(end)];
e2_ident_z_dynamic = [e2_ident_z_dynamic, e2_ident_z_dynamic(end)];

e1_roll = [e1_roll, e1_roll(end)];
e2_roll = [e2_roll, e2_roll(end)];
e1_ident_roll = [e1_ident_roll, e1_ident_roll(end)];
e2_ident_roll = [e2_ident_roll, e2_ident_roll(end)];

e1_pitch = [e1_pitch, e1_pitch(end)];
e2_pitch = [e2_pitch, e2_pitch(end)];
e1_ident_pitch = [e1_ident_pitch, e1_ident_pitch(end)];
e2_ident_pitch = [e2_ident_pitch, e2_ident_pitch(end)];

e1_yaw = [e1_yaw, e1_yaw(end)];
e2_yaw = [e2_yaw, e2_yaw(end)];
e1_ident_yaw = [e1_ident_yaw, e1_ident_yaw(end)];
e2_ident_yaw = [e2_ident_yaw, e2_ident_yaw(end)];

% ==========================================
% 4. VISUALIZACIÓN Y RESULTADOS
% ==========================================
% Llamamos a un script externo para no saturar este archivo
plot_results;  
%animate_drone; % Descomentar para generar el video 3D