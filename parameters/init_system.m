% Parametros Dron (Traslacional + Rotacional)
g  = 9.81;
m  = 0.468;
L  = 0.25;      % Longitud del brazo (m)
k_wind = 2;

% Parametros de motores
cT = 2.980e-6; % Motor Speed to Force Constant
cQ = 1.140e-7; % Motor Speed to Reactive Torque Constant
d = 0.225; % Longitud del centro a los rotores

% Inercias (kg*m^2) - Valores aproximados para m=0.5kg
Ix = 4.856e-3; 
Iy = 4.856e-3; 
Iz = 8.801e-3;
Inertia = [Ix, Iy, Iz]; % Vector para pasar a la función

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ==========================================
% 2. CONFIGURACIÓN DE SIMULACIÓN Y MASAS
% ==========================================
dt = 1e-3;
Tf = 30;       % Tiempo final
t  = 0:dt:Tf;
N  = numel(t) - 1;
cuarto_tiempo = round(N / 2); % Encuentra el índice K en la mitad

% Masa Nominal (Usada por el Controlador y la RHONN)
m_nominal = 0.468;

% Masa Real (Usada en la Dinámica Física)
m_real = zeros(1, N+1);
m_real(1:cuarto_tiempo) = m_nominal;
m_real(cuarto_tiempo+1:N+1) = m_nominal * 1; % ¡Aumento del 10% a la mitad!

% Reemplaza la inicialización de masa en tu código principal con m_nominal
m = m_nominal;
g = 9.81; % Asegurarte que g está definido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Estados y control de altura
x = zeros(1, N+1);
vx = zeros(1, N+1);
xn = zeros(1, N+1);
vxn = zeros(1, N+1);

y = zeros(1, N+1);
vy = zeros(1, N+1);
yn = zeros(1, N+1);
vyn = zeros(1, N+1);

z = zeros(1, N+1);
vz = zeros(1, N+1);
zn = zeros(1, N+1);
vzn = zeros(1, N+1);

% --- Estados Rotacionales ---
% Ángulos de Euler [roll (phi); pitch (theta); yaw (psi)]
ang = zeros(3, N+1);

ang_nn = zeros(3, N+1);

% Velocidades angulares del cuerpo [p (wx); q (wy); r (wz)]
omega = zeros(3, N+1);

omega_nn = zeros(3, N+1);

% Entradas de Control Rotacional (Momentos) [u2; u3; u4]
% Por ahora en 0. Si no tienes controlador de actitud, el dron no girará.
u_rot = zeros(4, N+1);

u_neural_rot = zeros(4, N+1);
ux_des = zeros(1, N+1);
uy_des = zeros(1, N+1);

U = zeros(4, N+1);

% Condiciones iniciales x1 y x2 del sistema
z(1) = 0;
vz(1) = 0;

% Inicialización ley de control
u_z_dynamic = m*g;

U(1,1) = u_z_dynamic;

% Condiciones iniciales x1 y x2 de la red
zn(1) = 0;
vzn(1) = 0;

% Inicialización de Roll y Pitch
ang(1,1) = deg2rad(10);
ang(2,1) = deg2rad(-10);

ang_nn(1,1) = ang(1,1);
ang_nn(2,1) = ang(2,1);

% EFK SIZE X DYNAMIC
dim_set_1_x_dynamic = 2; %C1
dim_set_2_x_dynamic = 3; %C2

% EFK SIZE Y DYNAMIC
dim_set_1_y_dynamic = 2; %C1
dim_set_2_y_dynamic = 3; %C2

% EFK SIZE Z DYNAMIC
dim_set_1_z_dynamic = 2; %C1
dim_set_2_z_dynamic = 3; %C2

% EFK SIZE ROLL
dim_set_1_roll = 3; %C1
dim_set_2_roll = 4; %C2

% EFK SIZE PITCH
dim_set_1_pitch = 3; %C1
dim_set_2_pitch = 4; %C2

% EFK SIZE YAW
dim_set_1_yaw = 3; %C1
dim_set_2_yaw = 3; %C2

% Inicialización Pesos Sinápticos X 
w1_x_dynamic = zeros(dim_set_1_x_dynamic, N+1);
w2_x_dynamic = zeros(dim_set_2_x_dynamic, N+1);

% Inicialización Pesos Sinápticos Y
w1_y_dynamic = zeros(dim_set_1_y_dynamic, N+1);
w2_y_dynamic = zeros(dim_set_2_y_dynamic, N+1);

% Inicialización Pesos Sinápticos Z 
w1_z_dynamic = zeros(dim_set_1_z_dynamic, N+1);
w2_z_dynamic = zeros(dim_set_2_z_dynamic, N+1);

%Inicialización Pesos Sinápticos Roll
w1_roll = zeros(dim_set_1_roll, N+1);
w2_roll = zeros(dim_set_2_roll, N+1);

%Inicialización Pesos Sinápticos Pitch
w1_pitch = zeros(dim_set_1_pitch, N+1);
w2_pitch = zeros(dim_set_2_pitch, N+1);

%Inicialización Pesos Sinápticos Yaw
w1_yaw = zeros(dim_set_1_yaw, N+1);
w2_yaw = zeros(dim_set_2_yaw, N+1);

% Inicialización p1 y p2 (Covarianza del error de predicción)
% Un valor grande en la diagonal de $P$ indica alta incertidumbre,
% esto hace que los pesos cambien drásticamente con los primeros errores de predicción
p1_x_dynamic = zeros(dim_set_1_x_dynamic, dim_set_1_x_dynamic, N+1);
p2_x_dynamic = zeros(dim_set_2_x_dynamic, dim_set_2_x_dynamic, N+1);
p1_x_dynamic(:,:,1) = eye(dim_set_1_x_dynamic) * 1e5;
p2_x_dynamic(:,:,1) = eye(dim_set_2_x_dynamic) * 1e3;

p1_y_dynamic = zeros(dim_set_1_y_dynamic, dim_set_1_y_dynamic, N+1);
p2_y_dynamic = zeros(dim_set_2_y_dynamic, dim_set_2_y_dynamic, N+1);
p1_y_dynamic(:,:,1) = eye(dim_set_1_y_dynamic) * 1e5;
p2_y_dynamic(:,:,1) = eye(dim_set_2_y_dynamic) * 1e3;

p1_z_dynamic = zeros(dim_set_1_z_dynamic, dim_set_1_z_dynamic, N+1);
p2_z_dynamic = zeros(dim_set_2_z_dynamic, dim_set_2_z_dynamic, N+1);
p1_z_dynamic(:,:,1) = eye(dim_set_1_z_dynamic) * 1e5;
p2_z_dynamic(:,:,1) = eye(dim_set_2_z_dynamic) * 1e3;

p1_roll = zeros(dim_set_1_roll, dim_set_1_roll, N+1);
p2_roll = zeros(dim_set_2_roll, dim_set_2_roll, N+1);
p1_roll(:,:,1) = eye(dim_set_1_roll) * 1e5;
p2_roll(:,:,1) = eye(dim_set_2_roll) * 1e3;

p1_pitch = zeros(dim_set_1_pitch, dim_set_1_pitch, N+1);
p2_pitch = zeros(dim_set_2_pitch, dim_set_2_pitch, N+1);
p1_pitch(:,:,1) = eye(dim_set_1_pitch) * 1e5;
p2_pitch(:,:,1) = eye(dim_set_2_pitch) * 1e3;

p1_yaw = zeros(dim_set_1_yaw, dim_set_1_yaw, N+1);
p2_yaw = zeros(dim_set_2_yaw, dim_set_2_yaw, N+1);
p1_yaw(:,:,1) = eye(dim_set_1_yaw) * 1e5;
p2_yaw(:,:,1) = eye(dim_set_2_yaw) * 1e3;

% Inicialización Q1 y Q2 (Covarianza de ruido del proceso)
% Esta matriz representa la incertidumbre en la dinámica de los propios pesos.
% Asume que los pesos podrían cambiar estocásticamente de un paso a otro.
% Si $Q$ es demasiado grande, los pesos nunca convergerán y oscilarán (ruido)
Q1_x_dynamic = zeros(dim_set_1_x_dynamic, dim_set_1_x_dynamic, N+1);
Q2_x_dynamic = zeros(dim_set_2_x_dynamic, dim_set_2_x_dynamic, N+1);
Q1_x_dynamic(:,:,1) = eye(dim_set_1_x_dynamic) * 1e-5;
Q2_x_dynamic(:,:,1) = eye(dim_set_2_x_dynamic) * 3e-5;

Q1_y_dynamic = zeros(dim_set_1_y_dynamic, dim_set_1_y_dynamic, N+1);
Q2_y_dynamic = zeros(dim_set_2_y_dynamic, dim_set_2_y_dynamic, N+1);
Q1_y_dynamic(:,:,1) = eye(dim_set_1_y_dynamic) * 1e-5;
Q2_y_dynamic(:,:,1) = eye(dim_set_2_y_dynamic) * 3e-5;

Q1_z_dynamic = zeros(dim_set_1_z_dynamic, dim_set_1_z_dynamic, N+1);
Q2_z_dynamic = zeros(dim_set_2_z_dynamic, dim_set_2_z_dynamic, N+1);
Q1_z_dynamic(:,:,1) = eye(dim_set_1_z_dynamic) * 1e-5;
Q2_z_dynamic(:,:,1) = eye(dim_set_2_z_dynamic) * 3e-5;

Q1_roll = zeros(dim_set_1_roll, dim_set_1_roll, N+1);
Q2_roll = zeros(dim_set_2_roll, dim_set_2_roll, N+1);
Q1_roll(:,:,1) = eye(dim_set_1_roll) * 1e-5;
Q2_roll(:,:,1) = eye(dim_set_2_roll) * 3e-5;

Q1_pitch = zeros(dim_set_1_pitch, dim_set_1_pitch, N+1);
Q2_pitch = zeros(dim_set_2_pitch, dim_set_2_pitch, N+1);
Q1_pitch(:,:,1) = eye(dim_set_1_pitch) * 1e-5;
Q2_pitch(:,:,1) = eye(dim_set_2_pitch) * 0.1;

Q1_yaw = zeros(dim_set_1_yaw, dim_set_1_yaw, N+1);
Q2_yaw= zeros(dim_set_2_yaw, dim_set_2_yaw, N+1);
Q1_yaw(:,:,1) = eye(dim_set_1_yaw) * 1e-5;
Q2_yaw(:,:,1) = eye(dim_set_2_yaw) * 0.1;

% Inicialización R1 y R2 (Covarianza del ruido de medición)
% Representa la incertidumbre o "confianza" que tienes en los datos con los
% que estás entrenando la red.
R1_x_dynamic = 1e-8;
R2_x_dynamic = 1e-6;

R1_y_dynamic = 1e-8;
R2_y_dynamic = 1e-6;

R1_z_dynamic = 1e-8;
R2_z_dynamic = 1e-6;

R1_roll = 1e-8;
R2_roll = 1e-6;

R1_pitch = 1e-8;
R2_pitch = 1e-6;

R1_yaw = 1e-8;
R2_yaw = 1e-6;

% Inicialización de estados para la simulación continua (RK4)
M = 10;                % Número de sub-pasos para la planta
dt_cont = dt / M;      % Paso de tiempo "continuo"
S = [x(1); vx(1); y(1); vy(1); z(1); vz(1); ang(1,1); ang(2,1); ang(3,1); omega(1,1); omega(2,1); omega(3,1)];

% Inicialización de errores para graficar e identificación
e1_x_dynamic = zeros(1, N);        % errores para graficar
e2_x_dynamic = zeros(1, N);
e1_ident_x_dynamic = zeros(1,N);  %errores de identificacion
e2_ident_x_dynamic = zeros(1,N);

e1_y_dynamic = zeros(1, N);        % errores para graficar
e2_y_dynamic = zeros(1, N);
e1_ident_y_dynamic = zeros(1,N);  %errores de identificacion
e2_ident_y_dynamic = zeros(1,N);

e1_z_dynamic = zeros(1, N);        % errores para graficar
e2_z_dynamic = zeros(1, N);
e1_ident_z_dynamic = zeros(1,N);  %errores de identificacion
e2_ident_z_dynamic = zeros(1,N);

e1_roll = zeros(1, N);        % errores para graficar
e2_roll = zeros(1, N);
e1_ident_roll = zeros(1,N);  %errores de identificacion
e2_ident_roll = zeros(1,N);

e1_pitch = zeros(1, N);        % errores para graficar
e2_pitch = zeros(1, N);
e1_ident_pitch = zeros(1,N);  %errores de identificacion
e2_ident_pitch = zeros(1,N);

e1_yaw = zeros(1, N);        % errores para graficar
e2_yaw = zeros(1, N);
e1_ident_yaw = zeros(1,N);  %errores de identificacion
e2_ident_yaw = zeros(1,N);