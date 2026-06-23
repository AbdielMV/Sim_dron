function [U, omega_motors] = saturacion_motors(u_torques, cT, d, cQ)

    % Matriz de mezcla para configuración en "X"
    M = [ cT,                 cT,                 cT,                 cT;
         -(sqrt(2)/2)*d*cT, -(sqrt(2)/2)*d*cT,  (sqrt(2)/2)*d*cT,  (sqrt(2)/2)*d*cT;
         -(sqrt(2)/2)*d*cT,  (sqrt(2)/2)*d*cT,  (sqrt(2)/2)*d*cT, -(sqrt(2)/2)*d*cT;
         -cQ,                 cQ,                -cQ,                 cQ];

    % Evitamos inv(M) usando división izquierda para mayor estabilidad numérica
    omega_square = M \ u_torques;

    % --- LÍMITES FÍSICOS DE LOS MOTORES ---
    rpm_max = 14000;                     % RPM máximas permitidas
    rad_s_max = rpm_max * (2*pi/60);     % Conversión a rad/s
    max_omega_sq = rad_s_max^2;          % Límite superior al cuadrado

    % Saturación inferior (0) y superior (14,000 RPM)
    omega_square = min(max(omega_square, 0), max_omega_sq);

    % Extracción de velocidades reales de los motores
    omega_motors = sqrt(omega_square);
    
    % Recálculo de las fuerzas y torques (U) que REALMENTE se generan
    U = M * (omega_motors.^2);

end