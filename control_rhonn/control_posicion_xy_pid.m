function [ref_phi, ref_theta] = control_posicion_xy_pid(x_act, y_act, vx_act, vy_act, x_des, y_des, vx_des, vy_des, psi)
    % IMPORTANTE: Fíjate que vx_des y vy_des están en los argumentos de arriba ^
    
    % Ganancias del PID de Posición
    kp = 1.3;%1.5; 
    kd = 1.2;%1.2; 
    
    % 1. Errores de Posición
    ex = x_des - x_act;
    ey = y_des - y_act;
    
    % 2. Errores de Velocidad (Feedforward)
    % Aquí usamos la vx_des que ahora sí existe
    evx = vx_des - vx_act; 
    evy = vy_des - vy_act;
    
    % 3. PID -> Aceleración Virtual Deseada (Inercial)
    % u = kp*error_pos + kd*error_vel
    ux_des = kp * ex + kd * evx;
    uy_des = kp * ey + kd * evy;

    % 4. Saturación de Aceleración (Seguridad)
    % Limitamos a ~2.5 m/s^2 (aprox 15 grados)
    sat = 2.5;
    ux_des = max(-sat, min(sat, ux_des));
    uy_des = max(-sat, min(sat, uy_des));

    % 5. Rotación de Yaw (Mundo -> Cuerpo)
    % Compensamos la rotación del dron
    acc_fwd = ux_des * cos(psi) + uy_des * sin(psi);
    acc_lat = ux_des * sin(psi) - uy_des * cos(psi);

    % 6. Mapeo Aceleración -> Ángulos
    g = 9.81;
    % Pitch positivo mueve hacia adelante (+X cuerpo) en tu modelo
    % Roll positivo mueve hacia derecha (+Y cuerpo) [Verifica tu signo de Roll]
    
    ref_theta =  acc_fwd / g;  % <--- AQUÍ YA ESTÁ CORREGIDO EL SIGNO POSITIVO
    ref_phi   =  acc_lat / g; 

    % Saturación de ángulos (Seguridad final)
    max_ang = deg2rad(40);
    ref_theta = max(-max_ang, min(max_ang, ref_theta));
    ref_phi   = max(-max_ang, min(max_ang, ref_phi));
end