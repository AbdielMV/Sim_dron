function [ref_phi, ref_theta] = compensator(ux_des, uy_des, psi)
    % 1. Convertir las fuerzas pedidas por el controlador RHONN a aceleraciones
    m = 0.468;
    g = 9.81;
    ax_des = ux_des / m;
    ay_des = uy_des / m;

    % 2. Compensación de Yaw (Rotar el vector de aceleración al frente del dron)
    acc_fwd = ax_des * cos(psi) + ay_des * sin(psi);
    acc_lat = ax_des * sin(psi) - ay_des * cos(psi);

    % 3. Mapeo Aceleración -> Ángulos (Con los signos correctos de tu física)
    ref_theta =  acc_fwd / g; % Pitch (+) mueve hacia adelante
    ref_phi   =  acc_lat / g; % Roll (+) mueve hacia la derecha (verifica la física)

    % 4. Saturación por seguridad (Máximo ~20 grados)
    max_ang = deg2rad(30);
    ref_theta = max(-max_ang, min(max_ang, ref_theta));
    ref_phi   = max(-max_ang, min(max_ang, ref_phi));
end