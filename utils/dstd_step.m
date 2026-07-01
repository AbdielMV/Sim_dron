function [z1_next, z2_next, z1_k, z1_k1, z1_k2] = dstd_step(f_in, z1_now, z2_now, k1, k2, dt, epsilon)
    % f_in: La señal de referencia que entra (ej. ref_phi_calc_rhonn)
    
    % 1. Cálculo del error entre el estado del observador y la señal entrante
    e_k = z1_now - f_in;
    
    % 2. Función signo aproximada (evita chattering en simulación numérica)
    sign_approx = e_k / (abs(e_k) + epsilon);
    
    % 3. Dinámica discreta del Super Twisting (Euler explícito)
    z1_next = z1_now + dt * (-k1 * sqrt(abs(e_k)) * sign_approx + z2_now);
    z2_next = z2_now - dt * (k2 * sign_approx);
    
    % 4. Proyección del futuro mediante Serie de Taylor de primer orden
    z1_k  = z1_now;                   % Instante actual k
    z1_k1 = z1_now + (dt * z2_now);       % Instante futuro k+1
    z1_k2 = z1_now + (2 * dt * z2_now);   % Instante futuro k+2
end

