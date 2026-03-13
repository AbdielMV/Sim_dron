% --- CONTROL NEURONAL ROLL ---
function [e1k, e2k, u_next] = control_rhonn_feedback_roll(phi, w_x, theta, w_y, w_z, phi_next, w_x_next, theta_next, w_y_next, w_z_next, w1, w2, dt, Iwx, Iwu, ref, ref_next, ref_two_next)
    x1k = phi;
    x1k_1 = phi_next;
    x2k   = w_x;
    x2k_1 = w_x_next;
    x3k = theta;
    x3k_1 = theta_next;
    x4k = w_y;
    x4k_1 = w_y_next;
    x5k = w_z;
    x5k_1 = w_z_next;
    x1dk    = ref;
    x1dk_1  = ref_next;
    x1dk_2  = ref_two_next;

    w14 = Iwx;
    w24 = Iwu;

    e1k  = x1k   - x1dk;
    e1k_1 = x1k_1 - x1dk_1;
    e2k  = (e1k_1 - e1k)/dt;

    k1 = 2e4; k2 = 1e3;

    % k1 = 2e4; k2 = 1e3; % Ganancias para Ref Constante

    % k1 = 2e3; k2 = (2*sqrt(k1)) + k1*0.1; % Ganancias para Ref Sinoidal

    v = -k1*e1k - k2*e2k;

    alpha = (((e2k + (dt*v))*dt) - (w1(1,1)*sgm(x1k_1)) - (w1(2,1)*sgm(x3k_1)*sgm(x1k_1)*sgm(x4k_1)) - (w1(3,1)*sgm(x3k_1)*sgm(x1k_1)*sgm(x5k_1)) + x1dk_2 + e1k_1)*(1/w14);

    u_next = (alpha - (w2(1,1)*sgm(x2k)) - (w2(2,1)*sgm(x4k)*sgm(x5k)) + (w2(3,1)*sgm(x4k)*sgm(x5k)) + w2(4,1))*(1/w24);
end