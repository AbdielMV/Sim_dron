% --- CONTROL NEURONAL YAW ---
function [e1k, e2k, u_next] = control_rhonn_feedback_yaw(psi, w_z, phi, theta, w_x, w_y, psi_next, w_z_next, phi_next, theta_next, w_x_next, w_y_next, w1, w2, dt, Iwx, Iwu, ref, ref_next, ref_two_next)
    x1k = psi;
    x1k_1 = psi_next;
    x2k   = w_z;
    x2k_1 = w_z_next;
    x3k = phi;
    x3k_1 = phi_next;
    x4k = theta;
    x4k_1 = theta_next;
    x5k = w_x;
    x5k_1 = w_x_next;
    x6k = w_y;
    x6k_1 = w_y_next;
    x1dk    = ref;
    x1dk_1  = ref_next;
    x1dk_2  = ref_two_next;

    w14 = Iwx;
    w24 = Iwu;

    e1k  = x1k   - x1dk;
    e1k_1 = x1k_1 - x1dk_1;
    e2k  = (x1k_1 - x1dk_1 - e1k)/dt;

    k1 = 3e3; k2 = 3e1; % Ganancias para Ref Constante

    % k1 = 2e3; k2 = (2*sqrt(k1)) + k1*0.1; % Ganancias para Ref Sinoidal

    v = -k1*e1k - k2*e2k;

    alpha = (((e2k + (dt*v))*dt) - (w1(1,1)*sgm(x1k_1)) - (w1(2,1)*sgm(x3k_1)*sgm(x4k_1)*sgm(x6k_1)) - (w1(3,1)*sgm(x3k_1)*sgm(x4k_1)*sgm(x2k_1)) + x1dk_2 + e1k_1)*(1/w14);

    u_next = (alpha - (w2(1,1)*sgm(x2k)) - (w2(2,1)*sgm(x5k)*sgm(x6k)) + (w2(3,1)*sgm(x5k)*sgm(x6k)))*(1/w24);
end