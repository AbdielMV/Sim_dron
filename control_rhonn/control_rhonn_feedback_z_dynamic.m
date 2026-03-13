% --- CONTROL NEURONAL HEIGHT ---
function [e1k, e2k, u_next] = control_rhonn_feedback_z_dynamic(z_now, v_now, phi, theta, zn_next, w1, w2, dt, Iwx, Iwu, ref, ref_next, ref_two_next, m, g)
    x1k   = z_now;
    x1k_1 = zn_next;
    x2k   = v_now;
    x3k   = phi;
    x4k   = theta;
    x1dk    = ref;
    x1dk_1  = ref_next;
    x1dk_2  = ref_two_next;

    % w11 = w1(1,1);
    % w12 = w1(2,1);
    w13 = Iwx;
    % w21 = w2(1,1);
    % w22 = w2(2,1);
    % % w23 = w2(3,1);
    w24 = Iwu;

    e1k  = x1k   - x1dk;
    e1k_1 = x1k_1 - x1dk_1;
    e2k  = (e1k_1 - e1k)/dt;

    k1 = 2e3; k2 = 2e1;

    %k1 = 3.5e3; k2 = (2*sqrt(k1))+((2*sqrt(k1))*0.1); % Ganancias para Ref Constante

    % k1 = 2e3; k2 = (2*sqrt(k1)) + k1*0.1; % Ganancias para Ref Sinoidal

    v = -k1*e1k - k2*e2k;

    alpha = (((e2k + (dt*v))*dt) - (w1(1,1)*sgm(x1k_1)) - w1(2,1) + x1dk_2 + e1k_1)*(1/w13);

    u_next = ((alpha - (w2(1,1)*sgm(x2k)) - w2(2,1) - (w2(3,1)*sgm(x3k)*sgm(x4k)))*(1/w24) + g)*(m);
        
end