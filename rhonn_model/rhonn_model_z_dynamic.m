% --- NN STRUCTURE VERTICAL DYNAMIC ---
function [XN1_next, XN2_next, H, Iwu, Iwx] = rhonn_model_z_dynamic(z_now, v_now, phi, theta, u_now, w1_now, w2_now, dt, g, m)
    x1k = z_now;
    x2k = v_now;
    x3k = phi;
    x4k = theta;
    w1 = w1_now;
    w2 = w2_now;

    %Define sets
    % C1 = [sgm(x1k);1]; %Dim = 2
    % C2 = [sgm(x2k);1;-g]; %Dim = 3

    C1 = [sgm(x1k);1]; %Dim = 2
    C2 = [sgm(x2k);1;sgm(x4k)*sgm(x3k)]; %Dim = 2

    %Input Weight
    Iw13= 0.001; %0.008; %0.08
    Iw24= 0.1; %0.09; %0.9

    %Update Neural States
    XN1_next = (w1'*C1) + (Iw13*x2k);
    XN2_next = (w2'*C2) + Iw24*((-g*dt + u_now*dt)/m);
    % XN2_next = (w2'*C2) + Iw24*((u_now*dt)/m);

    H = [C1;C2];
    Iwx = Iw13;
    Iwu = Iw24;
end