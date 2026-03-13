% --- NN STRUCTURE YAW ---
function [XN1_next, XN2_next, H, Iwu, Iwx] = rhonn_model_yaw_dynamic(psi, omega_z, phi, theta, omega_x, omega_y, u_now, w1_now, w2_now, dt)
    x1k = psi;
    x2k = omega_z;
    x3k = phi;
    x4k = theta;
    x5k = omega_x;
    x6k = omega_y;
    w1 = w1_now;
    w2 = w2_now;

    %Define sets
    % C1 = [sgm(x1k);1]; %Dim = 2
    % C2 = [sgm(x2k);1;-g]; %Dim = 3

    C1 = [sgm(x1k);sgm(x3k)*sgm(x4k)*sgm(x6k);sgm(x3k)*sgm(x4k)*sgm(x2k)]; %Dim = 3
    C2 = [sgm(x2k);sgm(x5k)*sgm(x6k);-sgm(x5k)*sgm(x6k)]; %Dim = 3

    %Input Weight
    Iw14= 0.001; %0.008; %0.08
    Iw24= 1; %0.09; %0.9

    %Update Neural States
    XN1_next = (w1'*C1) + (Iw14*x2k);
    XN2_next = (w2'*C2) + Iw24*(u_now*dt);
    % XN2_next = (w2'*C2) + Iw24*((u_now*dt)/m);

    H = [C1;C2];
    Iwx = Iw14;
    Iwu = Iw24;
end