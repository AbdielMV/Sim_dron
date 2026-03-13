% --- EFK TRAINING VERTICAL ---
function [w1_next,w2_next,p1_next,p2_next,e1,e2] = efk_training_x_dynamic(H,z_now,v_now,zn_now,vn_now,w1_now,w2_now,p1_now,p2_now,q1,q2,r1,r2)
    
    x1 = z_now; 
    x2 = v_now; 
    x1n = zn_now; 
    x2n = vn_now;    
    
    % Asignar H para cada capa.
    % Dron: C1 tiene 3 elementos, C2 tiene 4 elementos.
    % Ajusta los índices según el tamaño de tu vector H concatenado en rhonn_model.
    % H viene como [C1; C2].
    
    % Para Dron n=1, C1 dim=3, C2 dim=4.
    
    H1 = [H(1);H(2)];
    H2 = [H(3);H(4);H(5)];

    e_k = [x1 - x1n, x2 - x2n];
    eta = [0.5, 0.5]; % Si cambio el learning rate también cambia las ganancias necesarias

    % Capa 1
    M1 = (r1 + H1'*p1_now*H1)^-1;
    K1 = p1_now*H1*M1;
    w1_next = w1_now + eta(1)*K1*e_k(1);
    p1_next = p1_now - eta(1)*K1*H1'*p1_now + q1;

    % Capa 2
    M2 = (r2 + H2'*p2_now*H2)^-1;
    K2 = p2_now*H2*M2;
    w2_next = w2_now + eta(2)*K2*e_k(2);
    p2_next = p2_now - eta(2)*K2*H2'*p2_now + q2;
    
    e1 = e_k(1); e2 = e_k(2);
end