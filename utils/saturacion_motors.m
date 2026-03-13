function [U, omega_motors] = saturacion_motors(u_torques, cT, d, cQ)

    M = [ cT,     cT,     cT,     cT;
         -d*cT, -d*cT,  d*cT,  d*cT;
         -d*cT,  d*cT,  d*cT, -d*cT;
         -cQ,     cQ,   -cQ,    cQ];

    omega_square = inv(M)*u_torques;

    omega_square = max(omega_square, 0);

    omega_motors = sqrt(omega_square);

    U = M*(omega_motors.^2);

end