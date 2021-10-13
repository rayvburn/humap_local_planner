syms theta
R_yaw_inv = [cos(theta) 0 0;
             sin(theta) 0 0;
             0          0 1];
R_yaw = pinv(R_yaw_inv)
