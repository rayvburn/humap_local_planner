syms theta
R_yaw_inv = [cos(theta) 0 0;
             sin(theta) 0 0;
             0          0 1];
R_yaw = pinv(R_yaw_inv)

theta = deg2rad(30)
R_cl = [cos(theta) -sin(theta); sin(theta) cos(theta)]
R_cl_inv = pinv(R_cl)