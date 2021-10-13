function vel_global = computeVelocityGlobal(vel_local, pose_2d)
    yaw = pose_2d(3);
    R_yaw_inv = [cos(yaw) 0 0;
                 sin(yaw) 0 0;
                 0        0 1];
    vel_global = R_yaw_inv * vel_local';

    fprintf("[computeVelocityGlobal] velocity local : x: %2.4f, y: %2.4f, theta: %2.4f   (yaw: %2.4f,  sin(yaw): %2.4f,  cos(yaw): %2.4f)\n", ...
		vel_local(1), ...
		vel_local(2), ...
		vel_local(3), ...
		yaw, ...
		sin(yaw), ...
		cos(yaw) ...
	);
    fprintf("[computeVelocityGlobal] velocity global: x: %2.4f, y: %2.4f, z: %2.4f\n", ...
		vel_global(1), ...
		vel_global(2), ...
		vel_global(3) ...
	);
end