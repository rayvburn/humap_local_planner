function vel_global = computeVelocityGlobal(vel_local, pose_2d)
    yaw = pose_2d(3);
    % simplified matrix compatible with differential drives only
    % R_yaw_inv = [cos(yaw) 0 0;
    %              sin(yaw) 0 0;
    %              0        0 1];
    %
    % proper matrix for both holonomic and nonholonomic drives
    % ref: second to last equation at
    % https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-2d/
    R_yaw_inv = [cos(yaw) -sin(yaw) 0;
                 sin(yaw)  cos(yaw) 0;
                 0         0        1];
    vel_global = R_yaw_inv * [vel_local(1); vel_local(2); vel_local(3)];
end
