function vel_local = computeVelocityLocal(vel_global, pose_2d)
    % last equation in
    % https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-2d/
    yaw = pose_2d(3);
    R_yaw = [ cos(yaw)  sin(yaw) 0;
             -sin(yaw)  cos(yaw) 0;
              0         0        1];
    vel_local = R_yaw * [vel_global(1); vel_global(2); vel_global(3)];
end
