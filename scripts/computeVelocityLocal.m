function vel_local = computeVelocityLocal(vel_global, pose_2d, holonomic)
    if ~exist('holonomic', 'var')
        holonomic = false;
    end
    
    yaw = pose_2d(3);
    if holonomic
        % last equation in
        % https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-2d/
        R_yaw = [ cos(yaw)  sin(yaw) 0;
                 -sin(yaw)  cos(yaw) 0;
                  0         0        1];
        vel_local = R_yaw * [vel_global(1); vel_global(2); vel_global(3)];
    else
        % simplified version, reflects the cpp implementation
        R_yaw = [ cos(yaw)  sin(yaw) 0;
                  0         0        0;
                  0         0        1];
        vel_local = R_yaw * [vel_global(1); vel_global(2); vel_global(3)];
    end
end
