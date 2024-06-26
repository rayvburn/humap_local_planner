function pose_next = predictPose(pose, vel, dt)
    vel_row_v = [vel(1) vel(2) vel(3)];
    pose_next = pose + vel_row_v * dt;
    pose_next(3) = wrapToPi(pose_next(3));
    fprintf("[    predictPose      ] pose_curr: x %2.4f, y %2.4f, yaw %2.4f | vel: x %2.4f, y %2.4f, theta %2.4f\n",...
        pose(1), pose(2), pose(3), ...
        vel(1), vel(2), vel(3)...
    );
    fprintf("[    predictPose      ] pose_delta: x %2.4f, y %2.4f, yaw %2.4f\n",...
        pose_next(1) - pose(1), ...
        pose_next(2) - pose(2), ...
        pose_next(3) - pose(3) ...
    );
    fprintf("[    predictPose      ] pose_next: x %2.4f, y %2.4f, yaw %2.4f\n",...
        pose_next(1), pose_next(2), pose_next(3) ...
    );
end