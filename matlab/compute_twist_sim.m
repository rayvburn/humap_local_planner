% this script aims to evaluate force -> twist command conversion
% requires (with examples):
% SIM_PERIOD = 1.0; % a.k.a. `dt`
% ROBOT_MASS = 1.0;
% MIN_VEL_X = 0.0;
% MAX_VEL_X = 1.5;
% MAX_ROT_VEL = 2.0;
% 
% pose_2d = [0.0, 0.0, deg2rad(90)];
% vel_local = [0.0, 0.0, 0.05];
% force = [+0.2, -0.1, 0.0];
%
% iterations

vel_global = computeVelocityGlobal(vel_local, pose_2d);
drawVelocity(pose_2d, vel_global, [], 1)

for i=1:ITERATIONS
    fprintf("%d\n", i);
    % convert current local velocity to global vector
    vel_global = computeVelocityGlobal(vel_local, pose_2d);
    % compute twist based on pose, current velocity (global) and force
    vel_local = computeTwistFcn(...
        pose_2d,...
        force,...
        vel_local,...
        vel_global,...
        SIM_PERIOD,...
        ROBOT_MASS,...
        MIN_VEL_X,...
        MAX_VEL_X,...
        MAX_ROT_VEL...
    );
    % debugging -> converting twist to the global vector
    vel_global = computeVelocityGlobal(vel_local, pose_2d);
    pose_2d = predictPose(pose_2d, vel_global, SIM_PERIOD);
    drawVelocity(pose_2d, vel_global, [], 0)
end

xlim([-10 10]);
ylim([-10 10]);
fprintf("Simulation finished\r\n");
