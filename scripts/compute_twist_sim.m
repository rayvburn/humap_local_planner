% This script is internally ran by the `compute_twist_cases.m` script!
%
% this script aims to evaluate force -> twist command conversion
% requires (with examples):
% SIM_PERIOD = 1.0; % a.k.a. `dt`
% ROBOT_MASS = 1.0;
% MIN_VEL_X = 0.0;
% MAX_VEL_X = 1.5;
% MAX_ROT_VEL = 2.0;
% LIMITS % see kinodynamic limits in adjust_twist_with_acc_limits_test.m
% TWIST_ROTATION_COMPENSATION = 0.25;
% KINEMATIC_LIMITS = true;
% 
% pose_2d = [0.0, 0.0, deg2rad(90)];
% vel_local = [0.0, 0.0, 0.05];
% force = [+0.2, -0.1, 0.0];
%
% iterations

% list of 2D poses throughout the simulation
poses = [];
vels_local = [];
vels_global = [];

for i=1:ITERATIONS
    fprintf("%d\n", i);
    % convert current local velocity to global vector
    vel_global = computeVelocityGlobal(vel_local, pose_2d);

    % store poses and velocities
    poses       = [poses,       pose_2d];
    vels_local  = [vels_local,  vel_local];
    vels_global = [vels_global, vel_global];

    % compute twist based on pose, current velocity (global) and force    
    vel_cmd = computeTwistFcn(...
        pose_2d,...
        force,...
        vel_global,...
        SIM_PERIOD,...
        ROBOT_MASS,...
        MIN_VEL_X,...
        MAX_VEL_X,...
        MAX_ROT_VEL,...
        TWIST_ROTATION_COMPENSATION...
    );

    if ACCEL_LIMITS
    % conform to kinematic limits
        [~, vel_out] = adjustTwistWithAccLimits(vel_local, LIMITS, vel_cmd, true);
    else
        vel_out = vel_cmd';
    end
    
    vel_local = vel_out';
    % compute next pose
    pose_2d = computeNextPoseBaseVel(pose_2d, vel_out, SIM_PERIOD);
end

fprintf("Simulation finished\r\n");
