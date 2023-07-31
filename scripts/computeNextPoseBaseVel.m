function [pose_new] = computeNextPoseBaseVel(pose, vel, dt)
%COMPUTENEXTPOSEBASEVEL Given current pose, velocity in a local (base) coordinate system and time delta, computes
% a new 2D pose in a global coordinate system

    theta = pose(3);
    
    pose_new = [];
    
    % NOTE: calculations based on base_local_planner::SimpleTrajectoryGenerator::computeNewPositions
    new_x = pose(1) + (vel(1) * cos(theta) + vel(2) * cos((pi/2) + theta)) * dt;
    new_y = pose(2) + (vel(1) * sin(theta) + vel(2) * sin((pi/2) + theta)) * dt;
    new_yaw = pose(3) + vel(3) * dt;

    pose_new = [new_x; new_y; new_yaw];
end
