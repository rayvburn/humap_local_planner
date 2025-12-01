function [pose_new] = computeNextPose(pose, vel, dt)
%COMPUTENEXTPOSE Given current pose, velocity in a global coordinate system and time delta, computes a new 2D pose
% in a global coordinate system

    % See https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/?answer=231954#post-id-231954
    pose_new = [];
    new_x = pose(1) + vel(1) * dt;
    new_y = pose(2) + vel(2) * dt;
    new_yaw = pose(3) + vel(3) * dt;

    pose_new = [new_x; new_y; new_yaw];
end
