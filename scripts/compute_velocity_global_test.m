% this script aims to evaluate twist command -> global velocity conversion
clear all;
close all;

vel_local = [1.0, 0.0, 1.0];
pose_2d = [0.0, 0.0, deg2rad(20)];
vel_global = computeVelocityGlobal(vel_local, pose_2d);

dt = 0.1;
drawVelocity(pose_2d, vel_global, [], 1)

for i=1:50
    fprintf("%d\n", i);
    vel_global = computeVelocityGlobal(vel_local, pose_2d);
    pose_2d = predictPose(pose_2d, vel_global, dt);
    drawVelocity(pose_2d, vel_global, [], 0)
end

axis equal
