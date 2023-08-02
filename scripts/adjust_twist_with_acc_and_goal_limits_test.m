clc;
clear all;
close all;

% shared parameters
SIM_PERIOD = 0.01; % a.k.a. `dt`
SIM_TIME = 4.0;
LIMITS = [...
    1.0, ...   % acc_lim_x
    0.0, ...   % acc_lim_y
    1.05, ...  % acc_lim_th
    -0.1, ...  % vel_min_x
    0.0, ...   % vel_min_y
    -1.05, ... % vel_min_th
    0.50, ...  % vel_max_x
    0.00, ...  % vel_max_y
    +1.05, ... % vel_max_th
    SIM_PERIOD ...   % sim granularity
];

MAINTAIN_VEL_COMPONENTS_RATE = true;
GOAL_POSE = [2.0; 0.0; 0.0];
ROBOT_INIT_POSE = [0.0; 0.0; 0.0];
VEL_LOCAL_INIT = [0.0; 0.0; 0.0];
VEL_CMD = [0.50; 0.0; 0.0];

vel_local = VEL_LOCAL_INIT;
robot_pose = ROBOT_INIT_POSE;

figure;
for i=1:1e06
    dist_to_goal = norm(GOAL_POSE(1:2) - robot_pose(1:2));
    if dist_to_goal <= 0.05
        break;
    end
    
    [~, vel_out] = adjustTwistWithAccAndGoalLimits(vel_local, LIMITS, VEL_CMD, MAINTAIN_VEL_COMPONENTS_RATE, dist_to_goal, SIM_TIME);
    vel_local = vel_out;
    
    plot(dist_to_goal, vel_local(1), 'ro')
    %plot(i, vel_local(1), 'ro')
    hold on;
    plot(dist_to_goal, vel_local(2), 'go')
    %plot(i, vel_local(2), 'go')
    
    robot_pose = computeNextPoseBaseVel(robot_pose, vel_local, SIM_PERIOD);
end
