clear all;
close all;
clc;

limits = [...
    1.0, ...   % acc_lim_x
    0.0, ...   % acc_lim_y
    1.05, ...  % acc_lim_th
    -0.1, ...  % vel_min_x
    0.0, ...   % vel_min_y
    -1.05, ... % vel_min_th
    0.50, ...  % vel_max_x
    0.00, ...  % vel_max_y
    +1.05, ... % vel_max_th
    0.25 ...   % sim granularity
];

vel_init = [0.27, 0.0, 0.102];
vel_loop{1} = vel_init;
for i=1:10
    vel_out = findTwistWithAccLimits(vel_loop{i}, limits, false)
    vel_loop{i+1} = vel_out;
end
