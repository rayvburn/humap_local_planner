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

% initial vel. conditions
vel_init = [0.27, 0.0, 0.102];
% container storing feasible velocities
vel_loop = vel_init;
% container storing velocities arising from accel. limits
vel_acc_lim = [vel_init, vel_init];

cmd_vel_loop = [
    [0.00, 0.0, 0.40];
    [0.35, 0.0, 0.40];
    [0.35, 0.0, 0.40];
    [0.35, 0.0, 0.40];
    [0.00, 0.0, 0.00]; % stop
    [0.00, 0.0, 0.00];
    [0.00, 0.0, 0.00];
    [0.50, 0.0, 0.00]; % start
    [0.50, 0.0, 0.50]; % turn
    [0.50, 0.0, 0.50];
    [0.50, 0.0, 0.50];
    [0.60, 0.0, 0.00];
    [0.60, 0.0, 0.00];
    [0.60, 0.0, 0.00];
    [0.00, 0.0, 0.00]; % stop
    [0.00, 0.0, 0.00];
    [-0.1, 0.0, 1.00]; % backward
    [-0.1, 0.0, 1.00];
    [-0.1, 0.0, 1.00];
    [-0.1, 0.0, 1.00];
    [0.00, 0.0, 0.00]; % stop
    [0.00, 0.0, 0.00];
    [0.00, 0.0, 0.00];
    [0.00, 0.0, 0.00];
    [0.00, 0.0, 0.00]
];

for i=1:length(cmd_vel_loop)
    vel_curr = vel_loop(i,1:3);
    vel_cmd = cmd_vel_loop(i,1:3);
    
    vel_acc_lim_min = findTwistWithAccLimits(vel_curr, limits, true);
    vel_acc_lim_max = findTwistWithAccLimits(vel_curr, limits, false);
    
    [~, vel_out] = adjustTwistWithAccLimits(vel_curr, limits, vel_cmd, false);
    
    % collect into containers
    vel_loop = [vel_loop; vel_out];
    % velocity limits resulting from acceleration limits
    vel_acc_lim = [vel_acc_lim; [vel_acc_lim_min, vel_acc_lim_max]];
end

% mark vel/acc violations
lim_violations_lin = [];
lim_violations_ang = [];
for i=1:length(vel_loop)
    lim_acc_vel_min = vel_acc_lim(i, 1:3);
    lim_acc_vel_max = vel_acc_lim(i, 4:6);
    vel = vel_loop(i, 1:3);
    if vel(1) > lim_acc_vel_max(1) || vel(1) < lim_acc_vel_min(1)
        lim_violations_lin = [lim_violations_lin; [i, vel(1)]];
    end
    if vel(2) > lim_acc_vel_max(2) || vel(2) < lim_acc_vel_min(2)
        lim_violations_lin = [lim_violations_lin; [i, vel(2)]];
    end
    if vel(3) > lim_acc_vel_max(3) || vel(3) < lim_acc_vel_min(3)
        lim_violations_ang = [lim_violations_ang; [i, vel(3)]];
    end
end


% Visualization
COLOR_CMD_VEL = [0.65, 0.65, 0.65];
CMD_LINEWIDTH = 1.5;
LIM_LINEWIDTH = CMD_LINEWIDTH / 2;
VEL_LIM_LINESTYLE = '-.';
VEL_ACC_LIM_LINESTYLE = '--';
VEL_LINESTYLE = '--';
VEL_LINEWIDTH = 3.0;
x = 1:length(vel_loop);
limits_lin = [limits(4), limits(5), limits(7), limits(8)];
limits_ang = [limits(6), limits(9)];
ylimits_lin = [1.1 * min(limits_lin), 1.1 * max(limits_lin)];
ylimits_ang = [1.1 * min(limits_ang), 1.1 * max(limits_ang)];


figure('Position', [20 20 1800 1000])

fig_vels_lin = subplot(2, 1, 1);
hold on;
grid on;
%%% plot limits
% limits of vel_min_x, vel_min_y, vel_min_th
plot(x, ones(length(x), 1) * [limits(4), limits(5)], 'LineStyle', VEL_LIM_LINESTYLE)
% limits of vel_max_x, vel_max_y, vel_max_th
plot(x, ones(length(x), 1) * [limits(7), limits(8)], 'LineStyle', VEL_LIM_LINESTYLE)
% commanded (requested) velocities
stairs(x(2:end), cmd_vel_loop(:,1:2), 'LineWidth', CMD_LINEWIDTH, 'Color', COLOR_CMD_VEL);
% velocity limits resulting from current velocity and acceleration limits
%%% minimum velocities
stairs(x, vel_acc_lim(:,1:2), 'LineWidth', LIM_LINEWIDTH, 'LineStyle', VEL_ACC_LIM_LINESTYLE);
%%% maximum velocities
stairs(x, vel_acc_lim(:,4:5), 'LineWidth', LIM_LINEWIDTH, 'LineStyle', VEL_ACC_LIM_LINESTYLE);
% linear velocities
stairs(x, vel_loop(:,1:2), 'LineWidth', VEL_LINEWIDTH, 'LineStyle', VEL_LINESTYLE);
% velocity violations
if ~isempty(lim_violations_lin)
    plot(lim_violations_lin(:,1), lim_violations_lin(:,2), 'rx', 'MarkerSize', 15, 'LineWidth', 3)
end
legend(...
    'vxmin', 'vymin', ...
    'vxmax', 'vymax', ...
    'vxcmd', 'vycmd', ...
    'vxamin', 'vyamin', ...
    'vxamax', 'vyamax',  ...
    'vx', 'vy' ...
)
ylim(ylimits_lin)
title('Linear velocities')


fig_vel_ang = subplot(2, 1, 2);
hold on;
grid on;
% limits of vel_min_x, vel_min_y, vel_min_th
plot(x, ones(length(x), 1) * [limits(6)], 'LineStyle', VEL_LIM_LINESTYLE)
% limits of vel_max_x, vel_max_y, vel_max_th
plot(x, ones(length(x), 1) * [limits(9)], 'LineStyle', VEL_LIM_LINESTYLE)
% commanded (requested) velocities
stairs(x(2:end), cmd_vel_loop(:,3), 'LineWidth', CMD_LINEWIDTH, 'Color', COLOR_CMD_VEL);
% velocity limits resulting from current velocity and acceleration limits
%%% minimum velocities
stairs(x, vel_acc_lim(:,3), 'LineWidth', LIM_LINEWIDTH, 'LineStyle', VEL_ACC_LIM_LINESTYLE);
%%% maximum velocities
stairs(x, vel_acc_lim(:,6), 'LineWidth', LIM_LINEWIDTH, 'LineStyle', VEL_ACC_LIM_LINESTYLE);
% angular velocity
stairs(x, vel_loop(:,3), 'LineWidth', VEL_LINEWIDTH, 'LineStyle', VEL_LINESTYLE)
% velocity violations
if ~isempty(lim_violations_ang)
    plot(lim_violations_ang(:,1), lim_violations_ang(:,2), 'rx', 'MarkerSize', 15, 'LineWidth', 3)
end
legend(...
    'vthmin', ...
    'vthmax', ...
    'vthcmd', ...
    'vthamin', ...
    'vthamax', ...
    'vth' ...
)
ylim(ylimits_ang)
title('Angular velocity')
