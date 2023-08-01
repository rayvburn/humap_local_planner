clc;
clear all;
close all;

% shared parameters
SIM_PERIOD = 0.25; % a.k.a. `dt`
ROBOT_MASS = 1.0;

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
    0.25 ...   % sim granularity
];

MIN_VEL_X = LIMITS(4);
MAX_VEL_X = LIMITS(7);
MAX_ROT_VEL = LIMITS(9);
ACCEL_LIMITS = true;
TWIST_ROTATION_COMPENSATION = 0.0;

% sim-specific
ITERATIONS = 40;

% ASSUMPTION of global velocity to twist conversion:
% constant force, in a steady state, produces velocity of direction that
% matches force direction
%%%%%%%%%%%%%%%%%%%%%%%% cases %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
computeTwistFcn = @computeTwist;

% NOTE: assertion function takes `vel_global` variable from workspace
% 1
fprintf("\n\n[CASE] 1\n");
pose_2d_init = [0.0; 0.0; deg2rad(90)];
vel_local = [MAX_VEL_X; 0.0; MAX_ROT_VEL];
force = [+100; +50; 0.0];

pose_2d = pose_2d_init;
ACCEL_LIMITS = true;
run('compute_twist_sim.m');
visualizeSim(poses, vels_global, vels_local, SIM_PERIOD)

pose_2d = pose_2d_init;
ACCEL_LIMITS = false;
run('compute_twist_sim.m');
visualizeSim(poses, vels_global, vels_local, SIM_PERIOD)

% 2
fprintf("\n\n[CASE] 2\n");
ACCEL_LIMITS = true;
pose_2d = [0.0; 0.0; deg2rad(180)];
vel_local = [0.35; 0.0; -0.50];
force = [+20, -10, 0.0];
run('compute_twist_sim.m');
visualizeSim(poses, vels_global, vels_local, SIM_PERIOD)

% 3
fprintf("\n\n[CASE] 3\n");
pose_2d = [0.0; 0.0; deg2rad(90)];
vel_local = [0.0; 0.0; 0.00];
force = [+6, +1, 0.0];
run('compute_twist_sim.m');
visualizeSim(poses, vels_global, vels_local, SIM_PERIOD)

% 4
fprintf("\n\n[CASE] 4\n");
pose_2d = [0.0; 0.0; deg2rad(-90)];
vel_local = [0.0; 0.0; 0.0];
force = [+50, +25.0, 0.0];
run('compute_twist_sim.m');
visualizeSim(poses, vels_global, vels_local, SIM_PERIOD)

% 4b: 4 with angular correction
fprintf("\n\n[CASE] 4 with angular compensation\n");
pose_2d = [0.0; 0.0; deg2rad(-90)];
vel_local = [0.0; 0.0; 0.0];
force = [+0.5, +0.25, 0.0];
TWIST_ROTATION_COMPENSATION = 0.25;
run('compute_twist_sim.m');
visualizeSim(poses, vels_global, vels_local, SIM_PERIOD)

TWIST_ROTATION_COMPENSATION = 0.0;

function [] = assertTwist(force, vel_global)
    % condition and message
    assert_diff = atan2(force(2), force(1)) - atan2(vel_global(2), vel_global(1));
    assert(abs(assert_diff) < 1e-03, ...
        "Direction of global velocity does not match direction of the force. Is robot in a steady state? Consider increasing ITERATIONS value" ...
    );
end

% from `compute_next_pose_test.m`
function [x, y, u, v] = poseToQuiver(pose, arrow_len)
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    dir = R * [arrow_len; 0];
    u = dir(1);
    v = dir(2);
end

function [X, Y, U, V] = posesToQuiver(poses, arrow_len)
    X = [];
    Y = [];
    U = [];
    V = [];
    % subsequent poses are placed in subsequent column (column vectors)
    for i=1:size(poses, 2)
        pose = poses(:, i);
        [x, y, u, v] = poseToQuiver(pose, arrow_len);
        X = [X, x];
        Y = [Y, y];
        U = [U, u];
        V = [V, v];
    end
end

function [X, Y] = posesToPlot(poses)
    X = [];
    Y = [];
    % subsequent poses are placed in subsequent column (column vectors)
    for i=1:size(poses, 2)
        pose = poses(:, i);
        X = [X, pose(1)];
        Y = [Y, pose(2)];
    end
end

function [X, Vxglob, Vyglob, Vthglob, Vxlocal, Vylocal, Vthlocal] = velsToPlot(vels_global, vels_local)
    X = [];
    Vxglob = [];
    Vyglob = [];
    Vthglob = [];
    % subsequent vels are placed in subsequent column (column vectors)
    for i=1:size(vels_global, 2)
        velg = vels_global(:, i);
        X = [X, i];
        Vxglob = [Vxglob, velg(1)];
        Vyglob = [Vyglob, velg(2)];
        Vthglob = [Vthglob, velg(3)];
    end
    
    Vxlocal = [];
    Vylocal = [];
    Vthlocal = [];
    for i=1:size(vels_local, 2)
        vell = vels_local(:, i);
        Vxlocal = [Vxlocal, vell(1)];
        Vylocal = [Vylocal, vell(2)];
        Vthlocal = [Vthlocal, vell(3)];
    end
end

function limits = findLimAxis(data)
    min_val = min(data);
    max_val = max(data);
    range = max_val - min_val;
    if (range < 1)
        min_val = min_val - 0.5;
        max_val = max_val + 0.5;
    end
    limits = [min_val, max_val];
end

function [] = visualizeSim(poses, vels_global, vels_local, dt)
    % Define the colormap for the quiver data
    % Ref: https://www.mathworks.com/matlabcentral/answers/598141-how-do-i-create-a-colour-quiver-plot-colquiver-on-top-of-a-pcolor-depth-map#comment_1020076
    cmap = turbo(size(poses, 2)); 

    % Visualize poses
    figure;
    [X, Y, U, V] = posesToQuiver(poses, dt);
    % V1: all in one
    % quiver(X, Y, U, V, 'g', 'LineWidth', 1);
    % V2: one by one to colourize
    for i = 1:size(X, 2)
        quiver(X(:, i), Y(:, i), U(:, i), V(:, i), 'Color', cmap(i, :), 'LineWidth', 2)
        hold on;
    end
    [X, Y] = posesToPlot(poses);
    hold on;
    % V1: all in one
    % plot(X, Y, 'ro');
    % V2: one by one to colourize
    for i = 1:size(X, 2)
        plot(X(:, i), Y(:, i), 'o', 'Color', cmap(i, :));
        hold on;
    end
    xlim(findLimAxis(X));
    ylim(findLimAxis(Y));
    axis('equal')
    grid on;
    
    % Visualize velocities
    figure("Position", [89, 7, 1849, 413], "Name", "Velocities");
    [X, Vxglob, Vyglob, Vthglob, Vxlocal, Vylocal, Vthlocal] = velsToPlot(vels_global, vels_local);
    subplot(1, 2, 1);
    plot(X, Vxglob, 'LineWidth', 3);
    hold on;
    plot(X, Vyglob, 'LineWidth', 2);
    plot(X, Vthglob, 'LineWidth', 1);
    legend("Vxg", "Vyg", "Vthg")
    grid on;
    subplot(1, 2, 2); grid on;
    plot(X, Vxlocal, 'LineWidth', 3);
    hold on;
    plot(X, Vylocal, 'LineWidth', 2);
    plot(X, Vthlocal, 'LineWidth', 1);
    legend("Vxl", "Vyl", "Vthl")
    grid on;
end
