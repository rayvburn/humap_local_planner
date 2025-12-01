% This script simulates, how internal force affects velocity of the agent.
% Internal force normally drives agent straight to the goal position, using
% the shortest path (see the kinematically unconstrained case).

clc;
clear all;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% shared parameters
SIM_PERIOD = 0.1; % a.k.a. `dt`
SPEED_DESIRED = 1.50;
ROBOT_MASS = 21.5;
RELAXATION_TIME = 0.465;
MIN_VEL_X = 0.0;
MAX_VEL_X = 1.5;
MAX_ROT_VEL = 2.0;
TWIST_ROTATION_COMPENSATION = 0.25;

GOAL_XY_TOLERANCE = 0.025;
ITERATIONS = 500; % default

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1st set
% pose_start = [0.0, 1.0, deg2rad(90)];
% vel_local_start = [1.0, 0.0, deg2rad(45)];
% goal_pose = [-0.5, 1.0, 0.0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2nd set
% pose_start = [0.0, 1.0, deg2rad(90)];
% vel_local_start = [1.0, 0.0, deg2rad(45)];
% goal_pose = [-0.1, -0.5, 0.0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3rd set
% pose_start = [0.0, 1.0, deg2rad(90)];
% vel_local_start = [1.0, 0.0, deg2rad(45)];
% goal_pose = [+0.1, -2.5, 0.0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4th set
% ITERATIONS = 500;
% pose_start = [0.0, 0.0, deg2rad(90)];
% vel_local_start = [1.0, 0.0, deg2rad(45)];
% goal_pose = [+2.0, -0.1, 0.0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 5th set (close goal)
ITERATIONS = 200;
pose_start = [0.0, 0.0, deg2rad(90)];
vel_local_start = [1.0, 0.0, deg2rad(45)];
goal_pose = [+2.5, +0.15, 0.0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

executed_iterations = [0 0 0];
force_vel_dir_diff = {};
twist_rot_compensation = 0.0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% investigate multiple cases
for case_id = 1:length(executed_iterations)

    % used in internal loop
    pose_2d = pose_start;
    vel_local = vel_local_start;

    % init drawing
    vel_global = [0 0 0];
    drawVelocity(pose_2d, vel_global, [], 1)
    % draw goal
    drawVelocity(goal_pose, [0 0 0], [], 0)
    
    % select function to use in the internal force computations
    if case_id == 1
        computeInternalForceFcn = @computeInternalForce;
        computeTwistFcn = @computeTwist;
        % twist_rot_compensation does not matter when no kinematics constraints present
    elseif case_id == 2
        computeInternalForceFcn = @computeInternalForce;
        computeTwistFcn = @computeTwist;
        twist_rot_compensation = 0.0;
    elseif case_id == 3
        computeInternalForceFcn = @computeInternalForce;
        computeTwistFcn = @computeTwist;
        twist_rot_compensation = TWIST_ROTATION_COMPENSATION;
    end

    % simulation
    for i = 1:ITERATIONS
        % logging header
        if case_id == 1
            fprintf("%d UNCONSTRAINED\n", i);
        else
            fprintf("%d CONSTRAINED %d\n", i, case_id);
        end

        % compute internal force
        force = computeInternalForceFcn(pose_2d, vel_global, goal_pose, SPEED_DESIRED, ROBOT_MASS, RELAXATION_TIME);
        
        % compute global velocity
        if case_id == 1 % kinematic constraints ignored
           % ignore kinematic constraints, convert straight to the global velocity
            vel_global = [force(1) force(2) force(3)] / ROBOT_MASS * SIM_PERIOD; 
        else
            % compute twist based on pose, current velocity (global) and force
            vel_local = computeTwistFcn(...
                pose_2d,...
                force,...
                vel_global,...
                SIM_PERIOD,...
                ROBOT_MASS,...
                MIN_VEL_X,...
                MAX_VEL_X,...
                MAX_ROT_VEL,...
                twist_rot_compensation...
            );
            % converting twist to the global vector
            vel_global = computeVelocityGlobal(vel_local, pose_2d);
        end
        
        % eval correctness of velocity direction, compared to force direction
        force_dir = atan2(force(2), force(1));
        vel_dir = atan2(vel_global(2), vel_global(1));
            
        fprintf("[         loop        ] DIR: force %2.3f rad (%2.2f°), global vel %2.3f rad (%2.2f°)\n",...
            force_dir, rad2deg(force_dir), ...
            vel_dir, rad2deg(vel_dir) ...
        );
        force_vel_dir_diff{case_id}{i} = [force_dir, vel_dir, force_dir - vel_dir];

        % perform the movement and draw the current state
        pose_2d = predictPose(pose_2d, vel_global, SIM_PERIOD);
        drawVelocity(pose_2d, vel_global, [], 0);
        drawVelocity(pose_2d, 0.15 * force / norm(force), [], 0);

        % abort further computations if near the goal
        dist_to_goal = norm(pose_2d(1:2) - goal_pose(1:2));
        if dist_to_goal <= GOAL_XY_TOLERANCE
            fprintf("\naborting further simulation due to goal nearby\n");
            break;
        end
        if norm(force) <= 1e-03
            fprintf("\naborting further simulation due to minimal force\n");
            break; 
        end   

    end % i
    executed_iterations(case_id) = i;

end % case_id

% Summary
fprintf("\nIterations executed:\n");

for i = 1 : length(executed_iterations)
    fprintf("\tCASE ID %d\t%4d / %4d\n", ...
        i, executed_iterations(i), ITERATIONS ...
    );   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Put all existing figures into one window
% Ref: https://www.mathworks.com/matlabcentral/answers/101273-how-can-i-put-existing-figures-in-different-subplots-in-another-figure-in-matlab-6-5-r13
figlist = get(groot,'Children');
newfig = figure('Position', [0, 1000, 500, 500]);
tcl = tiledlayout(newfig, 'flow');

% prep new fig limits
OVERSIZE = 0.5;
x_min = min(pose_start(1), goal_pose(1)) - OVERSIZE;
x_max = max(pose_start(1), goal_pose(1)) + OVERSIZE;
y_min = min(pose_start(2), goal_pose(2)) - OVERSIZE;
y_max = max(pose_start(2), goal_pose(2)) + OVERSIZE;

figs_num = numel(figlist);
for i = 1:figs_num
    figure(figlist(i));
    ax = gca;
    ax.Parent = tcl;
    ax.Layout.Tile = i;
    set(ax, 'xlim' , [x_min x_max], 'ylim' , [y_min y_max]);
    title(ax, sprintf("case %d", figs_num - i + 1));
    close(figlist(i));
end

% plot force vs vel direction differences %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
case_ids_num = size(force_vel_dir_diff, 2);
cols = 2;
rows = ceil(case_ids_num / 2);
for i = 1:case_ids_num % i - iterate over case_id-s
    subplot(rows, cols, i);
    X = 1:size(force_vel_dir_diff{i}, 2);
	% Y1 = zeros(1, length(X)); % force_dir
	% Y2 = zeros(1, length(X)); % vel_dir
    Y3 = zeros(1, length(X)); % diff
    for j = 1:length(X) % j - iterate over specific simulation data
        % Y1(j) = force_vel_dir_diff{i}{j}(1);
        % Y2(j) = force_vel_dir_diff{i}{j}(2);
        Y3(j) = force_vel_dir_diff{i}{j}(3);
    end
    % plot(X, Y1);
    % hold on;
    % plot(X, Y2);
    % hold on;
    plot(X, Y3);
	% legend('force_dir', 'vel_dir', 'diff'); % full version: Y1-Y3
    legend('diff'); % Y3 only
    title(sprintf("case %d", i));
    ylim([-pi +pi]);
end
