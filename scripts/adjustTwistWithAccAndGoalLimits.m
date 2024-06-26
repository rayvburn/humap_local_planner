function [status, cmd_vel] = adjustTwistWithAccAndGoalLimits( ...
	vel_curr, ...
	limits,   ...
    cmd_vel,  ...
    maintain_vel_components_rate, ...
    dist_to_goal, ...
    sim_time ...
)
    %
    % Adjusts velocity command with acceleration and goal position limits
    % 
    
    if ~exist('maintain_vel_components_rate', 'var')
        maintain_vel_components_rate = true;
    end
    if ~exist('dist_to_goal', 'var')
        dist_to_goal = NaN;
    end
    if ~exist('sim_time', 'var')
        sim_time = NaN;
    end

	% decode limits
    acc_lim_x = limits(1);
	acc_lim_y = limits(2);
    vel_min_x = limits(4);
	vel_min_y = limits(5);
	vel_max_x = limits(7);
	vel_max_y = limits(8);
    sim_period = limits(10);
    
    vel_max_x_corrected = vel_max_x;
	vel_max_y_corrected = vel_max_y;

    if ~isnan(dist_to_goal) && ~isnan(sim_time)
        % "we limit the velocities to those that do not overshoot the goal in **sim_time**"
        % original formulation from DWA
        % vel_max_x_corrected = max([min([vel_max_x, dist_to_goal / sim_time]), vel_min_x])
        % vel_max_y_corrected = max([min([vel_max_y, dist_to_goal / sim_time]), vel_min_y])
        %
        % second version which often starts limiting the velocity commands too late and thus violates accel. limits
        % when decelerating
        % max_vx = 2 * dist_to_goal / acc_lim_x;
        % max_vy = 2 * dist_to_goal / acc_lim_y;
        % vel_max_x_corrected = max([min([vel_max_x, max_vx]), vel_min_x]);
        % vel_max_y_corrected = max([min([vel_max_y, max_vy]), vel_min_y]);
        %
        % third version
        acc_lim_decel = hypot(acc_lim_x, acc_lim_y);
        % maximum initial speed allowing to stop after dist_to_goal given a deceleration rate of "acc_lim_decel"
        speed_init_max = sqrt(2.0 * acc_lim_decel * dist_to_goal);

        % to trim both velocity components proportionally
        vel_vector_angle = atan2(vel_curr(2), vel_curr(1));
        % updated velocity limits that prevent from overshooting the goal pose
        max_vx = cos(vel_vector_angle) * speed_init_max;
        max_vy = sin(vel_vector_angle) * speed_init_max;
        % new limits can only be smaller than the original ones
        vel_max_x_corrected = max([min([vel_max_x, max_vx]), vel_min_x]);
        vel_max_y_corrected = max([min([vel_max_y, max_vy]), vel_min_y]);
    end
    
    % update limits
	limits(7) = vel_max_x_corrected;
	limits(8) = vel_max_y_corrected;
    
    [status, cmd_vel] = adjustTwistWithAccLimits(...
        vel_curr, ...
        limits,   ...
        cmd_vel,  ...
        maintain_vel_components_rate ...
    );
end
