function [status, cmd_vel] = adjustTwistWithAccLimits( ...
	vel_curr, ...
	limits,   ...
    cmd_vel,  ...
    maintain_vel_components_rate ...
)
    %
    % Adjusts velocity command with acceleration limits
    % Based on the current velocity `vel_curr`, acceleration limits and sim_period, recomputes `cmd_vel`
    % Continuous acceleration assumption is used here.
    % @return True if @ref cmd_vel was modified to comply with acceleration limits
    %
    % maintain_vel_components_rate - when true and any vel. component is not 
    %   within limits, others will be modified according to the one that cannot 
    %   be changed as requested; others will be proportionally scaled; setting
    %   this to true loosely corresponds to agreeing with dynamics violation,
    %   but keeping the path as intended

	status = false;
    if ~exist('maintain_vel_components_rate', 'var')
        maintain_vel_components_rate = true;
    end

	% decode limits
	acc_lim_x = limits(1);
	acc_lim_y = limits(2);
	acc_lim_th = limits(3);
	vel_min_x = limits(4);
	vel_min_y = limits(5);
	vel_min_th = limits(6);
	vel_max_x = limits(7);
	vel_max_y = limits(8);
	vel_max_th = limits(9);
	sim_period = limits(10);

	%fprintf("[adjustTwistWithAccLimits] vel in x %6.3f, y %6.3f, th %6.3f | dt %6.3f\r\n", vel_curr(1), vel_curr(2), vel_curr(3), sim_period);

	vel_min_x_acc = max([vel_min_x, vel_curr(1) - acc_lim_x * sim_period]);
	vel_min_y_acc = max([vel_min_y, vel_curr(2) - acc_lim_y * sim_period]);
	vel_min_th_acc = max([vel_min_th, vel_curr(3) - acc_lim_th * sim_period]);
	%fprintf("[adjustTwistWithAccLimits] vel_min_x_acc %6.3f, vel_min_y_acc %6.3f, vel_min_th_acc %6.3f\r\n", vel_min_x_acc, vel_min_y_acc, vel_min_th_acc);

	vel_max_x_acc = min([vel_max_x, vel_curr(1) + acc_lim_x * sim_period]);
	vel_max_y_acc = min([vel_max_y, vel_curr(2) + acc_lim_y * sim_period]);
	vel_max_th_acc = min([vel_max_th, vel_curr(3) + acc_lim_th * sim_period]);
	%fprintf("[adjustTwistWithAccLimits] vel_max_x_acc %6.3f, vel_max_y_acc %6.3f, vel_max_th_acc %6.3f\r\n", vel_max_x_acc, vel_max_y_acc, vel_max_th_acc);

	% if any component is modified, change other ones proportionally
	cmd_vel_backup = cmd_vel;
	cmd_vel(1) = min([max([vel_min_x_acc, cmd_vel_backup(1)]), vel_max_x_acc]);
	cmd_vel(2) = min([max([vel_min_y_acc, cmd_vel_backup(2)]), vel_max_y_acc]);
	cmd_vel(3) = min([max([vel_min_th_acc, cmd_vel_backup(3)]), vel_max_th_acc]);
	%fprintf("[adjustTwistWithAccLimits] cmd vel 1st {x %6.3f, y %6.3f, th %6.3f}\r\n", cmd_vel(1), cmd_vel(2), cmd_vel(3));
   
    if ~maintain_vel_components_rate
        status = true;
        return; 
    end
    
    % compute what part of the requested velocity delta is trimmed out due
    % to acceleration limits
    %%% x
    vel_cmd_diff_x = cmd_vel_backup(1) - vel_curr(1);
    if cmd_vel_backup(1) >= vel_curr(1)
        % CASE1 - speeding up along x direction
        vel_lim_diff_x = vel_max_x_acc - vel_curr(1); % ??
        % defines part of the velocity delta that is feasible
        vel_feas_factor_x = vel_lim_diff_x / vel_cmd_diff_x;
    else
        % CASE2 - slowing down along x direction
        vel_lim_diff_x = vel_min_x_acc - vel_curr(1);
        % defines part of the velocity delta that is feasible
        vel_feas_factor_x = vel_lim_diff_x / vel_cmd_diff_x;
    end
    %%% y
    vel_cmd_diff_y = cmd_vel_backup(2) - vel_curr(2);
    if cmd_vel_backup(2) >= vel_curr(2)
        % CASE1 - speeding up along y direction
        vel_lim_diff_y = vel_max_y_acc - vel_curr(2); % ??
        % defines part of the velocity delta that is feasible
        vel_feas_factor_y = vel_lim_diff_y / vel_cmd_diff_y;
    else
        % CASE2 - slowing down along y direction
        vel_lim_diff_y = vel_min_y_acc - vel_curr(2);
        % defines part of the velocity delta that is feasible
        vel_feas_factor_y = vel_lim_diff_y / vel_cmd_diff_y;
    end
	%%% theta
    vel_cmd_diff_th = cmd_vel_backup(3) - vel_curr(3);
    if cmd_vel_backup(3) >= vel_curr(3)
        % CASE1 - speeding up around th axis
        vel_lim_diff_th = vel_max_th_acc - vel_curr(3); % ??
        % defines part of the velocity delta that is feasible
        vel_feas_factor_th = vel_lim_diff_th / vel_cmd_diff_th;
    else
        % CASE2 - slowing down around th axis
        vel_lim_diff_th = vel_min_th_acc - vel_curr(3);
        % defines part of the velocity delta that is feasible
        vel_feas_factor_th = vel_lim_diff_th / vel_cmd_diff_th;
    end
    feasibility_factors = [vel_feas_factor_x, vel_feas_factor_y, vel_feas_factor_th];
    [min_feas_factor, ~] = min(feasibility_factors);
    
    if ~isnan(min_feas_factor) && min_feas_factor < 1
        % correction required
        xtemp = vel_curr(1) + vel_cmd_diff_x * min_feas_factor;
        ytemp = vel_curr(2) + vel_cmd_diff_y * min_feas_factor;
        thtemp = vel_curr(3) + vel_cmd_diff_th * min_feas_factor;
    else
        % within limits
        xtemp = vel_curr(1) + vel_cmd_diff_x;
        ytemp = vel_curr(2) + vel_cmd_diff_y;
        thtemp = vel_curr(3) + vel_cmd_diff_th;
    end
    
    cmd_vel = [xtemp, ytemp, thtemp];
    
    % NOTE: this should not be required...
    % make sure that any not investigated 'sign changes' are included and acceleration limits are satisfied
	cmd_vel(1) = min([max([vel_min_x_acc, cmd_vel(1)]), vel_max_x_acc]);
	cmd_vel(2) = min([max([vel_min_y_acc, cmd_vel(2)]), vel_max_y_acc]);
	cmd_vel(3) = min([max([vel_min_th_acc, cmd_vel(3)]), vel_max_th_acc]);

    status = true;
end
