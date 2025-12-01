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

	cmd_vel_backup = cmd_vel;
   
    if ~maintain_vel_components_rate
        cmd_vel(1) = min([max([vel_min_x_acc, cmd_vel_backup(1)]), vel_max_x_acc]);
        cmd_vel(2) = min([max([vel_min_y_acc, cmd_vel_backup(2)]), vel_max_y_acc]);
        cmd_vel(3) = min([max([vel_min_th_acc, cmd_vel_backup(3)]), vel_max_th_acc]);
	    %fprintf("[adjustTwistWithAccLimits] cmd vel 1st {x %6.3f, y %6.3f, th %6.3f}\r\n", cmd_vel(1), cmd_vel(2), cmd_vel(3));
        status = true;
        return; 
    end
    
    cmd_vel = adjustTwistProportional(vel_curr, cmd_vel_backup, ...
        vel_min_x_acc, vel_min_y_acc, vel_min_th_acc, ...
        vel_max_x_acc, vel_max_y_acc, vel_max_th_acc ...
    );
    status = true;
end
