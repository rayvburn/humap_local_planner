function [cmd_vel] = adjustTwistProportional(...
    vel_curr, cmd_vel_backup, ...
    vel_min_x_acc, vel_min_y_acc, vel_min_th_acc, ...
    vel_max_x_acc, vel_max_y_acc, vel_max_th_acc)
% Trims the translational and angular velocity to given limits keeping the proportions between them

    % compute what part of the requested velocity delta is trimmed out due
    % to acceleration limits
    %%% x
    vel_cmd_diff_x = cmd_vel_backup(1) - vel_curr(1);
    if cmd_vel_backup(1) >= vel_curr(1)
        % CASE1 - speeding up along x direction
        vel_lim_diff_x = vel_max_x_acc - vel_curr(1);
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
        vel_lim_diff_y = vel_max_y_acc - vel_curr(2);
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
        vel_lim_diff_th = vel_max_th_acc - vel_curr(3);
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
end

