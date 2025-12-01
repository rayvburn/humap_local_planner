function [cmd_vel] = saturateVelocity(cmd_vel, max_vel_x, max_vel_y, max_vel_trans, max_vel_theta, max_vel_x_backwards)
% Calculation method has been taken from TebLocalPlannerROS::saturateVelocity
% https://github.com/rst-tu-dortmund/teb_local_planner/blob/3b91dcc3b9e6733363e285e096ff7c7fea42883f/include/teb_local_planner/teb_local_planner_ros.h#L359
	ratio_x = 1.0;
	ratio_omega = 1.0;
	ratio_y = 1.0;

	vx = cmd_vel(1);
	vy = cmd_vel(2);
	omega = cmd_vel(3);

	% Limit translational velocity for forward driving
	if (vx > max_vel_x)
		ratio_x = max_vel_x / vx;
    end

	% limit strafing velocity
	if (vy > max_vel_y || vy < -max_vel_y)
		ratio_y = abs(vy / max_vel_y);
    end

	% Limit angular velocity
	if (omega > max_vel_theta || omega < -max_vel_theta)
		ratio_omega = abs(max_vel_theta / omega);
    end

	% Limit backwards velocity
	if (vx < -abs(max_vel_x_backwards))
		ratio_x = -abs(max_vel_x_backwards) / vx;
    end

	% TEB sets use_proportional_saturation to false by default, therefore appropriate version applied
	vx = vx * ratio_x;
	vy = vy * ratio_y;
	omega = omega * ratio_omega;

	vel_linear = hypot(vx, vy);
	if (vel_linear > max_vel_trans)
		max_vel_trans_ratio = max_vel_trans / vel_linear;
		vx = vx * max_vel_trans_ratio;
		vy = vy * max_vel_trans_ratio;
    end
    
    cmd_vel = [vx, vy, omega];
end
