% convert 2D forces into robot forces with non-holonomic contraints
function cmd_vel = computeTwist(...
    pose,...
	force,...
	robot_vel_glob,...
	sim_period,...
	robot_mass,...
	min_vel_x,...
	max_vel_x,...
	max_rot_vel,...
    twist_rotation_compensation ...
)   
    force_v = force;
	dt = sim_period;
    
    % force calculated in the current step affects robot motion
	acc_v = force_v / robot_mass;
	vel_v = acc_v * dt;
	dvel_v = vel_v - robot_vel_glob;
    
    fprintf("[     computeTwist    ] force = [%2.4f, %2.4f, %2.4f], vel_prev = [%2.4f, %2.4f, %2.4f]\n",...
        force_v(1), force_v(2), force_v(3),...
        robot_vel_glob(1), robot_vel_glob(2), robot_vel_glob(3)...
	);
    fprintf("[     computeTwist    ] dvel  = [%2.4f, %2.4f, %2.4f], vel_new  = [%2.4f, %2.4f, %2.4f]\n",...
        dvel_v(1), dvel_v(2), dvel_v(3), ...
        vel_v(1), vel_v(2), vel_v(3) ...
	);
    
    % ref: https://github.com/yinzixuan126/modified_dwa/blob/b379c01e37adc1f6414005750633b05e1a024ae5/iri_navigation/iri_akp_local_planner_companion/local_lib/src/scene_elements/robot.cpp#L91
    % projection to robot pose (dot product)
    lin_x = +cos(pose(3)) * vel_v(1) + sin(pose(3)) * vel_v(2);
    % cross product yaw x velocity
    ang_z = -sin(pose(3)) * vel_v(1) + cos(pose(3)) * vel_v(2);

    % compensation of the slow rotation in twist command
    % zero out the `twist_rotation_compensation` if this feature is
    % not used
    force_dir = atan2(force_v(2), force_v(1));
    ang_z_force_diff = force_dir - pose(3);
    ang_z = ang_z + twist_rotation_compensation * ang_z_force_diff;

    cmd_vel = [lin_x, 0, ang_z];

    force_dir = atan2(force(2), force(1));
    fprintf("[     computeTwist    ] force dir %2.3f rad (%2.2f°), x: %2.2f y: %2.2f \n", ...
        force_dir, rad2deg(force_dir), cos(force_dir) * norm(force), sin(force_dir) * norm(force) ...
    );
    fprintf("[     computeTwist    ] loc vel dir %2.3f rad (%2.2f°), x: %2.2f th: %2.2f \n", ...
        pose(3) + ang_z, rad2deg(pose(3) + ang_z), ...
        lin_x, ang_z ...
    );
    fprintf("[     computeTwist    ] linear = %2.4f (max = %2.4f), angular = %2.4f (max = %2.4f)\n",...
        cmd_vel(1),...
        max_vel_x,...
        cmd_vel(3),...
        max_rot_vel...
	);

    % check if within limits: try to maintain path, ignoring trajectory
    if max_vel_x < cmd_vel(1) || max_rot_vel < abs(cmd_vel(3))
       vel_x_excess = cmd_vel(1) / max_vel_x;
       vel_rot_excess = cmd_vel(3) / max_rot_vel;
       vel_shortening_factor = 1 / max(vel_x_excess, vel_rot_excess);
       cmd_vel(1) = cmd_vel(1) * vel_shortening_factor;
       cmd_vel(3) = cmd_vel(3) * vel_shortening_factor;
       fprintf("[     computeTwist    ] trimming cmd_vel (%2.5f) -> cmd_vel = [%2.4f, %2.4f, %2.4f]\n", ...
            vel_shortening_factor, ...
            cmd_vel(1), ...
            cmd_vel(2), ...
            cmd_vel(3) ...
       );
    end
    
    % trim minimum velocity to `min_vel_x`;
    % ignoring this stage may produce `Reeds-Shepp`-alike behaviour (going
    % backwards while dramatically changing direction)
    if min_vel_x > cmd_vel(1)
        fprintf("[     computeTwist    ] trimming cmd_vel.x from %2.5f to %2.5f\n", ...
            cmd_vel(1), ...
            min_vel_x ...
        );
        cmd_vel(1) = min_vel_x;
    end
end
