function [force] = computeInternalForce(robot_pose, robot_velocity, goal_pose, speed_desired, mass, relaxation_time)
    robot_goal_dist = goal_pose - robot_pose;
    % orientation is neglected here; zeroing is required due to use of norm
    robot_goal_dist(3) = 0.0;
    to_goal_direction = robot_goal_dist / norm(robot_goal_dist);
    
    ideal_vel_vector = speed_desired * to_goal_direction;
    
    vel_diff = ...
        [ideal_vel_vector(1) ideal_vel_vector(2) ideal_vel_vector(3)] ...
        - [robot_velocity(1) robot_velocity(2) robot_velocity(3)];
    vel_diff(3) = 0.0;
    
    force = mass * (1 / relaxation_time) * vel_diff;
    force(3) = 0.0;
    
    force = [force(1) force(2) force(3)];

    fprintf("[computeInternalForce ] goal dist  : x %2.3f y %2.3f th %2.3f\n", ...
        robot_goal_dist(1), robot_goal_dist(2), robot_goal_dist(3) ...
    );
    fprintf("[computeInternalForce ] to_goal_dir: x %2.3f y %2.3f th %2.3f\n", ...
        to_goal_direction(1), to_goal_direction(2), to_goal_direction(3) ...
    );
    fprintf("[computeInternalForce ] ideal_vel  : x %2.3f y %2.3f th %2.3f\n", ...
        ideal_vel_vector(1), ideal_vel_vector(2), ideal_vel_vector(3) ...
    );
    fprintf("[computeInternalForce ] vel global : x %2.3f y %2.3f th %2.3f\n", ...
        robot_velocity(1), robot_velocity(2), robot_velocity(3) ...
    );
    fprintf("[computeInternalForce ] vel_diff   : x %2.3f y %2.3f th %2.3f\n", ...
        vel_diff(1), vel_diff(2), vel_diff(3) ...
    );
end
