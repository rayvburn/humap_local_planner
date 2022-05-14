clc;
clear all;
close all;

% shared parameters
SIM_PERIOD = 1.0; % a.k.a. `dt`
ROBOT_MASS = 1.0;
MIN_VEL_X = 0.0;
MAX_VEL_X = 1.5;
MAX_ROT_VEL = 2.0;

% sim-specific
ITERATIONS = 70;

% ASSUMPTION of global velocity to twist conversion:
% constant force, in a steady state, produces velocity of direction that
% matches force direction
%%%%%%%%%%%%%%%%%%%%%%%% cases %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: assertion function takes `vel_global` variable from workspace
% 1
fprintf("\n\n[CASE] 1\n");
pose_2d = [0.0, 0.0, deg2rad(90)];
vel_local = [1.0, 0.0, 0.65];
force = [+0.2, -0.1, 0.0];
computeTwistFcn = @computeTwist;
run('compute_twist_sim.m');
assertTwist(force, vel_global);

% 2
fprintf("\n\n[CASE] 2\n");
pose_2d = [0.0, 0.0, deg2rad(180)];
vel_local = [0.0, 0.0, 0.05];
force = [+0.2, -0.1, 0.0];
computeTwistFcn = @computeTwist;
run('compute_twist_sim.m');
assertTwist(force, vel_global);

% 3
fprintf("\n\n[CASE] 3\n");
pose_2d = [0.0, 0.0, deg2rad(90)];
vel_local = [0.0, 0.0, 0.00];
force = [+1.2, +0.2, 0.0];
computeTwistFcn = @computeTwist;
run('compute_twist_sim.m');
assertTwist(force, vel_global);

% 4
fprintf("\n\n[CASE] 4\n");
pose_2d = [0.0, 0.0, deg2rad(-90)];
vel_local = [0.0, 0.0, 0.0];
force = [+0.5, +0.25, 0.0];
computeTwistFcn = @computeTwist;
run('compute_twist_sim.m');
assertTwist(force, vel_global);

% 4b: 4 with angular correction
fprintf("\n\n[CASE] 4 with angular compensation\n");
pose_2d = [0.0, 0.0, deg2rad(-90)];
vel_local = [0.0, 0.0, 0.0];
force = [+0.5, +0.25, 0.0];
computeTwistFcn = @computeTwistAngCompensation;
run('compute_twist_sim.m');
assertTwist(force, vel_global);

function [] = assertTwist(force, vel_global)
    % condition and message
    assert_diff = atan2(force(2), force(1)) - atan2(vel_global(2), vel_global(1));
    assert(abs(assert_diff) < 1e-03, ...
        "Direction of global velocity does not match direction of the force. Is robot in a steady state? Consider increasing ITERATIONS value" ...
    );
end