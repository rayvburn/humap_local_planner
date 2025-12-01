clear all;
close all;
clc;

% computeVelocityGlobal takes local velocity and a pose

% %% TEST computeVelocityGlobal
% vel_local = [0.15; 0.0; 0.25];
% 
% % 1st case, robot orientation Euler Z 0 deg
% vel_global = computeVelocityGlobal(vel_local, [0.0, 0.0, 0.0])
% 
% % 2nd case: robot orientation Euler, Z 45 deg
% vel_global = computeVelocityGlobal(vel_local, [0.0, 0.0, pi/4])
% 
% % 3rd case: robot orientation Euler, Z -90 deg
% vel_global = computeVelocityGlobal(vel_local, [0.0, 0.0, -pi/2])
% 
% % 4th case: robot orientation Euler, Z +90, local.angular.z negative
% vel_global = computeVelocityGlobal(vel_local, [0.0, 0.0, pi/2])

%% TEST computeVelocityLocal
vel_global = [];
vel_local = [];

% computeVelocityLocal takes global velocity, a pose, and a flag indicating holonomic/nonholonomic drive (v_y)

% 1) going straight ahead along X axis
vel_local = computeVelocityLocal([1.0;  0.0;  0.0], [0.0, 0.0, 0.0])

% 2) going straight ahead along Y axis
vel_local = computeVelocityLocal([0.0;  1.0;  0.0], [0.0, 0.0, pi/2])

% 3) going straight aback Y axis
vel_local = computeVelocityLocal([0.0;  -1.0;  0.0], [0.0, 0.0, -pi/2])

% 4) going straight aback X axis
vel_local = computeVelocityLocal([-1.0;  0.0;  0.0], [0.0, 0.0, pi])

% 5) backwards aback X axis
vel_local = computeVelocityLocal([-1.0;  0.0;  0.0], [0.0, 0.0, 0.0])

% 6) going with sqrt(2) at 45 degrees globally
vel_local = computeVelocityLocal([1.0;  1.0;  0.0], [0.0, 0.0, pi/4])

% 7) fully backwards
vel_local = computeVelocityLocal([-1.0;  -1.0;  0.0], [0.0, 0.0, pi/4])

% 8)
vel_local = computeVelocityLocal([-1.0;  -1.0;  -pi/2], [0.0, 0.0, -3 * pi/4])

%% TEST computeVelocityLocalHolonomic
vel_global = [];
vel_local = [];

vel_local = computeVelocityLocal([1.0;  1.0; pi/2], [0.0, 0.0, 0.0], true)

vel_local = computeVelocityLocal([-1.0;  -1.0; pi/2], [0.0, 0.0, 0.0], true)

vel_local = computeVelocityLocal([0.5;  0.6; pi/4], [0.0, 0.0, pi/4], true)

vel_local = computeVelocityLocal([0.7;  0.5;  pi/4], [0.0, 0.0, -pi/4], true)
