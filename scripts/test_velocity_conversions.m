clear all;
close all;
clc;

% %% TEST computeVelocityGlobal
% vel_local = [0.15; 0.0; 0.25];
% 
% % 1st case, robot orientation Euler Z 0 deg
% yaw = 0.0;
% pose = [0.0, 0.0, yaw];
% vel_global = computeVelocityGlobal(vel_local, pose)
% 
% % 2nd case: robot orientation Euler, Z 45 deg
% yaw = pi/4;
% pose = [0.0, 0.0, yaw];
% vel_global = computeVelocityGlobal(vel_local, pose)
% 
% % 3rd case: robot orientation Euler, Z -90 deg
% yaw = -pi/2;
% pose = [0.0, 0.0, yaw];
% vel_global = computeVelocityGlobal(vel_local, pose)
% 
% % 4th case: robot orientation Euler, Z +90, local.angular.z negative
% yaw = pi/2;
% pose = [0.0, 0.0, yaw];
% vel_global = computeVelocityGlobal(vel_local, pose)

%% TEST computeVelocityLocal
vel_global = [];
vel_local = [];

% 1) going straight ahead along X axis
yaw = 0.0;
pose = [0.0, 0.0, yaw];
vel_global = [1.0;  0.0;  0.0];
vel_local = computeVelocityLocal(vel_global, pose)

% 2) going straight ahead along Y axis
yaw = pi/2;
pose = [0.0, 0.0, yaw];
vel_global = [0.0;  1.0;  0.0];
vel_local = computeVelocityLocal(vel_global, pose)

% 3) going straight aback Y axis
yaw = -pi/2;
pose = [0.0, 0.0, yaw];
vel_global = [0.0;  -1.0;  0.0];
vel_local = computeVelocityLocal(vel_global, pose)

% 4) going straight aback X axis
yaw = pi;
pose = [0.0, 0.0, yaw];
vel_global = [-1.0;  0.0;  0.0];
vel_local = computeVelocityLocal(vel_global, pose)

% 5) backwards aback X axis
yaw = 0.0;
pose = [0.0, 0.0, yaw];
vel_global = [-1.0;  0.0;  0.0];
vel_local = computeVelocityLocal(vel_global, pose)

% 6) going with sqrt(2) at 45 degrees globally
yaw = pi/4;
pose = [0.0, 0.0, yaw];
vel_global = [1.0;  1.0;  0.0];
vel_local = computeVelocityLocal(vel_global, pose)

% 7) fully backwards
yaw = pi/4;
pose = [0.0, 0.0, yaw];
vel_global = [-1.0;  -1.0;  0.0];
vel_local = computeVelocityLocal(vel_global, pose)

% 8)
yaw = -3 * pi/4;
pose = [0.0, 0.0, yaw];
vel_global = [-1.0;  -1.0;  -pi/2];
vel_local = computeVelocityLocal(vel_global, pose)

% 9)
yaw = -pi/4;
pose = [0.0, 0.0, yaw];
vel_global = [0.7;  0.5;  pi/4];
vel_local = computeVelocityLocal(vel_global, pose)

%% TEST computeVelocityLocalHolonomic
vel_global = [];
vel_local = [];

pose = [0.0, 0.0, 0.0];
vel_global = [1.0;  1.0; pi/2];
vel_local = computeVelocityLocal(vel_global, pose)

pose = [0.0, 0.0, 0.0];
vel_global = [-1.0;  -1.0; pi/2];
vel_local = computeVelocityLocal(vel_global, pose)

pose = [0.0, 0.0, pi/4];
vel_global = [0.5;  0.6; pi/4];
vel_local = computeVelocityLocal(vel_global, pose)
