close all;
clear all;
clc;


% select between local/global velocities for comparison with velocities
% calculation method from subsequent poses
verify_with_local_velocities = true;

ITER_NUM = 8;

traj_dt = 0.25;
traj_xv = 1.0;
traj_yv = 0.5;
traj_thetav = 1.00;

init_dir = atan2(traj_yv, traj_xv);
pose = [0; 0; init_dir];

vel_local = [traj_xv; traj_yv; traj_thetav];
% global velocities differ over time when local velocity is constant
vels_global = [];

% global/local -> computed from global/local velocities
pose0_global = pose;
pose0_local = pose;

poses_global = [pose0_global];
poses_local = [pose0_local];

for i=1:ITER_NUM
    % compute global velocity as it differs in each time step, given that
    % the local velocity is constant
    vel_global = computeVelocityGlobal(vel_local, poses_local(:, end));
    vels_global = [vels_global, vel_global];
    
    % compare initially computed local velocity with the inverse transformation
    % from the global vector
    if vel_local ~= computeVelocityLocal(vel_global, poses_local(:, end))
        error("Inverse transformation of velocity vector (into local c.s.) is invalid");
    end
    
    p_global = computeNextPose(poses_global(:, end), vel_global, traj_dt);
    poses_global = [poses_global, p_global];
    
    p_local = computeNextPoseBaseVel(poses_local(:, end), vel_local, traj_dt);
    poses_local = [poses_local, p_local];
end

% collect data for quiver plots
Xq_global = [];
Yq_global = [];
Uq_global = [];
Vq_global = [];

Xq_local = [];
Yq_local = [];
Uq_local = [];
Vq_local = [];

for i=1:length(poses_global)
    p_global = poses_global(:, i);
    [x, y, u, v] = poseToQuiver(p_global, traj_dt);
    Xq_global = [Xq_global, x];
    Yq_global = [Yq_global, y];
    Uq_global = [Uq_global, u];
    Vq_global = [Vq_global, v];
    
    p_local = poses_local(:, i);
    [x, y, u, v] = poseToQuiver(p_local, traj_dt);
    Xq_local = [Xq_local, x];
    Yq_local = [Yq_local, y];
    Uq_local = [Uq_local, u];
    Vq_local = [Vq_local, v];
end

figure;
plot(pose(1), pose(2), 'ko', 'MarkerSize', 8)
hold on;
quiver(Xq_global, Yq_global, Uq_global, Vq_global, 'g', 'LineWidth', 5)
quiver(Xq_local, Yq_local, Uq_local, Vq_local, 'r', 'LineWidth', 2)
legend("pos init", "glob. vel", "local vel", "Location", "best")


% pose addition/subtraction test
% simulates actions performed in World::predict
diff_1_2 = subtractPoses(poses_local(:, 2), poses_local(:, 1));
poses_local_2_check = addPoses(poses_local(:, 1), diff_1_2);
[x_check, y_check, u_check, v_check] = poseToQuiver(poses_local_2_check, traj_dt);
quiver(x_check, y_check, u_check, v_check, 'bo')

diff_4_5 = subtractPoses(poses_local(:, 5), poses_local(:, 4));
poses_local_5_check = addPoses(poses_local(:, 4), diff_4_5);
[x_check, y_check, u_check, v_check] = poseToQuiver(poses_local_5_check, traj_dt);
quiver(x_check, y_check, u_check, v_check, 'bo')

diff_begin_end = subtractPoses(poses_local(:, ITER_NUM), poses_local(:, 1));
poses_local_begin_end = addPoses(poses_local(:, 1), diff_begin_end);
[x_check, y_check, u_check, v_check] = poseToQuiver(poses_local_begin_end, traj_dt);
quiver(x_check, y_check, u_check, v_check, 'bo')


% also, check velocity calculation from trajectory points
% select arbitrary "dataset"
if verify_with_local_velocities
    % poses
    poses_verif = poses_local;
    % velocities
    velones = ones(1, length(poses_local) - 1);
    vel_verif_ref = vel_local * velones;
else
    % poses
    poses_verif = poses_global;
    % velocities
    vel_verif_ref = vels_global;
end
vels_verif = [];
t_verif = [];

% check all robot trajectory points ...
for i=2:length(poses_verif)
    % retrieve poses
    x = poses_verif(1, i);
    y = poses_verif(2, i);
    th = poses_verif(3, i);
    % eval robot displacement
    x_prev = poses_verif(1, i - 1);
    y_prev = poses_verif(2, i - 1);
    th_prev = poses_verif(3, i - 1);
    
    if verify_with_local_velocities
        vel = computeBaseVelocityFromPoses( ...
            [x_prev; y_prev; th_prev], ...
            [x; y; th], ...
            traj_dt ...
        );
    else
        vel = computeVelocityFromPoses( ...
            [x_prev; y_prev; th_prev], ...
            [x; y; th], ...
            traj_dt ...
        );
    end
    
    vels_verif = [vels_verif, vel];
    t_verif = [t_verif, i * traj_dt];
end

% reference velocities vs velocities to verify
figure("Position", [89, 7, 1849, 413], "Name", "Velocities from subsequent poses");

ax_vx = subplot(1, 3, 1);
plot(t_verif, vel_verif_ref(1,:), 'LineWidth', 3);
hold on;
plot(t_verif, vels_verif(1,:));
legend("Vx ref", "Vx verif")

ax_vy = subplot(1, 3, 2);
plot(t_verif, vel_verif_ref(2,:), 'LineWidth', 3);
hold on;
plot(t_verif, vels_verif(2,:));
legend("Vy ref", "Vy verif")

ax_vth = subplot(1, 3, 3);
plot(t_verif, vel_verif_ref(3,:), 'LineWidth', 3);
hold on;
plot(t_verif, vels_verif(3,:));
legend("Vth ref", "Vth verif")



function [x, y, u, v] = poseToQuiver(pose, arrow_len)
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    dir = R * [arrow_len; 0];
    u = dir(1);
    v = dir(2);
end
