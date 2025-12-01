function [] = drawVelocity(pose, vel, color, new_fig)
    if new_fig
       figure; 
    end
    
    % current pose (dot)
    plot(pose(1), pose(2), 'o', 'MarkerSize', 3);
    hold on;
    
    % pose prep
    yaw = pose(3);
    POSE_MARKER_LENGTH = 0.08;
    pose_marker = [cos(yaw), sin(yaw)] * POSE_MARKER_LENGTH;

    % velocity
    quiver(...
        [pose(1), pose(1)], ... % X
        [pose(2), pose(2)], ... % Y
        [pose_marker(1), vel(1)],...% U
        [pose_marker(2), vel(2)]...% V
    );
end