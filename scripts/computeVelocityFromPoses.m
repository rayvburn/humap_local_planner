function [vel] = computeVelocityFromPoses(pose1, pose2, dt)
%COMPUTEVELOCITYFROMPOSES Summary of this function goes here
%   Detailed explanation goes here
    
    % a.k.a. 'subtractPoses'
    displacement = [...
        pose2(1) - pose1(1), ...
        pose2(2) - pose1(2), ...
        pose2(3) - pose1(3) ...
    ];

    % compute vel from pose difference
    vel = [displacement(1) / dt;
           displacement(2) / dt;
           displacement(3) / dt ...
    ];
end
