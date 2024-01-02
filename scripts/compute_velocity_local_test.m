clear;
clc;

% entries from unit test:
% TEST(HumapVelocityConversions, computeVelocityLocal)

% 1) going straight ahead along X axis
fprintf("1) "); computeVelocityLocal([ 1.0;  0.0;  0.0 ], [nan; nan;    0.0  ]);
% 2) going straight ahead along Y axis
fprintf("2) "); computeVelocityLocal([ 0.0;  1.0;  0.0 ], [nan; nan;    pi/2 ]);
% 3) going straight aback Y axis
fprintf("3) "); computeVelocityLocal([ 0.0; -1.0;  0.0 ], [nan; nan;   -pi/2 ]);
% 4) going straight aback X axis
fprintf("4) "); computeVelocityLocal([-1.0;  0.0;  0.0 ], [nan; nan;    pi   ]);
% 5) backwards aback X axis
fprintf("5) "); computeVelocityLocal([-1.0;  0.0;  0.0 ], [nan; nan;    0.0  ]);
% 6) going with sqrt(2) at 45 degrees globally
fprintf("6) "); computeVelocityLocal([ 1.0;  1.0;  0.0 ], [nan; nan;    pi/4 ]);
% 7) fully backwards
fprintf("7) "); computeVelocityLocal([-1.0; -1.0;  0.0 ], [nan; nan;    pi/4 ]);
% 8)
fprintf("8) "); computeVelocityLocal([-1.0; -1.0; -pi/2], [nan; nan; -3*pi/4 ]);

function v_local = computeVelocityLocal(v_global, pose)
    yaw = pose(3);
    R = [cos(yaw), sin(yaw), 0;
                0,        0, 0;
                0,        0, 1];
    v_local = R * v_global;
    fprintf(...
        "v_local {x %7.3f, y %7.3f, omega: %7.3f}\n", ...
        v_local(1), ...
        v_local(2), ...
        v_local(3) ...
    );
end
