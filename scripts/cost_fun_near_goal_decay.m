% reduce scales of MapGridCostFunction-based cost functions when the robot is close to the goal
% See updateLocalCosts method of the planner class

clear all;
close all;
clc;

dists_to_goal = linspace(0, 5, 1000);
FORWARD_PT_DIST = 0.85;
scales = zeros(size(dists_to_goal, 1), size(dists_to_goal, 2));

for i=1:length(dists_to_goal)
    dist_to_goal = dists_to_goal(i);
    if dist_to_goal <= FORWARD_PT_DIST
        scale = 0.0;
    elseif dist_to_goal <= 2 * FORWARD_PT_DIST
        scale = min(max(exp(dist_to_goal - FORWARD_PT_DIST) - 1, 0), 1);
    else
        scale = 1.0; 
    end
    scales(i) = scale;
end

figure
plot(dists_to_goal, scales)

