close all;
clear all;

% Ellipse parameters
h = 2; % X-coordinate of the center of the ellipse
k = -1; % Y-coordinate of the center of the ellipse
a = 5; % Semi-major axis of the ellipse
b = 3; % Semi-minor axis of the ellipse
angle = 0.0; % Angle in degrees by which the line is inclined
ellipse_angle = 45.0; % Angle of rotation of the ellipse

% Find the intersection point for the line with the given direction and ellipse rotation
%intersection_point = findEllipseLineIntersectionLocal(h, k, a, b, angle, ellipse_angle);
intersection_point = findEllipseLineIntersection(h, k, a, b, angle, ellipse_angle);

% Plot the ellipse
t = linspace(0, 2*pi, 100);
ellipse_x = h + a * cos(t) * cosd(ellipse_angle) - b * sin(t) * sind(ellipse_angle);
ellipse_y = k + a * cos(t) * sind(ellipse_angle) + b * sin(t) * cosd(ellipse_angle);
plot(ellipse_x, ellipse_y);
hold on;
plot(h, k, 'r*', 'MarkerSize', 10);

% Plot the intersection point
plot(intersection_point(1), intersection_point(2), 'ro', 'MarkerSize', 10);

% Set plot labels and title
xlabel('X-axis');
ylabel('Y-axis');
title('Intersection Point of Rotated Ellipse and Line from Center');

% Display legend for the plot
legend('Ellipse', 'Intersection Point');


% trying to find a safe candidate for a recovery position
PERSON_MODEL_RADIUS = 2.28;
is_pt_dir = atan2(intersection_point(2) - k, intersection_point(1) - h);
extension_point = intersection_point ...
    + [PERSON_MODEL_RADIUS * cos(is_pt_dir); PERSON_MODEL_RADIUS * sin(is_pt_dir)];
plot(extension_point(1), extension_point(2), 'go', 'MarkerSize', 10);


% This one is not perfect but find the intersection points of rotated
% ellipses good enough. For ellipse_angle = 0, ellipse_angle = +/-90,
% works perfectly
% Expects "theta" expressed in the ellipse's local coordinate system
function intersection_point = findEllipseLineIntersectionLocal(h, k, a, b, theta, ellipse_angle)   
    % Convert angles to radians
    theta = theta * pi / 180.0;
    ellipse_angle = ellipse_angle * pi / 180.0;

    % Calculate the intersection point
    x = h + a * cos(theta) * cos(ellipse_angle) - b * sin(theta) * sin(ellipse_angle);
    y = k + a * cos(theta) * sin(ellipse_angle) + b * sin(theta) * cos(ellipse_angle);

    intersection_point = [x; y];
end

% Expects "theta" expressed in the global coordinate system
function intersection_point = findEllipseLineIntersection(h, k, a, b, theta, ellipse_angle)   
    % Convert angles to radians
    theta = theta * pi / 180.0;
    ellipse_angle = ellipse_angle * pi / 180.0;
    
    theta = wrapToPi(theta - ellipse_angle);

    % Calculate the intersection point
    x = h + a * cos(theta) * cos(ellipse_angle) - b * sin(theta) * sin(ellipse_angle);
    y = k + a * cos(theta) * sin(ellipse_angle) + b * sin(theta) * cos(ellipse_angle);

    intersection_point = [x; y];
end