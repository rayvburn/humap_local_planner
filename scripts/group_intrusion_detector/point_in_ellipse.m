% ChatGPT generated this script

% Test values
x = 2.0; % X-coordinate of the point
y = 3.0; % Y-coordinate of the point
centerX = 0.0; % X-coordinate of the center of the ellipse
centerY = 0.0; % Y-coordinate of the center of the ellipse
semiMajorAxis = 5.0; % Semi-major axis of the ellipse
semiMinorAxis = 3.0; % Semi-minor axis of the ellipse
angle = 12.0; % Angle in degrees by which the ellipse is rotated

% Check if the point is inside the ellipse
isInside = pointInEllipse(x, y, centerX, centerY, semiMajorAxis, semiMinorAxis, angle);

% Plot the ellipse
t = linspace(0, 2*pi, 100);
ellipse_x = centerX + semiMajorAxis * cos(t) * cosd(angle) - semiMinorAxis * sin(t) * sind(angle);
ellipse_y = centerY + semiMajorAxis * cos(t) * sind(angle) + semiMinorAxis * sin(t) * cosd(angle);
plot(ellipse_x, ellipse_y);
hold on;

% Plot the point of interest
plot(x, y, 'ro');

% Set plot labels and title
xlabel('X-axis');
ylabel('Y-axis');
title('Point within Ellipse');

% Display legend for the plot
legend('Ellipse', 'Point');

if isInside
    disp('The point is inside the ellipse.');
else
    disp('The point is outside the ellipse.');
end

function inside = pointInEllipse(x, y, h, k, a, b, theta)
    % Convert theta to radians
    theta = theta * pi / 180.0;

    % Calculate the value of the equation for the given point
    value = (((x - h) * cos(theta) + (y - k) * sin(theta)) / a)^2 ...
          + (((x - h) * sin(theta) - (y - k) * cos(theta)) / b)^2;

    % Check if the point is inside the ellipse
    inside = value <= 1.0;
end
