% ChatGPT generated this script

% Test values
x = -1.0; % X-coordinate of the point
y = -2.0; % Y-coordinate of the point
centerX = 0.0; % X-coordinate of the center of the ellipse
centerY = 0.0; % Y-coordinate of the center of the ellipse
semiMajorAxis = 5.0; % Semi-major axis of the ellipse
semiMinorAxis = 3.0; % Semi-minor axis of the ellipse
angle = 45.0; % Angle in degrees by which the ellipse is rotated

% Determine the position of the point relative to the ellipse
position = pointPositionRelativeToEllipse(x, y, centerX, centerY, semiMajorAxis, semiMinorAxis, angle);

% Plot the ellipse
t = linspace(0, 2*pi, 100);
ellipse_x = centerX + semiMajorAxis * cos(t) * cosd(angle) - semiMinorAxis * sin(t) * sind(angle);
ellipse_y = centerY + semiMajorAxis * cos(t) * sind(angle) + semiMinorAxis * sin(t) * cosd(angle);
plot(ellipse_x, ellipse_y);
hold on;

% Plot the point of interest
plot(x, y, 'ro');

% Plot the major axis of the ellipse
quiver(centerX, centerY, semiMajorAxis * cosd(angle), semiMajorAxis * sind(angle), 'LineWidth', 2, 'Color', 'blue');
hold on;

% Plot the vector from the center to the point
quiver(centerX, centerY, x - centerX, y - centerY, 'LineWidth', 1, 'Color', 'green');

% Set plot labels and title
xlabel('X-axis');
ylabel('Y-axis');
title('Point Position Relative to Ellipse');

% Display legend for the plot
legend('Ellipse', 'Point', 'Major Axis', 'Center to Point');

disp(['The point is ', position]);

function angle_degrees = angleBetweenVectors(u, v)
    % Calculate the dot product
    dot_product = dot(u, v);

    % Calculate the magnitudes of the vectors
    magnitude_u = norm(u);
    magnitude_v = norm(v);
    
    % Calculate the angle in radians
    % angle_radians = atan2(norm(cross(u,v)), dot(u,v));
    % Not used as cross function is intended to be used with 3D vectors
    
    % Calculate the angle in radians
    angle_radians = atan2(u(1) * v(2) - u(2) * v(1), u(1) * v(1) + u(2) * v(2));
    
    % Convert angle to degrees
    angle_degrees = angle_radians * 180.0 / pi;
end

function position = pointPositionRelativeToEllipse(x, y, h, k, a, b, theta)
    % Convert theta to radians
    theta = theta * pi / 180.0;

    % Vector from center of the ellipse to the point
    pointVector = [x - h; y - k];

    % Vector representing the major axis of the ellipse
    majorAxisVector = [a * cos(theta); b * sin(theta)];
   
    angle = angleBetweenVectors(majorAxisVector, pointVector)
    if angle > 0
        position = 'On the left of major axis';
    elseif angle <= 0
        position = 'On the right of major axis';
    end
    return;

    % Calculate the dot product
    dotProduct = dot(pointVector, majorAxisVector);
    
    % Determine the relative position
    if dotProduct > 0
        position = 'Right of major axis';
    elseif dotProduct < 0
        position = 'Left of major axis';
    else
        position = 'On major axis';
    end
end

