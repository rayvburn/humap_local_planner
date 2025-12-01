x = 1.0;
y = 1.0;
theta = pi/4;
offset = 0.1;

[x1, y1] = footprintCostKernel(x, y, theta, offset, 0.0);
[x2, y2] = footprintCostKernel(x, y, theta, offset, pi/4);
[x3, y3] = footprintCostKernel(x, y, theta, offset, pi/2);
[x4, y4] = footprintCostKernel(x, y, theta, offset, 3 * pi/4);
[x5, y5] = footprintCostKernel(x, y, theta, offset, pi);
[x6, y6] = footprintCostKernel(x, y, theta, offset, -3 * pi/4);
[x7, y7] = footprintCostKernel(x, y, theta, offset, -pi/2);
[x8, y8] = footprintCostKernel(x, y, theta, offset, -pi/4);

plot([x1, x2, x3, x4, x5, x6, x7, x8], [y1, y2, y3, y4, y5, y6, y7, y8])

function [xk, yk] = footprintCostKernel(x, y, th, offset, angle)
    % based on MapGridCostFunction
    xk = x + offset * cos(th + angle);
    yk = y + offset * sin(th + angle);
    pos = [xk, yk];
end
