clear all;
close all;
clc;

inscribed_radius = 0.275;

% nominal cost scale when far from the goal, e.g., 50
yn = 50;
xn = 2 * inscribed_radius;

% cost scale when at the goal
y0 = 0;
x0 = 0;

% slope of the linear function
a = yn / xn;

% plotting
x = linspace(0.0, 1.0, 100);
y = a * x;
% trim
y = min(y, 50);
y = max(y, 0);

plot(x, y)