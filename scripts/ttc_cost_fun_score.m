% ttc_cost_fun_score.m
%
% Visualises TTCCostFunction::computeCost
%

close all;
clear all;
clc;

time_plan_horizon = 4.0;
x_ttc = linspace(0, time_plan_horizon, 100);
% compute cost values
for i=1:length(x_ttc)
    y_ttc_v1(i) = time_plan_horizon / x_ttc(i);
    y_ttc_v2(i) = 1 - x_ttc(i) / time_plan_horizon;
end

figure('Name', 'V1');
plot(x_ttc, y_ttc_v1)
figure('Name', 'V2');
plot(x_ttc, y_ttc_v2);
