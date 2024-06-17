clear all;
close all;
clc;

angle_mean = pi/2;
angle_range = pi/4;
% 2 sigma rule applied to the range
angle_stddev = angle_range / 2;
angle_variance = angle_stddev * angle_stddev;

% mirrored
[Xcross, gauss_cross_pos, ~, ~] = prepareGaussianAngleDomain(+angle_mean, angle_variance);
[~, gauss_cross_neg, ~, ~] = prepareGaussianAngleDomain(-angle_mean, angle_variance);
figure_cross = figure('Name', 'Gaussian visualization');

gauss_cross = max(gauss_cross_pos, gauss_cross_neg);
Xcrossdeg = Xcross * 180 / pi;
gauss_cross_norm = gauss_cross / max(gauss_cross);
plot(Xcrossdeg, gauss_cross_norm);

% frontal detections
front_mean = 0.0;
front_range = deg2rad(180.0);
% 2 sigma rule applied to the range
front_stddev = front_range / 2;
front_variance = front_stddev * front_stddev;
figure_cross = figure('Name', 'Front detections Gaussian visualization');

[Xfront, gauss_front_pos, ~, ~] = prepareGaussianAngleDomain(+front_mean, front_variance);
Xfrontdeg = Xcross * 180 / pi;
gauss_front_pos_norm = gauss_front_pos / max(gauss_front_pos);
plot(Xfrontdeg, gauss_front_pos_norm);


% NOTE: Copied from Matlab resources related to "benchmark"
% Prepares mesh with X coords and Gaussian value in each point,
% Additionally, prepares 2-sigma range of angles and their Gaussians
function [X, gauss, X2sigma, gauss2sigma] = prepareGaussianAngleDomain(...
    mu, variance, resolution, angle_from, angle_to)

    if ~exist('resolution', 'var')
        resolution = 0.01;
    end
    if ~exist('angle_from', 'var')
        angle_from = -pi;
    end
    if ~exist('angle_to', 'var')
        angle_to = pi;
    end
    
    stddev = sqrt(variance);
    if angle_from > angle_to
        X1 = (angle_from):resolution:(pi);
        X2 = (pi):resolution:(angle_to + 2*pi);
        X = [X1, X2];
    else
        X = (angle_from):resolution:(angle_to);
    end
    gauss = zeros(1, length(X));
    
    X2sigma = [];
    gauss2sigma = [];
    
    for i = 1:length(X)
        % angle domain Gaussian
        % not friendly for plotting
        % X(i) = wrapToPi(X(i));
        gauss_neg = normpdf(X(i), mu - 2*pi, stddev);
        gauss_mid = normpdf(X(i), mu       , stddev);
        gauss_pos = normpdf(X(i), mu + 2*pi, stddev);
        gauss(i) = max([gauss_neg, gauss_mid, gauss_pos]);
        
        if X(i) >= (mu - 2 * stddev) && X(i) <= (mu + 2 * stddev)
            X2sigma = [X2sigma, X(i)];
            gauss2sigma = [gauss2sigma, gauss(i)];
        end
    end
end
