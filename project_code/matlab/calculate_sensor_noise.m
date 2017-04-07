% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

% Calculates the sensor noise using several measured runs over fixed points
load sensor_noise.mat
data_vars = {fixed_aroundA, fixed_aroundB, top_s2, top_s3, top_s1};

n = length(data_vars);

noise_matrix = zeros(n+1, 3);

aggregated_data = [];

for j=1:n
    data = data_vars{j}.x * 340.29e-6;
    noise_matrix(j, :) = std(medfilt1(data'));
    
    data = medfilt1(data')';
    data_mean = mean(data');
    error_data = data' - data_mean(ones(size(data,2),1),:);
    aggregated_data = [aggregated_data; error_data];
%     noise_matrix(j, :) = std(data');
end
noise_matrix(n+1, :) = mean(noise_matrix(1:n,:))