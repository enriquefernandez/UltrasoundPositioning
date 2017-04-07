% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

function plot_noise(var, j)

figure
dt = 1/var.sampling_rate;
N = size(var.x,2);
t = 0:dt:dt*N;
t = t(1:N);
h1 = plot(t, var.x(j,:),'b');
hold on;
h2 = plot(t, medfilt1( var.x(j,:)'), 'r-.', 'LineWidth',1.5);

legend([h1 h2], 'Raw Sensor', 'Median');
xlabel('time (s)')
ylabel('Sensor value');
title('Raw Sensor vs Median Filtered')