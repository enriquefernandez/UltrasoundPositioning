% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

function plot_measurements(x)

figure
subplot(3,1,1);
plot(x(1,:));
ylim([0 7500])

subplot(3,1,2);
plot(x(2,:))
ylim([0 7500])

subplot(3,1,3);
plot(x(3,:));
ylim([0 7500])