% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

%% This script sets up the constants for my ultrasound positioning system

% Sampling rate
dt = 0.05; % 20 Hz

% Process Noise
w = 0.05;
W = w*eye(3);

% Sensor noise
% Noise with median filter
r1 = 0.0042^2;
r2 = 0.0043^2;
r3 = 0.0047^2;

R = [r1 0 0;0 r2 0;0 0 r3]*1;

%Sensor distances
l2 = 0.567; %m
l3 = 0.560; %m

%Rectangle size for sensor plot
rec_s = 0.04;

%Localization area size
rec_large = 0.618;