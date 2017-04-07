% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

% Main Project File
% Run in order to generate all project plots.

clear all;
close all;
% Add export_fig to the path, so that we can export figures
addpath('export_fig')

disp('Welcome to Enrique Fernandez 16.322 Stochastic Estimation and Control Final Project: Ultrasound Positioning Sytstem using the Kalman Filter')
disp('This script will generate the project plots in the graphs folder.');

%% Generate Noise Plots
disp('Press any key to generate the noise analysis plots in the graphs folder...');
pause();
setup_vars;
calculate_sensor_noise;
generate_noise_plots;
close all;
disp('Noise analysis plots generated!');

%% Generate Results Plots
disp('Press any key to generate the results plots in the graphs folder...');
pause();
generate_results;
close all
disp('Results plots generated!');
disp('All project plots have been generated!');
