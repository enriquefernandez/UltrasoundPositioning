% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

% Creates a structure with the data and saves it

captured_data = struct;
captured_data.x = x;
captured_data.errors = errors;
captured_data.t = t;
captured_data.sampling_rate = sampling_rate;

% Ask for name to save data.
var_name = input('Enter variable name to save: ', 's');
if ~isempty(var_name)
    assignin('base', var_name, captured_data);
    save(var_name, var_name);
else
    error('Specify a correct name');
end