% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

function x = simple_estimate(y)
% Takes microseconds and returns x, y, z
 [m, n] = size(y);

 %Sensor setup
% l2 = 0.575; %m
% l3 = 0.565; %m
l2 = 0.567; %m
l3 = 0.560; %m

 d = y * 1e-6 * 340.29;
 
 d2 = d.^2;
 
 x = zeros(m, n); 
 
 x(1,:) = (d2(1,:) - d2(2,:) + l2^2) / (2*l2);
 x(2,:) = (d2(1,:) - d2(3,:) + l3^2) / (2*l3);
 x(3,:) = real( sqrt(d2(1,:) - x(1,:).^2 - x(2,:).^2));
 
end
