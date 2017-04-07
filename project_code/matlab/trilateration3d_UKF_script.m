% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

% To run, set yt to the vector containing the measured time in microseconds
% to each sensor. First row, sensor 1, second row sensor 2, etc. The number
% of samples equals the number of columns.

%% Configuration
%Initial estimate
% X0 = [0.25, 0.36, 1, 0, 0, 0];
X0 = [0.5, 0.5, 1, 0, 0, 0]';
Q0 = diag([ 0.04 0.04 0.01 0.005 0.005 0.005]);
%Sampling rate
dt = 0.25; % Initially 4 Hz

%Sensor setup
% l2 = 0.575; %m
% l3 = 0.565; %m
l2 = 0.567; %m
l3 = 0.560; %m

Y = yt * 340.29e-6; % Microseconds to m.
[nh, N] = size(Y);

%% Noise setup
w = 0.0025 ; % (m/s)^2
%w = 0.25 ; % (m/s)^2
w=0.05;
W = w*eye(3);
% Sensor noise
% Noise with median filter
r1 = 0.0042^2;
r2 = 0.0043^2;
r3 = 0.0047^2;
R = [r1 0 0;0 r2 0;0 0 r3];
% r = (2e-2)^2 ; % m^2
%r = (2e-4)^2 ; %

%% System definition
nA = 6;
A = zeros(nA, nA);
A(1:3, 4:6) = eye(3);
B = zeros(nA, 3);
B(4:nA,:) = eye(3);

% Discretization
S = [-A B*W*B'; zeros(nA,nA) A'];
Cc = expm(S*dt);

Ad = Cc(nA+1:end, nA+1:end)';
Wd = Ad * Cc(1:nA, nA+1:end);

% State equation
f = @(x) Ad * x;

% Measurement equation
h = @(x) [sqrt(x(1)^2 + x(2)^2 + x(3)^2); sqrt((x(1)-l2)^2 + x(2)^2 + x(3)^2); sqrt(x(1)^2 + (x(2)-l3)^2 + x(3)^2)];



%% Unscented Kalman Filter

Xhatm = zeros(nA, N);
Xhatp = zeros(nA,N);
Xhatm(:,1) = X0;
%L = zeros(3, N);
Qm = Q0;
Qxb = zeros(nA, N);

x = X0;
Qp = Q0;

for i = 1:N
    
    [x, Qp] = ukf_trilat(Ad,x,Qp,h,Y(:,i),Wd,R);
    
    Xhatp(:,i) = x;
    Qxb(:,i) = sqrt(diag(Qp));
    
   
end