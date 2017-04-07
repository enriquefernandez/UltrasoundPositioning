% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

function [Xhatp, Qxb] = trilateration3d_UKF(dt, W, R, yt, X0, Q0)

%% Configuration
%Initial estimate
% X0 = [0.25, 0.36, 1, 0, 0, 0];
if nargin < 5
%     X0 = [0.5, 0.5, 1, 0, 0, 0];
      X0 = [1, 1, 1, 0, 0, 0]';
end
if nargin < 6
%     Q0 = diag([ 0.04 0.04 0.01 0.005 0.005 0.005]);
    Q0 = diag([ 0.5 0.5 0.01 0.005 0.005 0.005]) * 1000;
end
%Sampling rate
% dt = 0.25; % Initially 4 Hz

%Sensor setup
% l2 = 0.575; %m
% l3 = 0.565; %m
l2 = 0.567; %m
l3 = 0.560; %m

speed_sound = 340.29;
% speed_sound = 343.216;

% Y = yt * 340.29e-6; % Microseconds to m.
Y = yt * speed_sound * 1e-6; % Microseconds to m.
[~, N] = size(Y);

% %% Noise setup
% % w = 0.0025 ; % (m/s)^2
% w = 0.25 ; % (m/s)^2
% W = w*eye(3);
% % Sensor noise
% % 0.5*(std(t1_long*340.29e-6) + std(t1_short*340.29e-6))
% r1 = 4e-6 * 30;
% r2 = 3.6697e-06 * 30;
% r3 = 5.3850e-06 * 30;
% R = [r1 0 0;0 r2 0;0 0 r3];
% % r = (2e-2)^2 ; % m^2
% %r = (2e-4)^2 ; %

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
% f = @(x) Ad * x;

% Measurement equation
h = @(x) [sqrt(x(1)^2 + x(2)^2 + x(3)^2); sqrt((x(1)-l2)^2 + x(2)^2 + x(3)^2); sqrt(x(1)^2 + (x(2)-l3)^2 + x(3)^2)];


%% Unscented Kalman Filter

Xhatp = zeros(nA,N);

Qxb = zeros(nA, N);

x = X0;
Qp = Q0;

for i = 1:N
    
    [x, Qp] = ukf_trilat(Ad,x,Qp,h,Y(:,i),Wd,R);
    
    Xhatp(:,i) = x;
    Qxb(:,i) = sqrt(diag(Qp));
    
   
end

end