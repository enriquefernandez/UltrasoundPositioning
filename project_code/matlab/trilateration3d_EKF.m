% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

function [Xhatp, Qxb] = trilateration3d_EKF(dt, W, R, yt, X0, Q0)

%% Configuration
%Initial estimate
% X0 = [0.25, 0.36, 1, 0, 0, 0];
if nargin < 5
%     X0 = [0.5, 0.5, 1, 0, 0, 0];
      X0 = [1, 1, 1, 0, 0, 0];
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

Y = yt * 340.29e-6; % Microseconds to m.
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

h = @(x) [sqrt(x(1)^2 + x(2)^2 + x(3)^2); sqrt((x(1)-l2)^2 + x(2)^2 + x(3)^2); sqrt(x(1)^2 + (x(2)-l3)^2 + x(3)^2)];
dh = @(x)  [x(1)/sqrt(x(1)^2 + x(2)^2 + x(3)^2) x(2)/sqrt(x(1)^2 + x(2)^2 + x(3)^2) x(3)/sqrt(x(1)^2 + x(2)^2 + x(3)^2) 0 0 0;
            (x(1)-l2)/sqrt((x(1)-l2)^2 + x(2)^2 + x(3)^2) x(2)/sqrt((x(1)-l2)^2 + x(2)^2 + x(3)^2) x(3)/sqrt((x(1)-l2)^2 + x(2)^2 + x(3)^2) 0 0 0;
            x(1)/sqrt(x(1)^2 + (x(2)-l3)^2 + x(3)^2) (x(2)-l3)/sqrt(x(1)^2 + (x(2)-l3)^2 + x(3)^2) x(3)/sqrt(x(1)^2 + (x(2)-l3)^2 + x(3)^2) 0 0 0];


% Discretization
S = [-A B*W*B'; zeros(nA,nA) A'];
Cc = expm(S*dt);

Ad = Cc(nA+1:end, nA+1:end)';
Wd = Ad * Cc(1:nA, nA+1:end);


%% Extended Kalman Filter

Xhatm = zeros(nA, N);
Xhatp = zeros(nA,N);
Xhatm(:,1) = X0;
%L = zeros(3, N);
Qm = Q0;
Qxb = zeros(nA, N);

for i = 1:N
    C = dh(Xhatm(:, i));
    hx = h(Xhatm(:, i));
    
    Qp = Qm - Qm*C'*inv(C*Qm*C' + R)*C*Qm;
    L = Qp*C'/R;
    Xhatp(:,i) = Xhatm(:,i) + L*(Y(:,i)-hx);
    
    Qxb(:,i) = sqrt(diag(Qp));
    Xhatm(:, i+1) = Ad * Xhatp(:,i);
    Qm = Ad*Qp*Ad' + Wd;
end

end