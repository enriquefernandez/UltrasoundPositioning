% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

%% Sensor setup
a = 3; b = 3;
x1 = 0; y1 = 0; %S1
x2 = a; y2 = 0; %S2
x3 = 0; y3 = b; %S3

%% Noise setup
w = 0.0025 ; % (m/s)^2
W = w*eye(2);
r = (2e-2)^2 ; % m^2
%r = (2e-4)^2 ; % m^2
R = r * eye(3);

%% System definition
nA = 4;
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0 0; 1 0; 0 0; 0 1];

h = @(x) [sqrt((x(1)-x1)^2+(x(3)-y1)^2); sqrt((x(1)-x2)^2+(x(3)-y2)^2); sqrt((x(1)-x3)^2+(x(3)-y3)^2)];
dh = @(x) [(x(1)-x1)/sqrt((x(1)-x1)^2 + (x(3)-y1)^2) 0 (x(3)-y1)/sqrt((x(1)-x1)^2 + (x(3)-y1)^2) 0;
           (x(1)-x2)/sqrt((x(1)-x2)^2 + (x(3)-y2)^2) 0 (x(3)-y2)/sqrt((x(1)-x2)^2 + (x(3)-y2)^2) 0;
           (x(1)-x3)/sqrt((x(1)-x3)^2 + (x(3)-y3)^2) 0 (x(3)-y3)/sqrt((x(1)-x3)^2 + (x(3)-y3)^2) 0];
% Discretization
dt = 0.05;
S = [-A B*W*B'; zeros(4,4) A'];
Cc = expm(S*dt);

Ad = Cc(5:end, 5:end)';
Wd = Ad * Cc(1:4, 5:end); 

%% Simulation

tf = 10;
N = round(tf/dt);
t = [0:dt:tf];
t = t(1:N);
X0 = [1; 1; 1; 0.05]; % Moving to the right
Q0 = diag([0.1 0.05 0.05 0.01]);

Xact = zeros(nA, N);
Xact(:, 1) = X0 + sqrtm(Q0)*randn(nA,1);
%Xact(:, 1) = X0;

y = zeros(3, N);

for i=1:N
    y(:, i) = h(Xact(:,i));
    Xact(:, i+1) = Ad*Xact(:, i);
end
Y = y + sqrtm(R)*randn(size(y));
%Y = y;

%% Simple estimate
xs = (Y(1,:).^2 - Y(2,:).^2 + a^2)/(2*a);
ys = real(sqrt(Y(1,:).^2 - xs.^2));

%% Plot comparison
figure
title('Simple Estimate')
axis equal
% Plot sensors
hold on
rec_s = 0.15;
rectangle('Position', [x1-.5*rec_s, y1-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'blue')
rectangle('Position', [x2-.5*rec_s, y2-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'blue')
rectangle('Position', [x3-.5*rec_s, y3-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'blue')

plot(Xact(1,:), Xact(3,:),'-b')
hold on
xlim([0 5])
ylim([0 5])
plot(xs, ys, '--og')

% Comparison plot
figure

subplot(2,1,1);
plot(t, Xact(1,1:N),'-ob')
hold on
plot(t, xs(1:N),'-xg')
subplot(2,1,2);
plot(t, Xact(3,1:N),'-ob')
hold on
plot(t, ys(1:N),'-xg')
title('Simple Estimate Error')

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
    L = Qp*C'*inv(R);
    Xhatp(:,i) = Xhatm(:,i) + L*(Y(:,i)-hx);
    
    Qxb(:,i) = sqrt(diag(Qp));
    Xhatm(:, i+1) = Ad * Xhatp(:,i);
    Qm = Ad*Qp*Ad' + Wd;
end

%% Plot EKF vs real data
figure
title('EKF Estimate')
axis equal
% Plot sensors
hold on
rec_s = 0.15;
rectangle('Position', [x1-.5*rec_s, y1-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'blue')
rectangle('Position', [x2-.5*rec_s, y2-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'blue')
rectangle('Position', [x3-.5*rec_s, y3-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'blue')

plot(Xact(1,:), Xact(3,:),'-b')
hold on
xlim([0 5])
ylim([0 5])
plot(Xhatp(1,:), Xhatp(3,:), '--or')

% Comparison plot
figure

subplot(2,1,1);
plot(t, Xact(1,1:N),'-ob')
hold on
plot(t, Xhatp(1,1:N),'-xr')
subplot(2,1,2);
plot(t, Xact(3,1:N),'-ob')
hold on
plot(t, Xhatp(3,1:N),'-xr')
title('EKF Error')

% Error Plot (EKF vs Simple Estimate)
figure

subplot(2,1,1);
plot(t, Xhatp(1,1:N)-Xact(1,1:N),'-or')
hold on
plot(t, Qxb(1,1:N),'--r')
plot(t, -Qxb(1,1:N),'--r')

plot(t, xs(1:N)-Xact(1,1:N),'-og')

subplot(2,1,2);
plot(t, Xhatp(3,1:N)-Xact(3,1:N),'-or')
hold on
plot(t, Qxb(3,1:N),'--r')
plot(t, -Qxb(3,1:N),'--r')

plot(t, ys(1:N)-Xact(3,1:N),'-og')
title('Error Plot')

