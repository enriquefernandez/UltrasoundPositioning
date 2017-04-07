% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

function plot_loc_data(seq_ts, filters,showQ , X0, Q0)

if nargin < 5
%     X0 = [0.5, 0.5, 1, 0, 0, 0];
      X0 = [1, 1, 1, 0, 0, 0]';
end
if nargin < 6
%     Q0 = diag([ 0.04 0.04 0.01 0.005 0.005 0.005]);
    Q0 = diag([ 0.5 0.5 0.01 0.005 0.005 0.005]) * 1000;
end

%% Configuration
%Sampling rate
% dt = 0.25; % Initially 4 Hz
% dt = 0.1; % 10 Hz
dt = 0.05; % 20 Hz

% Noise
% w = 0.0025 ; % (m/s)^2
% w = 0.25 ; % (m/s)^2
% w = 0.5;

w = 0.05;
% w = 0.0045;

% w = 0.01e-2;
% w = 0.5 *100; % (m/s)^2
W = w*eye(3);
% Sensor noise
% 0.5*(std(t1_long*340.29e-6) + std(t1_short*340.29e-6))
% r1 = 4e-6 * 3;
% r2 = 3.6697e-06 * 3;
% r3 = 5.3850e-06 * 3;
%  Noise from top_s2 data (assuming emitter fixed)
% r1 = 0.0138;
% r2 = 0.0096;
% r3 = 0.0096;

% Noise with median filter
r1 = 0.0042^2;
r2 = 0.0043^2;
r3 = 0.0047^2;

% Noise without median filter
% r1 = 0.0052^2;
% r2 = 0.0056^2;
% r3 = 0.0065^2;

R = [r1 0 0;0 r2 0;0 0 r3]*1;
% r = (2e-2)^2 ; % m^2
%r = (2e-4)^2 ; %

%Sensor setup
% l2 = 0.575; %m
% l3 = 0.565; %m
l2 = 0.567; %m
l3 = 0.560; %m

%% Function arguments
narginchk(2,5);

plotSimple = bitand(filters, 1);
plotEKF = bitand(filters, 2);
plotUKF = bitand(filters, 4);

if (plotSimple + plotEKF + plotUKF == 0)
    error('Need to plot at least one filter (simple, EKF, UKF)');
end

%% Setup axes and figure

figure

hx = subplot(3,2,2);
hold on;
title('x');
ylabel('x');
ylim([-0.5, 1.2]);
grid on

hy = subplot(3,2,4);
hold on;
title('y');
ylabel('y');
ylim([-0.5, 1.2]);
grid on

hz = subplot(3,2,6);
hold on;
title('z');
ylabel('z');
xlabel('t');
ylim([0, 1.8]);
grid on

hxy = subplot(3,2,[1 3 ]);
hold on;
title('XY');
xlabel('x');
ylabel('y');
axis equal
xlim([-0.25, 0.85]);
ylim([-0.25, 0.85]);
grid on

%% Plot sensors and box
axes(hxy);
rec_s = 0.04;
rectangle('Position', [-.5*rec_s, -.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [l2-.5*rec_s, -.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [-.5*rec_s, l3-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')

rec_large = 0.618;
rectangle('Position', [0, 0, rec_large, rec_large], 'EdgeColor', 'black')

%% Get data and plot

[~, N] = size(seq_ts);
t = 0:dt:(N-1)*dt;
H = [hx, hy, hz, hxy];

if plotSimple
    Xs = simple_estimate(seq_ts);
    hs = plot_graphs(H,'r', t, Xs);
    %legend(hs, 'Simple Estimate');
end

if plotEKF
    [Xekf, Qekf] = trilateration3d_EKF(dt, W, R, seq_ts, X0, Q0);
    hekf = plot_graphs(H,'b', t, Xekf, Qekf, showQ);
    %legend(hekf, 'EKF');
end

if plotUKF
    [Xukf, Qukf] = trilateration3d_UKF(dt, W, R, seq_ts, X0, Q0);
    hekf = plot_graphs(H,'g', t, Xukf, Qukf, showQ);
end

% Display legends

%% Covariances plot
% figure
% colors = ['r','g','b'];
% for j=1:3
%     
%     hold on
%     plot(t, Qekf(j,:), colors(j));
%     plot(t, Qukf(j,:), colors(j), 'LineStyle', '--');
% end


end
