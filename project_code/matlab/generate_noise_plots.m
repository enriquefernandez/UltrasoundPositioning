% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

%% Setup
load sensor_noise.mat

%% Sensor noise spikes (raw vs median)

plot_noise(desc_zigzag,1)
set(gcf, 'Color', 'w');
export_graph 'sensor_raw_median.pdf'

%% Filtered sensor noise
figure
var = top_s3;
j = 2;
dt = 1/var.sampling_rate;
N = size(var.x,2);
t = 0:dt:dt*N;
t = t(1:N);
% h1 = plot(t, var.x(j,:),'b');
hold on;
h2 = plot(t, medfilt1( var.x(j,:)'), 'r');
% plot(t, var.x(j,:), 'b');

% legend([h1 h2], 'Raw Sensor', 'Median');
xlabel('time (s)')
ylabel('Sensor value');
title('Median Filtered Sensor Values')
export_graph 'sensor_filtered_median.pdf'

%% Fixed points displated in map
figure
hold on;
title('Map of fixed points');
xlabel('x');
ylabel('y');
axis equal
xlim([-0.25, 0.85]);
ylim([-0.25, 0.85]);

grid on
rec_s = 0.04;
hold on;
rectangle('Position', [-.5*rec_s, -.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [l2-.5*rec_s, -.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [-.5*rec_s, l3-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rec_large = 0.618;
rectangle('Position', [0, 0, rec_large, rec_large], 'EdgeColor', 'black')

fixed_point_vars = {fixed_aroundA, fixed_aroundB, top_s2, top_s3, top_s1};
label_points = [0.5 0.25; 0.4 0.5; 0.65 -0.05; -0.05 0.7; -0.1 -0.1];
label_str = {'A', 'B', 'S2', 'S3', 'S1'};

n = length(data_vars);

noise_matrix = zeros(n+1, 3);

for j=1:n
    seq_ts = data_vars{j}.x;
    [~, N] = size(seq_ts);
    t = 0:dt:(N-1)*dt;
    Xs = simple_estimate(seq_ts);
    plot( Xs(1,:), Xs(2,:), 'r');
    text(label_points(j,1), label_points(j,2), label_str{j})
end
export_graph 'fixed_point_map.pdf'