% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

%% Setup
load 'results'


%% Map of fixed points

setup_vars;

figure
hold on;
title('Map of fixed points');
xlabel('x');
ylabel('y');
axis equal
xlim([-0.25, 0.85]);
ylim([-0.25, 0.85]);

grid on
hold on;
rectangle('Position', [-.5*rec_s, -.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [l2-.5*rec_s, -.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [-.5*rec_s, l3-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [0, 0, rec_large, rec_large], 'EdgeColor', 'black')

fixed_point_vars = {fixed_aroundA, fixed_aroundB, top_s2, top_s3, top_s1};
label_points = [0.45 0.20; 0.4 0.5; 0.65 -0.05; -0.05 0.7; -0.1 -0.1];
real_points = [0.540 0.267; 0.393 0.553; 0.567 -0.00; -0.00 0.560; -0.0 -0.0];
label_str = {'A', 'B', 'S2', 'S3', 'S1'};

n = length(fixed_point_vars);

error_dist = zeros(n,1);
for j=1:n
    Q0 = diag([0.05, 0.05, 0.05, 0.01,0.01,0.01]).^2;
    X0 = [real_points(j,:)' ;1.1;0;0;0];
    seq_ts = medfilt1(fixed_point_vars{j}.x')';
    [~, N] = size(seq_ts);
    t = 0:dt:(N-1)*dt;
    [Xukf, Qukf] = trilateration3d_UKF(dt, W, R, seq_ts, X0, Q0);
    % Calculate estimate from last third
    Xukf = Xukf(1:2,N-floor(N/3):N);
    estimated = mean(Xukf');
    % Plot
    plot(Xukf(1,:), Xukf(2,:),'g');
    h1=plot(estimated(1), estimated(2),'r*');
    h2=plot(real_points(j,1), real_points(j,2), '+b');
    text(label_points(j,1), label_points(j,2), label_str{j})
    error_dist(j) = norm(estimated - real_points(j,:));
    
    fprintf('%.3f & %.3f & %.3f & %.3f & %.3f\n', real_points(j,1), real_points(j,2), estimated(1), estimated(2), error_dist(j));
end
% legend([h1 h2],'Estimated', 'Real')
% legendflex([h1 h2],{'Estimated', 'Real'}, 'ref', gca, 'anchor', {'ne', 'ne'})
error_dist
export_graph 'map_fixed_points_error.pdf'

%% Height error
figure
hold on
ylim([0,1.4])
xlabel('t (s)')
ylabel('height (m)')
fixed_point_vars = {fixed_far_corner};
real_height = [1.21];

n = length(fixed_point_vars);

noise_matrix = zeros(n+1, 3);

error_height = zeros(n,1);
for j=1:n
    Q0 = diag([0.05, 0.05, 0.05, 0.01,0.01,0.01]).^2;
    X0 = [0.618; 0.618 ;1.2;0;0;0];
    seq_ts = medfilt1(fixed_point_vars{j}.x')';
    [~, N] = size(seq_ts);
    t = 0:dt:(N-1)*dt;
    [Xukf, Qukf] = trilateration3d_UKF(dt, W, R, seq_ts, X0, Q0);
    % Calculate estimate from last third
    Xukf = Xukf(3,N-floor(N/3):N);
    t = t(N-floor(N/3):N);
    estimated = mean(Xukf);
    N = numel(t);
    % Plot
    h3 = plot(t, Xukf,'g');
    h1=plot(t, ones(N,1)*estimated,'r--');
    h2=plot(t, ones(N,1)*real_height, 'b-.');
%     text(label_points(j,1), label_points(j,2), label_str{j})
    error_height(j) = abs(estimated - real_height);
    xlim([t(1), t(end)])
    grid on
end
ylim([1 1.3])
title('Height Estimation Error');
legend([h3 h1 h2],'UKF Estimate', 'UKF Average Estimate', 'Real Height', 'Location', 'South')
export_graph('height_error.pdf')
error_height


%% Zigzag map plot
setup_vars;
figure
hold on;
title('Trajectory in the xy plane');
xlabel('x (m)');
ylabel('y (m)');
axis equal
xlim([-0.1, 0.7]);
ylim([-0.1, 0.7]);

grid on
hold on;
rectangle('Position', [-.5*rec_s, -.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [l2-.5*rec_s, -.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [-.5*rec_s, l3-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [0, 0, rec_large, rec_large], 'EdgeColor', 'black')
data = medfilt1(desc_zigzag.x(:,10:end)')';

N = size(data,2);
t = 0:dt:dt*N;
t = t(1:N);

% Initial conditions
X0 = [0.3, 0.3, 1, 0, 0, 0]';
Q0 = diag([ 0.5 0.5 0.2 0.2 0.2 0.2])
% Estimate position
Xs = simple_estimate(data);
[Xekf, Qekf] = trilateration3d_EKF(dt, W, R, data, X0, Q0);
[Xukf, Qukf] = trilateration3d_UKF(dt, W, R, data, X0, Q0);

hs = plot(Xs(1,:),Xs(2,:),'k*--', 'MarkerSize',5, 'LineWidth',0.5);
hekf = plot(Xekf(1,:),Xekf(2,:),'b', 'LineWidth',1);
hukf = plot(Xukf(1,:),Xukf(2,:),'r', 'LineWidth',1);

legend([hs hekf hukf], 'Simple Estimate', 'EKF', 'UKF');
export_graph('zigzag_xy_map.pdf',12,12)

%% Zigzag xyz positions
setup_vars;

var_names = {'x','y','z'};

for j=1:3 
    %x
    figure
    hold on;
    title(sprintf('Estimate of %s',var_names{j}))
    % hs = plot(t, Xs(1,:),'k*--', 'MarkerSize',5, 'LineWidth',0.5);
    hs = plot(t, Xs(j,:),'k.','MarkerSize',5);
    hekf = plot(t, Xekf(j,:),'b', 'LineWidth',0.5);
    hukf = plot(t, Xukf(j,:),'r', 'LineWidth',0.5);
%     plot(t, Xekf(j,:)+2*Qekf(j,:),'b--', 'LineWidth',0.5);
%     plot(t, Xekf(j,:)-2*Qekf(j,:),'b--', 'LineWidth',0.5);
%     plot(t, Xukf(j,:)+2*Qukf(j,:),'r--', 'LineWidth',0.5);
%     plot(t, Xukf(j,:)-2*Qukf(j,:),'r--', 'LineWidth',0.5);
    
    xlabel('t (s)');
    ylabel(sprintf('%s (m)',var_names{j}));
    %ylim([-0.07, 0.65]);
    ylim([min(Xs(j,:)), max(Xs(j,:))]);
    grid on
    % legend([hs hekf hukf], 'Simple Estimate', 'EKF', 'UKF');
    export_graph(sprintf('zigzag_%s.pdf',var_names{j}),8,4)

    
    %x zoom
    figure
    hold on;
    title(sprintf('Estimate of %s (zoomed)',var_names{j}))
    % hs = plot(t, Xs(1,:),'k*--', 'MarkerSize',5, 'LineWidth',0.5);
    hs = plot(t(98:128), Xs(j,98:128),'k.','MarkerSize',7);
    hekf = plot(t(98:128), Xekf(j,98:128),'b', 'LineWidth',0.5);
    hukf = plot(t(98:128), Xukf(j,98:128),'r', 'LineWidth',0.5);
    plot(t(98:128), Xekf(j,98:128)+2*Qekf(j,98:128),'b--', 'LineWidth',0.5);
    plot(t(98:128), Xekf(j,98:128)-2*Qekf(j,98:128),'b--', 'LineWidth',0.5);
    plot(t(98:128), Xukf(j,98:128)+2*Qukf(j,98:128),'r--', 'LineWidth',0.5);
    plot(t(98:128), Xukf(j,98:128)-2*Qukf(j,98:128),'r--', 'LineWidth',0.5);
    xlabel('t (s)');
    ylabel(sprintf('%s (m)',var_names{j}));
    xlim([t(98),t(128)]);
    ylim([min(Xs(j,98:128)), max(Xs(j,98:128))]);
    % ylim([-0.07, 0.65]);
    grid on
    % legend([hs hekf hukf], 'Simple Estimate', 'EKF', 'UKF');   
    export_graph(sprintf('zigzag_%s_zoom.pdf',var_names{j}),8,4)
end

%% EKF vs UKF plot
figure;
hold on;
grid on;
title('Estimate of x');
plot_range = 1:25;
plot(t(plot_range), Xs(1,plot_range),'k.--','MarkerSize',5);
hekf = plot(t(plot_range), Xekf(1,plot_range),'b', 'LineWidth',0.5);
hukf = plot(t(plot_range), Xukf(1,plot_range),'r', 'LineWidth',0.5);
plot(t(plot_range), Xekf(1,plot_range)+2*Qekf(1,plot_range),'b--', 'LineWidth',0.5);
plot(t(plot_range), Xekf(1,plot_range)-2*Qekf(1,plot_range),'b--', 'LineWidth',0.5);
plot(t(plot_range), Xukf(1,plot_range)+2*Qukf(1,plot_range),'r--', 'LineWidth',0.5);
plot(t(plot_range), Xukf(1,plot_range)-2*Qukf(1,plot_range),'r--', 'LineWidth',0.5);
xlabel('t (s)');
xlim([0,t(25)]);
ylim([0.45, 0.68]);
ylabel('x (m)');
export_graph('ekf_vs_ukf.pdf',8,4)

%% Height vs xy variance

figure;
colors = {'r','g','b'};
plot_range = 1:120;
hold on
grid on
title('Q_{xx}, Q_{yy} and Q_{zz} for EKF and UKF');
xlabel('t (s)');
ylabel('cov (mm^2)');
h = [];
for j=1:3
    h(j) = plot(t(plot_range), Qekf(j,plot_range).^2*1e4,colors{j}, 'LineWidth',0.3);
    plot(t(plot_range), Qukf(j,plot_range).^2*1e4,colors{j}, 'LineStyle', '--', 'LineWidth',0.5);
end
ylim([0, 3]);
xlim([0,t(120)]);
legend(h, 'Q_{xx}', 'Q_{yy}', 'Q_{zz}', 'Orientation', 'Horizontal')
export_graph('xyz_cov.pdf',8,4)

%% Converge to wrong estimate plot
setup_vars;
figure
hold on;
title('Trajectory in the xy plane');
xlabel('x (m)');
ylabel('y (m)');
axis equal
xlim([-0.1, 0.7]);
ylim([-0.1, 0.7]);

grid on
hold on;
rectangle('Position', [-.5*rec_s, -.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [l2-.5*rec_s, -.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [-.5*rec_s, l3-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'green')
rectangle('Position', [0, 0, rec_large, rec_large], 'EdgeColor', 'black')
data = medfilt1(desc_zigzag.x(:,10:end)')';

N = size(data,2);
t = 0:dt:dt*N;
t = t(1:N);

% Initial conditions
X0 = [0.3, 0.3, -1, 0, 0, 0]';
Q0 = diag([ 0.5 0.5 0.2 0.2 0.2 0.2])
% Estimate position
Xs = simple_estimate(data);
[Xekf, Qekf] = trilateration3d_EKF(dt, W, R, data, X0, Q0);
[Xukf, Qukf] = trilateration3d_UKF(dt, W, R, data, X0, Q0);

hs = plot(Xs(1,:),Xs(2,:),'k*--', 'MarkerSize',5, 'LineWidth',0.5);
hekf = plot(Xekf(1,:),Xekf(2,:),'b', 'LineWidth',1);
hukf = plot(Xukf(1,:),Xukf(2,:),'r', 'LineWidth',1);

legend([hs hekf hukf], 'Simple Estimate', 'EKF', 'UKF');
export_graph('wrong_est_xy.pdf',8,8)

figure
hold on;
title('Wrong z estimate')
% hs = plot(t, Xs(1,:),'k*--', 'MarkerSize',5, 'LineWidth',0.5);
hs = plot(t, Xs(3,:),'k.','MarkerSize',3);
hekf = plot(t, Xekf(3,:),'b', 'LineWidth',0.5);
hukf = plot(t, Xukf(3,:),'r', 'LineWidth',0.5);
plot(t, zeros(N,1),'--k');
export_graph('wrong_est_z.pdf',8,6)