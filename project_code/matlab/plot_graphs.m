% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

function plot_h = plot_graphs(handles, formatStr, t, X, Q, showQ)

if nargin < 5
    showQ = 0;
    Q = 0;
end

plot_h = zeros(4, 1);

for j=1:3
    % Plot x, y ,z
%     axes(h(j))
    plot_h(j) = plot(handles(j), t, X(j,:), formatStr);
    if showQ
        plot(handles(j), t, X(j,:) + 2*Q(j,:), formatStr, 'LineStyle', '--');
        plot(handles(j), t, X(j,:) - 2*Q(j,:), formatStr, 'LineStyle', '--');
    end
end

% Plot xy
% axes(h(4));
plot_h(4) = plot(handles(4), X(1,:), X(2,:), formatStr);

end
