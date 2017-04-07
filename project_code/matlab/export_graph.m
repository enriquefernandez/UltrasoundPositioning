% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

function export_graph(name, width, height)

if nargin <2
    width = 8;
end
if nargin <3
    height = 6;
end

% For two column (or side by side), good values
% width = 8, height = 6
% For full width
% width = 16, height = 7

% Example how to adjust your figure properties for
% publication needs
s = gcf;
% Select the default font and font size
% Note: Matlab does internally round the font size
% to decimal pt values
set(s, 'DefaultTextFontSize', 10); % [pt]
set(s, 'DefaultAxesFontSize', 10); % [pt]
set(s, 'DefaultAxesFontName', 'Times New Roman');
set(s, 'DefaultTextFontName', 'Times New Roman');
set(s, 'DefaultLineMarkerSize', 2);
% Select the preferred unit like inches, centimeters,
% or pixels
set(s, 'Units', 'centimeters');
pos = get(s, 'Position');
% pos(3) = 8; % Select the width of the figure in [cm]
% pos(4) = 6; % Select the height of the figure in [cm]
pos(3) = width; % Select the width of the figure in [cm]
pos(4) = height; % Select the height of the figure in [cm]
set(s, 'Position', pos);
set(s, 'PaperType', 'a4letter');
% From SVG 1.1. Specification:
% "1pt" equals "1.25px"
% "1pc" equals "15px"
% "1mm" would be "3.543307px"
% "1cm" equals "35.43307px"
% "1in" equals "90px"

set(gcf, 'Color', 'w');
box on
export_fig(sprintf('graphs/%s', name));
end
