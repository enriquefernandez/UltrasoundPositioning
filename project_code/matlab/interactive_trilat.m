% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

function interactive_trilat()

%% Sensor setup
a = 3; b = 3;
x1 = 0; y1 = 0; %S1
x2 = a; y2 = 0; %S2
x3 = 0; y3 = b; %S3

%% Noise setup
w = 0.0025 ; % (m/s)^2
w = 0.5
W = w*eye(2);
r = (1e-2)^2 ; % m^2
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
dt = 0.02;
S = [-A B*W*B'; zeros(4,4) A'];
Cc = expm(S*dt);

Ad = Cc(5:end, 5:end)';
Wd = Ad * Cc(1:4, 5:end); 


%% Simulation
tf = 5;
N = round(tf/dt);
t = [0:dt:tf];
t = t(1:N);
X0 = [1; 1; 1; 0.05]; % Moving to the right
Q0 = diag([0.1 0.05 0.05 0.01]);

Xact = zeros(2, N);
Y = zeros(3, N);


%% Extended Kalman Filter

xhatm = 0;

%Xhatm = zeros(nA, N);
%Xhatp = zeros(nA,N);
%Xhatm(:,1) = X0;
Qm = Q0;

Xhatp = [];

    function [xhatp, xhatm_out, Qm_out] = updateEKF(xhatm_in, Qm_in, y)
        disp('Updating EKF')
        C = dh(xhatm_in);
        hx = h(xhatm_in);

        Qp = Qm_in - Qm_in*C'*inv(C*Qm_in*C' + R)*C*Qm_in;
        L = Qp*C'*inv(R);
        %Xhatp(:,i) = xhatm + L*(y-hx);

        %Qxb(:,i) = sqrt(diag(Qp));
        %Xhatm(:, i+1) = Ad * Xhatp(:,i);
        Qm_out = Ad*Qp*Ad' + Wd;
        xhatp = xhatm_in + L*(y-hx);
        xhatm_out = Ad * xhatp;
     end


%% Capture functions

figure;
axis equal
% Plot sensors
hold on
rec_s = 0.15;
rectangle('Position', [x1-.5*rec_s, y1-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'blue')
rectangle('Position', [x2-.5*rec_s, y2-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'blue')
rectangle('Position', [x3-.5*rec_s, y3-.5*rec_s, rec_s, rec_s],'Curvature',[1,1], 'FaceColor', 'blue')

hold on
xlim([0 5])
ylim([0 5])

set (gcf, 'WindowButtonMotionFcn', @mouseMoveCallback);
% set (gcf, 'WindowButtonDownFcn', @mouseDownCallback);
% set (gcf, 'WindowButtonUpFcn', @mouseUpCallback);
set (gcf, 'KeyPressFcn', @keyDown);
%set (gcf, 'KeyReleaseFcn', @keyUp);
capture = 0
i = 1
currentPointX = 0;
currentPointY = 0;

hXact = plot(nan,'b');
hXEKF = plot(nan,'-or');
hSimple = plot(nan,'g');
simpleEst = [];

timerObj = timer('TimerFcn',@timerCallback,'Period',dt,'ExecutionMode','fixedRate');

    function timerCallback(timerObj,event,str_arg)
        %disp('CallbackT');
        if capture == 1
            %C = get (gca, 'CurrentPoint');
            %title(gca, ['(X,Y) = (', num2str(C(1,1)), ', ',num2str(C(1,2)), ')']);            
            %point = C(1,1:2)';
            point = [currentPointX; currentPointY];
%             timeCapture = [timeCapture; recordedTime];
            Xact(:,i) = point;
    %         tic;
            %plot(point(1),point(2),'dr');
            set(hXact, 'XData', Xact(1,:));
            set(hXact, 'YData', Xact(2,:));
            
            % Measurement
            %Y(:, i) = h(point);
            
            state_point = [point(1); 0; point(2); 0];
            y = h(state_point) + sqrtm(R)*randn(3,1);
            
            % EKF Update
            [xhatp, xhatm, Qm] = updateEKF(xhatm, Qm, y);
            
            Xhatp = [Xhatp xhatp];
            set(hXEKF, 'XData', Xhatp(1,:));
            set(hXEKF, 'YData', Xhatp(3,:));
            
            % Simple estimation
            xs = (y(1)^2 - y(2)^2 + a^2)/(2*a);
            ys = real(sqrt(y(1)^2 - xs^2));
            simpleEst = [simpleEst [xs ys]']
            set(hSimple, 'XData', simpleEst(1,:));
            set(hSimple, 'YData', simpleEst(2,:));
            
            i = i + 1;
            if i> N
                capture = 0;
                stop(timerObj);
                Xact
            end
        end
       
        
    end
    function keyDown(object, eventdata)
%         disp('down');
        if capture
            capture = 0;
            stop(timerObj);
        else
            capture = 1;
            % Setup EKF
            xhatm = [currentPointX 0 currentPointY 0]';
            Qm = Q0;
            disp('EKF ready');
            start(timerObj);
            disp('Capturing!!');
            title(gca,'Capturing!');
        end
        
       
    end
    function mouseMoveCallback(object, eventdata)
        [currentPointX, currentPointY] = gpos(gca);
        %disp(['currentPointX: ' num2str(currentPointX) ', currentPointY: ',num2str(currentPointY)]);
    end
end