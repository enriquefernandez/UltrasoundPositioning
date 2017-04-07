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
X0 = [0.5, 0.5, 1, 0, 0, 0];
Q0 = diag([ 0.04 0.04 0.01 0.005 0.005 0.005]);
%Sampling rate
dt = 0.05; % 20 Hz


%Sensor setup
% l2 = 0.575; %m
% l3 = 0.565; %m
l2 = 0.567; %m
l3 = 0.560; %m

Y = yt * 340.29e-6; % Microseconds to m.
[nh, N] = size(Y);
t = 0:dt:(N-1)*dt;
Tf = t(end);

%% Noise setup
% w = 0.0025e-2 ; % (m/s)^2
% w = 0.0025 ; % (m/s)^2
% w = 0.25 ; % (m/s)^2
% w = 5 ; % (m/s)^2
w = 0.2;
w=0.05*1;
% w = 100 ; % (m/s)^2
W = w*eye(3);
% Sensor noise
% r1 = 0.0138;
% r2 = 0.0096;
% r3 = 0.0096;

% r1 = 4e-6 * 30;
% r2 = 3.6697e-06 * 30;
% r3 = 5.3850e-06 * 30;
% Noise with median filter
r1 = 0.0042^2;
r2 = 0.0043^2;
r3 = 0.0047^2;
R = [r1 0 0;0 r2 0;0 0 r3]*1;
% r = (2e-2)^2 ; % m^2
%r = (2e-4)^2 ; %

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

inn = zeros(3, N);
S = zeros(3,3,N);
vareps=zeros(N,1);Chi2pts=zeros(N,2);

for i = 1:N
    C = dh(Xhatm(:, i));
    hx = h(Xhatm(:, i));
    
    Qp = Qm - Qm*C'*inv(C*Qm*C' + R)*C*Qm;
    L = Qp*C'/R;
    Xhatp(:,i) = Xhatm(:,i) + L*(Y(:,i)-hx);
    
    % KF analysis
    inn(:,i) = Y(:,i)-hx;
    S(:,:,i)=C*Qm*C'+R;
    vareps(i)=inn(:, i)'*inv(S(:,:,i))*inn(:,i);
    Chi2pts(i,:)=chi2inv([.025 1-.025],i)/i; % 95% test
    
    
    Qxb(:,i) = sqrt(diag(Qp));
    Xhatm(:, i+1) = Ad * Xhatp(:,i);
    Qm = Ad*Qp*Ad' + Wd;
end

% Plot KF analysis plots

% Innovations
for j=1:3
    Sd = squeeze([S(1,1,:) S(2,2,:) S(3,3,:)]);
    kk=3;figure(j)
    plot(t,inn(j,:),'.')
    hold on;plot(t,sqrt(Sd(j,:)),'g--',t,-sqrt(Sd(j,:)),'g--');hold off
    legend('innovation','\pm sqrt(Sd(j,:))','Location','NorthWest');ylabel('Innovations Process');
    xlabel('time');title(['Sensor: ', num2str(j)])
    range=max(max(inn(j,:))-min(inn(j,:)),2*max(sqrt(Sd(j,:))));
%     text(75,.8*range,['sr_{act}=',num2str(sr_act),' sr=',num2str(sr)])
%     text(75,.6*range,['sw_{act}=',num2str(sw_act),' sw=',num2str(sw)])
    axis([0 Tf -range range])
end

%form test statistic
%
varepsavg=cumsum(vareps)./[1:N]';
kk=4;figure(kk)
plot(t,vareps,'.')
hold on;plot(t,varepsavg,'k--')
plot(t,Chi2pts,'r:');hold off
legend('Normalized Innovation','Moving avg','95% confid bounds')
xlabel('time');ylabel('NES variable')
axis([0 Tf 0 2*max([max(varepsavg) max(Chi2pts)])])
title(['NES Test '])

% form time-averaged auto-correlation
testr=xcorr(inn,'unbiased');
rr=testr(N:2*N-1)'/testr(N);
%rr=zeros(N,1);
%for ii=1:N;rr(ii)=inn(1:end-ii+1)*inn(ii:end)'/(N-(ii-1));end
%rr=rr/rr(1);
kk=5;figure(kk)
ll=1:floor(N/2);
plot(ll-1,rr(ll),'b.')
hold on;plot(ll-1,2/sqrt(N)*ones(length(ll),2),'r:')
plot(ll-1,-2/sqrt(N)*ones(length(ll),2),'r:');hold off
title(['Approx Autocorrelation of Innovation '])
ylabel('Normalized Correlation')
xlabel('Correlation times')
