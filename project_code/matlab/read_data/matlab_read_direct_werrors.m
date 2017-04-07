% This file is part of:
% Ultrasound Positioning System using the Kalman Filter
% by Enrique Fernandez (efernan@mit.edu)
% 16.322 Stochastic Estimation and Control Final Project
% Massachusetts Institute of Technology
% Fall 2013 - December 8, 2013

% This file receives the data from the Arduino and stores it as a variable.

sampling_rate = 20; %Hz
max_time = 10;
N = max_time * sampling_rate;


s = serial('/dev/tty.usbmodem1421', 'BaudRate', 9600)

disp ('Opening Arduino Serial Connection');
% Open arduino connection
fopen(s);

disp('Sending handshake to Arduino....');
% Send handshake to arduino
fwrite(s, 'S');

fprintf('Waiting for Arduino ack...');
% Wait for Arduino's answer
rcv = 'z';
while (rcv ~='s')
    rcv = fread(s, 1, 'uchar');
end
fprintf('OK!!\n');
disp('Arduino acknowledged! Serial communication established!');

i = 1;

x = zeros(3, N);
t = zeros(1, N);
errors = zeros(1, N);
%flush serial
fscanf(s);

% while i<=100
while i<=N
    % Request data to arduino
    fwrite(s, 'd');
%     str = fscanf(s, '%s');
%     disp(str)
    data = fscanf(s, '%d, %d, %d, %d');
    errors(i) = data(1);
    x(1:3, i) = data(2:end);
    if i == 1
        tic
        t(i) = 0;
    else
        t(i) = toc;
    end
%     x(1:3, i) = sscanf(str, '%d, %d, %d');
    fprintf('%d: (%d) | %d, %d, %d\n', i, errors(i), x(1,i), x(2, i), x(3, i));
    
    i = i + 1;
%     disp(str)
    %pause(0.05);
end
    
% % Read data
% fscanf('%d, %d, %d');

% Close the connection
fclose(s);

% Delete the serial port
%clear s;
%%
