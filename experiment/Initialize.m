clc;clear
addpath(genpath('../my_function'));
dt = 1/25;% sampling frequency: 25Hz
K = 5/dt; %measure times
time = dt*(0:K-1);
% mhe gain
% gain = [1 1 1 1];
gain = [10 1 100 0];
% imu_noise = sqrt(0.1)*randn(3,K);
imu_noise = 0*randn(3,K);
