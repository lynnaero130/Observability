clc;clear
addpath(genpath('../my_function')) % add the path
dt = 1/25;  % sampling time; 25Hz
K = 250/dt; % measure times
sigma_omega =diag([0.001,0.001,0.001])*0; % imu noise
sigma_v = 0.001; % observation noise

gain = [10 10 100 0]; % mhe gain
global imu_noise uwb_noise
% imu_noise = sqrt(sigma_omega)*randn(3,K)+0.05*rand(3,K);
imu_noise = sqrt(sigma_omega)*randn(3,K)+(sigma_omega)*abs(randn(3,K));
uwb_noise = sqrt(sigma_v)*randn(1,K+1);  % used to simulate measured uwb