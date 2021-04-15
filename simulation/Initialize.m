clc;clear
addpath(genpath('../my_function')) % add the path
x0 = [-0.5;-0.5;0.5;0;0;0]; % use prediction as initial guess
% x0 = [0;0;0;0;0;0]; % use prediction as initial guess
xg = [0.5;1;1.2;0;0;0]; % end point
rg = 0.01; % position tolerance
ru = 2; % input constraint
dt = 1/50;
K = 2/dt; %measure times
t = dt*(0:K);
x_initial_guess = [ones(3,K) ones(3,K) zeros(3,K)];
sigma_omega =diag([0.001,0.001,0.001])*100; %diag([0,0,0]); % imu noise
sigma_v = 0.01;%0; % observation noise
[b2,a2] = butter(2,5*dt,'low'); 
% mhe gain
gain = [10 1 100 0];