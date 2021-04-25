%% 1. clear workspace
clc;close all
clearvars -except dt K time gain imu_noise

%% 2. load data & preprocess
name = 'screw';
filename = ['./data/0411/' name '_1.mat'];
[gtd,gtd_o,imu,imu_o,y,vy] = preprocess(filename,dt,K,imu_noise);
% %% 3. plot before mhe
% plot_before_mhe(time,gtd,imu,imu_o,y,vy,dt,K) % run the script
% %% 4.1 MHE
% clc
% xt = MHE(gtd,imu,y,vy,dt,K-1,gain);
% %plotres
% figure(5)
% [~]  = plot_result(time,xt,gtd,name);
% 
% %% 4.3 LSR to estimate x
% clc;
% x_LSR = [];
% num = 15;
% for i = 1:K-num
% %     temp = estimate_LSR(imu(:,i:i+num-1),y(:,i:i+num),dt);
%     temp = estimate_NLS(imu(:,i:i+num-1),y(:,i:i+num),vy(:,i:i+num),dt);
%     x_LSR(:,i) = temp(1:6);
% end
% figure(6)
% [~] = plot_result(time(:,1:size(x_LSR,2)),x_LSR,gtd(:,1:size(x_LSR,2)),'screw\_exp');

%% 4.3 Observer
% x_Observer = Observer(imu(:,1/dt:end),y(:,1/dt:end),gtd(:,1/dt),dt);
% figure(6)
% [~]  = plot_result(time(1/dt:end),x_Observer(:,1:end-1),gtd(:,1/dt:end),'screw');
x_Observer = Observer(imu,y,gtd(:,1),dt);
figure(6)
[~]  = plot_result(time,x_Observer(:,1:end-1),gtd,'screw');
