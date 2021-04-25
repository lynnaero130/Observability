%% 1. clear workspace
clc;close all
clearvars -except dt K time gain imu_noise

%% 2. load data & preprocess
name = 'line';
filename = ['./data/0411/' name '_1.mat'];
[gtd,gtd_o,imu,imu_o,y,vy] = preprocess(filename,dt,K,imu_noise);

% line_d = load('../simulation/data/line.csv');
% for a=1:K
%     gtd(:,a) = gtd_o(:,a*4-3);
% end
% plot(time,gtd(1,:)-line_d(2:end,1)',time,gtd(2,:)-line_d(2:end,2)',time,gtd(3,:)-line_d(2:end,3)')
% legend('x','y','z')
% plot3
% %% 3. plot before mhe
% plot_before_mhe(time,gtd,imu,imu_o,y,vy,dt,K) % run the script
% %% 4.1 MHE
% clc
% xt_OG = MHE(gtd,imu,y,vy,dt,K-1,gain);
% %plotres
% figure(5)
% [~]  = plot_result(time,xt_OG,gtd,name);
% 
% %% 4.2 LSR to estimate x
% clc;
% x_LSR = [];
% num = 20;
% for i = 1:K-num
% %     temp = estimate_LSR(imu(:,i:i+num-1),y(:,i:i+num),dt);
%     temp = estimate_NLS(imu(:,i:i+num-1),y(:,i:i+num),vy(:,i:i+num),dt);
%     x_LSR(:,i) = temp(1:6);
% end
% figure(6)
% [~] = plot_result(time(:,1:size(x_LSR,2)),x_LSR,gtd(:,1:size(x_LSR,2)),'line\_exp');
%% 4.3 Observer
% x_Observer = Observer(imu(:,1/dt:end),y(:,1/dt:end),gtd(:,1/dt),dt);
% figure(6)
% [~]  = plot_result(time(1/dt:end),x_Observer(:,1:end-1),gtd(:,1/dt:end),'line');
x_Observer = Observer(imu,y,gtd(:,1),dt);
figure(6)
[~]  = plot_result(time,x_Observer(:,1:end-1),gtd,'line');
