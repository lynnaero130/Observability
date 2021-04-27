%% 1. Initialize
clc;close all
clearvars -except imu_noise uwb_noise  dt K sigma_omega sigma_v gain
x0 = [-0.5;-0.5;0.5;0;0;0]; % use prediction as initial guess
% x0 = [0;0;0;0;0;0]; % use prediction as initial guess
xg = [0.5;1;1.2;0;0;0]; % end point

x0 = [0;0;0;0;0;0]; % use prediction as initial guess
xg = [50;50;10;0;0;0]; % end point

% x0 = [3;0;0;0;0;0]; % use prediction as initial guess
% % x0 = [0;0;0;0;0;0]; % use prediction as initial guess
% xg = [5;10;12;0;0;0]; % end point
rg = 0.01; % position tolerance
ru = 2; % input constraint
t = dt*(0:K);
x_initial_guess = [ones(3,K) ones(3,K) zeros(3,K)];
[b2,a2] = butter(2,5*dt,'low'); 
%% 2. straight line
p_line = x0(1:3)+(xg(1:3)-x0(1:3))*t/(K*dt);
v_line = (xg(1:3)-x0(1:3))/(K*dt).*ones(1,K+1);%0.5*ones(3,K+1);%
u_line = zeros(3,K);
% X_line = [u_line p_line u_line];
figure(1)
plot3(p_line(1,:),p_line(2,:),p_line(3,:),'om',x0(1),x0(2),x0(3),'dg',xg(1),xg(2),xg(3),'*r')
xlabel('x')
ylabel('y')
zlabel('z')
% grid on
% csvwrite('./data/line.csv',[p_line'])

%% 3.1 generate measured data
X = [u_line(:,1:K) p_line(:,2:K+1) v_line(:,2:K+1)];
gtd  = [p_line;v_line];
% measured data
[z_measured, imu, xt] = cal_real2(X,x0,sigma_omega,sigma_v,K,dt);

figure

plot(X(1,K+1:2*K).^2+X(2,K+1:2*K).^2+X(3,K+1:2*K).^2);
hold on
plot(z_measured.^2,'r');
legend('sensor','uwb')
% return;
% % filter distance and velocity
uwb=z_measured;
uwb = filtfilt(b2,a2,z_measured);
vy =  [0,0];
for i = 2:K+1
    vy(i) = abs(uwb(i)-uwb(i-1))/(dt);
end
uwb=z_measured;
%% 3.2 MHE
% xt = MHE(gtd,imu,uwb,vy,dt,K,gain); 
% 
% figure(3)
% [~] = plot_result(t,xt,gtd,'line');
% % %% 3.3 LSR to estimate x (line)
% % clc;
% % x_LSR = [];
% % num = 20;
% % for i = 1:K-num
% %     temp = estimate_LSR(imu(:,i:i+num-1),uwb(:,i:i+num),dt);
% % %     temp = estimate_NLS(imu(:,i:i+num-1),y(:,i:i+num),vy(:,i:i+num),dt);
% %     x_LSR(:,i) = temp(1:6);
% % end
% % figure(3)
% % [~] = plot_result2(t(:,1:size(x_LSR,2)),x_LSR,gtd(:,1:size(x_LSR,2)),'line');
% %% 3.4 Observer
% % x0 = [x0(1:3);v_line(:,1)]
% x_Observer = Observer(imu,uwb,x0,dt);
% figure(6)
% [~]  = plot_result(t,x_Observer,gtd,'line');

%% 3.5 KF
close all;
x0 = [x0(1:3);v_line(:,1)];%/-0.2;
x0(1) = x0(1)+5;
% imu = imu + imu_noise;
% x0 = [x0(1:3);zeros(3,1)];
% x_KF = KF(imu,uwb,x0,dt,sigma_omega,sigma_v);
[x_KF,xt1] = KF(u_line,uwb,x0,dt,sigma_omega,sigma_v,1);

figure(7)
[~]  = plot_result(t,x_KF(1:6,:),xt,'line');

figure(8)
plot(sqrt(abs(x_KF(8,:))));
hold on
plot(x_KF(7,:)./sqrt(abs(x_KF(8,:))),'r');
% 
% x_KF1(1:6,:)=x_KF(1:6,:)-xt(1:6,:);
% 
 p0=norm(gtd(1:3,1))
 v0=norm(gtd(4:6,1))