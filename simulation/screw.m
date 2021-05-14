%% 1. Initialize
clc;close all
clearvars -except imu_noise uwb_noise dt K sigma_omega sigma_v gain
% x0 = [0;0;1;0;0;0]; % use prediction as initial guess
x0 = [-50;-50;-30;0;0;0]; % use prediction as initial guess
xg = [50;60;80;0;0;0]; % end point
rg = 0.01; % position tolerance
ru = 2; % input constraint
t = dt*(0:K);
x_initial_guess = [ones(3,K) ones(3,K) zeros(3,K)];
[b2,a2] = butter(2,5*dt,'low'); 
%% 2. screw
p_screw = [x0(1)+(xg(1)-x0(1))*sin(pi*t/(2*K*dt));
           xg(2)+(x0(2)-xg(2))*cos(pi*t/(2*K*dt));
           (x0(3)+xg(3))/2+(x0(3)-xg(3))*cos(pi*t/(K*dt))/2;];
v_screw = [(xg(1)-x0(1))*pi*cos(pi*t/(2*K*dt))/(2*K*dt);
           (xg(2)-x0(2))*pi*sin(pi*t/(2*K*dt))/(2*K*dt);
           (xg(3)-x0(3))*pi*sin(pi*t/(K*dt))/(2*K*dt);]; 
u_screw = [-(xg(1)-x0(1))*pi^2*sin(pi*t/(2*K*dt))/(2*K*dt)^2;
           (xg(2)-x0(2))*pi^2*cos(pi*t/(2*K*dt))/(2*K*dt)^2;
           (xg(3)-x0(3))*pi^2*cos(pi*t/(K*dt))/2/(K*dt)^2;];

figure(2)
plot3(p_screw(1,:),p_screw(2,:),p_screw(3,:),'om',x0(1),x0(2),x0(3),'dg',xg(1),xg(2),xg(3),'*r')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
% csvwrite('./data/screw.csv',[p_screw'])
%% 3.1 generate data
X = [u_screw(:,1:K) p_screw(:,2:K+1) v_screw(:,2:K+1)];
% gtd  = [p_screw;v_screw];

% measured data
[z_measured, u,gtd] = cal_real2(X,x0,sigma_omega,sigma_v,K,dt); % xt is gdt, imu is real control input, u_screw is measured control input
imu = u_screw(:,1:K);
%-----------sgolayfilt filter--------------%
order = 4;
framelen = 31;
% y = sgolayfilt(uwb,order,framelen);
uwb = sgolayfilt(z_measured, order, framelen); 

%------------ buterworth filter----------%
% uwb = filtfilt(b2,a2,z_measured);

vy =  [0,0];
for i = 2:K+1
    vy(i) = abs(uwb(i)-uwb(i-1))/(dt);
end

%% 3.2 MHE
% clc
% xt = MHE(gtd,imu,uwb,vy,dt,K,gain);
% %plotres
% figure(2)
% [~]  = plot_result(t,xt,gtd,'screw');

%% 3.3 LSR to estimate x (screw)
% clc;
% x_LSR = [];
% num = 20;
% for i = 1:K-num
%     temp = estimate_LSR(imu(:,i:i+num-1),uwb(:,i:i+num),dt);
% %     temp = estimate_NLS(imu(:,i:i+num-1),uwb(:,i:i+num),vy(:,i:i+num),dt);
%     x_LSR(:,i) = temp(1:6);
% end
% figure(5)
% [~] = plot_result2(t(:,1:size(x_LSR,2)),x_LSR,gtd(:,1:size(x_LSR,2)),'screw');

%% 3.4 Observer
% x0 = [x0(1:3);v_screw(:,1)];
% [x_con,xd] = Observer(imu,uwb,x0,dt);
% figure(6)
% [~]  = plot_result(t,x_con,gtd,'con');
% figure(7)
% [~]  = plot_result(t,xd,gtd,'discon');
%% 3.5 KF
close all;
% x0 = [x0(1:3);v_screw(:,1)];%+0.5;%-0.2;%+0.01;%+0.02;
x0 = [gtd(1:3,1);gtd(4:6,1)] + [1 1 -1 0 0 0]';
% x0(1) = x0(1)+5;
[x_KF,xt1] = KF(imu,uwb,x0,dt,sigma_omega,sigma_v);
% real u is u (u_screw + noise)

% figure(6)
% plot3(gtd(1,:),gtd(2,:),gtd(3,:),'r',x0(1),x0(2),x0(3),'dg',xg(1),xg(2),xg(3),'*r')
% hold on 
% plot3(p_screw(1,:),p_screw(2,:),p_screw(3,:),'k--')
% legend('real trajectory','desired trajectory')
% xlabel('x')
% ylabel('y')
% zlabel('z')
% grid on

figure(7)
[~]  = plot_result(t,x_KF(1:6,:),gtd,'screw');

figure(8)
plot(t,x_KF(7,:)-gtd(1:3,1)'*gtd(4:6,1),'k');
hold on
plot(t,x_KF(8,:)-gtd(4:6,1)'*gtd(4:6,1),'r--');
legend('est--p0^Tv0','real--p0^Tv0','est--v0^2','real--v0^2')
