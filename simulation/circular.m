%-----observable----%
%% 1. Initialize
clc;close all
clearvars -except imu_noise uwb_noise K dt sigma_omega sigma_v gain
x0 = [51;1;0;0;0]; % use prediction as initial guess
xg = [51;1;-300;0;0;0]; % end point
% t = dt*(0:K);
[b2,a2] = butter(2,5*dt,'low'); 
%% 2. screw
t = 0:dt:K*dt;
% omega=10;
% p_screw = [x0(1)-50+200*cos(omega*pi*t/150);
%            x0(2)-50+200*sin(omega*pi*t/150);
%            x0(3) + 2*(xg(3) - x0(3))*t.^3/t(end)^3;];
% v_screw = [-200*(omega*pi/150)*sin(omega*pi*t/150);
%            200*(omega*pi/150)*cos(omega*pi*t/150);
%            2*3*(xg(3) - x0(3))*t.^2/t(end)^3;]; 
% u_screw = [-200*(omega*pi/150)^2*cos(omega*pi*t/150);
%            -200*(omega*pi/150)^2*sin(omega*pi*t/150);
%           2* 6*(xg(3) - x0(3))*t/t(end)^3;];
omega=10;
p_screw = [x0(1)-200+200*cos(omega*pi*t/150); %T = 2*pi/(omega*pi/150) = 300/omega;
           x0(2)+200*sin(omega*pi*t/150);
           x0(3)-200+200*cos(2*omega*pi*t/150);];  %T = 2*pi/(2*omega*pi/150) = 150/omega;
v_screw = [-200*(omega*pi/150)*sin(omega*pi*t/150);
           200*(omega*pi/150)*cos(omega*pi*t/150);
           -200*(2*omega*pi/150)*sin(2*omega*pi*t/150);]; 
u_screw = [-200*(omega*pi/150)^2*cos(omega*pi*t/150);
           -200*(omega*pi/150)^2*sin(omega*pi*t/150);
          -200*(2*omega*pi/150)^2*cos(2*omega*pi*t/150);];

% ------used to testify the v_screw-----%
figure(1)
v_c(:,1) = [0;0;0];
for i = 2:K+1
    v_c(:,i) = (p_screw(:,i)-p_screw(:,i-1))/dt; % first order approixmate
end
 plot(t,v_c(3,:),'k-',t,v_screw(3,:),'r--')
% ------used to testify the v_screw-----%
% ------used to testify the u_screw-----%
figure(2)
u_c(:,1) = [0;0;0];
for i = 2:K+1
    u_c(:,i) = (v_screw(:,i)-v_screw(:,i-1))/dt;
end
plot(t,u_c(3,:),'k-',t,u_screw(3,:),'r--')
% ------used to testify the u_screw-----%
       
figure(3)
plot3(p_screw(1,:),p_screw(2,:),p_screw(3,:),'m',x0(1),x0(2),x0(3),'og',0,0,0,'dr');
xlabel('x')
ylabel('y')
zlabel('z')
grid on
% csvwrite('./data/screw.csv',[p_screw'])
%% 3.1 generate data
X = [u_screw(:,1:K) p_screw(:,1:K) v_screw(:,1:K)];
gtd  = [p_screw;v_screw];

% measured data
[z_measured, imu,xt] = cal_real2(X,x0,sigma_omega,sigma_v,K,dt); % z_measured is uwb, imu is real control input, xt is real state


% % filter distance and velocity
uwb = filtfilt(b2,a2,z_measured);
vy =  [0,0];
for i = 2:K+1
    vy(i) = abs(uwb(i)-uwb(i-1))/(dt);
end
uwb =z_measured;
%% 3.5 KF
x0 = xt(:,1) + [0 0 1 0 0 0]';
% x0(1) = x0(1)+5;
[x_KF,x_pre] = KF(u_screw(:,1:K),uwb,x0,dt,sigma_omega,sigma_v);

figure(6)
plot3(x_pre(1,:),x_pre(2,:),x_pre(3,:),'r')
hold on 
plot3(xt(1,:),xt(2,:),xt(3,:),'k--')
legend('predict trajectory','desired trajectory')
xlabel('x')
ylabel('y')
zlabel('z')
grid on

figure(7)
[~]  = plot_result(t,x_KF(1:6,:),xt,'circular')

figure(8)
set(gcf,'Position',[100,20,600,600]);
subplot(3,1,1)
plot(t,x_KF(1,:)-xt(1,:),'r',t,x_KF(2,:)-xt(2,:),'b',t,x_KF(3,:)-xt(3,:),'k','linewidth',1);
legend('x','y','z')
ylabel('Position Error')
grid on
subplot(3,1,2)
plot(t,x_KF(4,:)-xt(4,:),'r',t,x_KF(5,:)-xt(5,:),'b',t,x_KF(6,:)-xt(6,:),'k','linewidth',1);
legend('vx','vy','vz')
ylabel('Velocity Error')
grid on
subplot(3,1,3)
plot(t,x_KF(7,:)-xt(1:3,1)'*xt(4:6,1),'k','linewidth',1);%,t,x0(1:3,1)'*x0(4:6,1)*ones(1,K+1),'k--');
hold on
plot(t,x_KF(8,:)-xt(4:6,1)'*xt(4:6,1),'r','linewidth',1);%,t,x0(4:6,1)'*x0(4:6,1)*ones(1,K+1),'r--');
legend('error--p0^Tv0','error--v0^2')
ylabel('x_7,x_8')
grid on
figure(9)
subplot(2,1,1)
plot(t,sqrt((x_KF(1,:)-xt(1,:)).^2 + (x_KF(2,:)-xt(2,:)).^2+ (x_KF(3,:)-xt(3,:)).^2),'r','linewidth',1)
ylabel('position error norm')
grid on
subplot(2,1,2)
plot(t,sqrt((x_KF(4,:)-xt(4,:)).^2 + (x_KF(5,:)-xt(5,:)).^2+ (x_KF(6,:)-xt(6,:)).^2),'r','linewidth',1)
ylabel('velocity error norm')
grid on