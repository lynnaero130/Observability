%-----unobservable----%
%% 1. Initialize
clc;close all
clearvars -except imu_noise uwb_noise K dt sigma_omega sigma_v gain
x0 = [51;1;4;0;0;0]; % use prediction as initial guess
xg = [51;1;-300;0;0;0]; % end point
% t = dt*(0:K);
[b2,a2] = butter(2,5*dt,'low'); 
%% 2. screw
% z is t;
t = 0:dt:K*dt;
omega=50;
p_screw = [x0(1)-50+50*cos(omega*pi*t/150);
           x0(2) + 50*sin(omega*pi*t/150);
           x0(3) +  50*sin(omega*pi*t/150);];
v_screw = [-50*(omega*pi/150)*sin(omega*pi*t/150);
           (omega*pi/150)*50*cos(omega*pi*t/150);
           (omega*pi/150)*50*cos(omega*pi*t/150);]; 
u_screw = [-50*(omega*pi/150)^2*cos(omega*pi*t/150);
           -(omega*pi/150)^2*50*sin(omega*pi*t/150);
           -(omega*pi/150)^2*50*sin(omega*pi*t/150);];
% ------used to testify the v_screw-----%
% v_c(:,1) = [0;0;0];
% for i = 2:K+1
%     v_c(:,i) = (p_screw(:,i)-p_screw(:,i-1))/dt;
% end
%  plot(t,v_c(1,:),'k-',t,v_screw(1,:),'r--')
% ------used to testify the v_screw-----%
% ------used to testify the u_screw-----%
% u_c(:,1) = [0;0;0];
% for i = 2:K+1
%     u_c(:,i) = (v_screw(:,i)-v_screw(:,i-1))/dt;
% end
% plot(t,u_c(1,:),'k-',t,u_screw(1,:),'r--')
% ------used to testify the u_screw-----%     
figure(2)
plot3(p_screw(1,:),p_screw(2,:),p_screw(3,:),'om',x0(1),x0(2),x0(3),'dg',xg(1),xg(2),xg(3),'*r')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
% csvwrite('./data/screw.csv',[p_screw'])
%% 3.1 generate data
X = [u_screw(:,1:K) p_screw(:,2:K+1) v_screw(:,2:K+1)];
gtd  = [p_screw;v_screw];

% measured data
[z_measured, imu,xt] = cal_real2(X,x0,sigma_omega,sigma_v,K,dt);

% % filter distance and velocity
uwb = filtfilt(b2,a2,z_measured);
vy =  [0,0];
for i = 2:K+1
    vy(i) = abs(uwb(i)-uwb(i-1))/(dt);
end
uwb =z_measured;
%% 3.5 KF
close all
x0 = [x0(1:3);v_screw(:,1)] + [1 1 -1 0 0 0]';
x0 = xt(:,1) + [1 1 -1 0 0 0]';
% x0(1) = x0(1)+5;
x_KF = KF(u_screw(:,1:K),uwb,x0,dt,sigma_omega,sigma_v);

figure(6)
plot3(xt(1,:),xt(2,:),xt(3,:),'r',x0(1),x0(2),x0(3),'dg',xg(1),xg(2),xg(3),'*r')
hold on 
plot3(xt(1,:),xt(2,:),xt(3,:),'k--')
legend('real trajectory','desired trajectory')
xlabel('x')
ylabel('y')
zlabel('z')
grid on

figure(7)
[~]  = plot_result(t,x_KF(1:6,:),xt,'circular2')

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