%-----observable----%
%% 1. Initialize
clc;close all
clearvars -except imu_noise uwb_noise K dt sigma_omega sigma_v gain
x0 = [51;1;0;0;0]; % use prediction as initial guess
xg = [51;1;-30000;0;0;0]; % end point
% t = dt*(0:K);
[b2,a2] = butter(2,5*dt,'low'); 
%% 2. screw
t = 0:dt:K*dt;
% p_screw = [x0(1)-50+50*cos(2*pi*t/150);
%            x0(2)+50*sin(2*pi*t/150);
%            x0(3) + (xg(3) - x0(3))*t.^2/t(end)^2;];
% 
% v_screw = [-50*(2*pi/150)*sin(2*pi*t/150);
%            50*(2*pi/150)*cos(2*pi*t/150);
%            2*(xg(3) - x0(3))*t/t(end)^2;]; 
% u_screw = [-50*(2*pi/150)^2*cos(2*pi*t/150);
%            -50*(2*pi/150)^2*sin(2*pi*t/150);
%            2*(xg(3) - x0(3))*ones(1,length(t))/t(end)^2;];
omega=50;
p_screw = [x0(1)-50+50*cos(omega*pi*t/150);
           x0(2)-50+50*sin(omega*pi*t/150);
           x0(3) + (xg(3) - x0(3))*t.^3/t(end)^3;];
v_screw = [-50*(omega*pi/150)*sin(omega*pi*t/150);
           50*(omega*pi/150)*cos(omega*pi*t/150);
           3*(xg(3) - x0(3))*t.^2/t(end)^3;]; 
u_screw = [-50*(omega*pi/150)^2*cos(omega*pi*t/150);
           -50*(omega*pi/150)^2*sin(omega*pi*t/150);
           6*(xg(3) - x0(3))*t/t(end)^3;];

% ------used to testify the v_screw-----%
% figure(1)
% v_c(:,1) = [0;0;0];
% for i = 2:K+1
%     v_c(:,i) = (p_screw(:,i)-p_screw(:,i-1))/dt;
% end
%  plot(t,v_c(3,:),'k-',t,v_screw(3,:),'r--')
% ------used to testify the v_screw-----%
% ------used to testify the u_screw-----%
% figure(2)
% u_c(:,1) = [0;0;0];
% for i = 2:K+1
%     u_c(:,i) = (v_screw(:,i)-v_screw(:,i-1))/dt;
% end
% plot(t,u_c(3,:),'k-',t,u_screw(3,:),'r--')
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
x0 = [p_screw(:,1);v_screw(:,1)] + [1 1 -1 0 0 0]';
x0 = xt(:,1) + [1 1 -1 0 0 0]';
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
[~]  = plot_result(t,x_KF(1:6,:),gtd,'circular')

figure(8)
% plot(t,x_KF(7,:),'k',t,x0(1:3,1)'*x0(4:6,1)*ones(1,K+1),'k--');
plot(t,x_KF(7,:)-xt(1:3,1)'*xt(4:6,1),'k');%,t,x0(1:3,1)'*x0(4:6,1)*ones(1,K+1),'k--');
hold on
plot(t,x_KF(8,:)-xt(4:6,1)'*xt(4:6,1),'r');%,t,x0(4:6,1)'*x0(4:6,1)*ones(1,K+1),'r--');
legend('est--p0^Tv0','est--v0^2')