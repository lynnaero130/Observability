%% 1. Initialize
clc;close all
clearvars -except imu_noise uwb_noise K dt sigma_omega sigma_v gain
x0 = [51;1;4;]; % use prediction as initial guess
% x0 = [0;0;0;0;0;0]; % use prediction as initial guess
xg = [51;1;-300]; % end point
% t = dt*(0:K);
[b2,a2] = butter(2,5*dt,'low'); 
%% 2. screw
t = 0:dt:K*dt;
p_screw = [x0(1)-50+50*cos(2*pi*t/150);
           x0(2) + sin(2*pi*t/150);
           x0(3) + (xg(3) - x0(3))*t.^2/t(end)^2;];
% ------used to testify the v_screw-----%
% v_c(:,1) = [0;0;0];
% for i = 2:K+1
%     v_c(:,i) = (p_screw(:,i)-p_screw(:,i-1))/dt;
% end
%  plot(t,v_c(1,:),'k-',t,v_screw(1,:),'r--')
% ------used to testify the v_screw-----%

v_screw = [-50*(2*pi/150)*sin(2*pi*t/150);
           (2*pi/150)*cos(2*pi*t/150);
           2*(xg(3) - x0(3))*t/t(end)^2;]; 

% ------used to testify the u_screw-----%
% u_c(:,1) = [0;0;0];
% for i = 2:K+1
%     u_c(:,i) = (v_screw(:,i)-v_screw(:,i-1))/dt;
% end
% plot(t,u_c(1,:),'k-',t,u_screw(1,:),'r--')
% ------used to testify the u_screw-----%
u_screw = [-50*(2*pi/150)^2*cos(2*pi*t/150);
           -(2*pi/150)^2*sin(2*pi*t/150);
           2*(xg(3) - x0(3))*ones(1,length(t))/t(end)^2;];
       
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
[z_measured, imu] = cal_real(X,x0,sigma_omega,sigma_v,K);

% % filter distance and velocity
uwb = filtfilt(b2,a2,z_measured);
vy =  [0,0];
for i = 2:K+1
    vy(i) = abs(uwb(i)-uwb(i-1))/(dt);
end

%% 3.5 KF
x0 = [x0(1:3);v_screw(:,1)];
x_KF = KF(imu,uwb,x0,dt,sigma_omega,sigma_v,gtd);
% x_measured = p_screw + 0.001*randn(3,size(p_screw,2));
% x_KF = KF(imu,uwb,x0,dt,sigma_omega,sigma_v,x_measured);

figure(7)
[~]  = plot_result(t,x_KF(1:6,:),gtd,'circular')