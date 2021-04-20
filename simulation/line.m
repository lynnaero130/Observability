%% 1. Initialize
% Initialize
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
[z_measured, imu] = cal_real(X,x0,sigma_omega,sigma_v,K);
% % filter distance and velocity
uwb = filtfilt(b2,a2,z_measured);
vy =  [0,0];
for i = 2:K+1
    vy(i) = abs(uwb(i)-uwb(i-1))/(dt);
end
%% 3.2 MHE
% x_estimate = MHE(groundtruth,u_measured,uwb_v, z_measured,dt,K,source);
% xt = MHE3(gtd,imu,uwb,vy,dt,K,gain); 
xt = MHE(gtd,imu,uwb,vy,dt,K,gain); 

figure(3)
e_line = plot_result(t,xt,gtd,'line');
%% 3.3 LSR to estimate x (line)
clc;
x_LSR = [];
num = 40;
for i = 1:K-num
    temp = estimate_LSR(imu(:,i:i+num-1),uwb(:,i:i+num),dt);
    x_LSR(:,i) = temp(1:6);
end
figure(3)
[~] = plot_result(t(:,1:size(x_LSR,2)),x_LSR,gtd(:,1:size(x_LSR,2)),'line');
