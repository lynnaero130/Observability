%% 1. Initialize
clc;clear
% x0 = [-0.5;-0.5;0.5;0;0;0]; % use prediction as initial guess
x0 = [0;0;0;0;0;0]; % use prediction as initial guess
xg = [0.5;1.2;0.7;0;0;0]; % end point
rg = 0.01; % position tolerance
ru = 2; % input constraint
source = [0;0;0]; % the position of uwb
dt = 0.04;%1/25;
K = 2/dt; %measure times
t = dt*(0:K);
x_initial_guess = [ones(3,K) ones(3,K) zeros(3,K)];
% x_initial_guess = rand(3,3*K);
% x_initial_guess = ones(3,3*K);

%% 2. OG-based Trajectory Planning Problem
clc;
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
[X,fval] = fmincon(@(x)OG_obj(x,dt,K),x_initial_guess,[],[],[],[],[],[],@(x)nonlinear_constraint(x,x0,xg,rg,ru,K,dt),options);
% x(:,1:K) is u_0 ~ u_(K-1); each variable has three elements.
% x(:,K+1:2K) is p_1 ~ p_K
% x(:,1:K) is v_1 ~ v_K 
% plot
figure(1)
px = [x0(1) X(1,K+1:2*K)];
py = [x0(2) X(2,K+1:2*K)];
pz = [x0(3) X(3,K+1:2*K)];
plot3(px,py,pz,'s-')
% set(gca,'XLim',[-2.5 1]);
% set(gca,'YLim',[-0.6 2.5]);
hold on
plot3(xg(1),xg(2),xg(3),'om',x0(1),x0(2),x0(3),'dg',source(1),source(2),source(3),'*r')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
%% 3.1 preprocess
clc;close all
sigma_omega =diag([0.001,0.001,0.001])*10; %diag([0,0,0]); % imu noise
sigma_v = 0.01;%0; % observation noise

gtd  = [x0 [X(:,K+1:2*K);X(:,2*K+1:3*K)]];

% measured data
[z_measured, imu] = cal_real(X,x0,sigma_omega,sigma_v,source,K);

% % filter distance and velocity
[b2,a2] = butter(2,5*dt,'low'); 
uwb = filtfilt(b2,a2,z_measured);
uwb_v =  [0,0];
for i = 2:K+1
    uwb_v(i) = (uwb(i)-uwb(i-1))/(dt);%abs(uwb(i)-uwb(i-1))/(dt);
end

%% 3.2 MHE
%MHE
% x_estimate = MHE(gtd,imu,uwb_v, z_measured,dt,K,source);
xt = MHE(gtd,imu,uwb_v, uwb,dt,K,source);
%plotres
figure(2)
e_OG = plot_result(t,xt,gtd,'OG\_based');

%% 3.2 LSR to estimate x
% clc;
% x_LSR = [];
% num = 40;
% for i = 1:K-num
%     temp = estimate_LSR(imu(:,i:i+num-1),uwb(:,i:i+num),dt);
%     x_LSR(:,i) = temp(1:6);
% end
% xt = x_LSR;
% figure(3)
% [~] = plot_result(t(:,1:size(xt,2)),xt,gtd(:,1:size(xt,2)),'OG\_based');
%% 4. plot
figure(4)
subplot(2,1,1)
% x_relative = gtd(1:3,:) - source;
gdt_d = sqrt(xt(1,:).^2+xt(2,:).^2+xt(3,:).^2);
plot(t,z_measured,t,uwb,t,gdt_d)
legend('measure','filter','gtd')
ylabel('distance')
figure(4)
subplot(2,1,2)
gdt_v = sqrt(gtd(4,:).^2+gtd(5,:).^2+gtd(6,:).^2);
plot(t,uwb_v,t,gdt_v)
legend('measure','gtd')
ylabel('velocity')