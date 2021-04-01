%% Initialize
clc;clear
K = 100; %measure times
x0 = [0;0;0;0;0;0]; % use prediction as initial guess
xg = [0.5;1;1.2;0;0;0]; % end point
rg = 0.01; % position tolerance
ru = 2; % input constraint
source = [0;0;0]; % the position of uwb
dt = 0.02;
t = dt*(0:K);
x_initial_guess = [ones(3,K) ones(3,K) zeros(3,K)];
% x_initial_guess = rand(3,3*K);
% x_initial_guess = ones(3,3*K);

%% OG-based Trajectory Planning Problem
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
%% MHE
clc;close all
sigma_omega =diag([0.001,0.001,0.001])*100; %diag([0,0,0]); % imu noise
sigma_v = 0.01;%0; % observation noise

groundtruth  = [x0 [X(:,K+1:2*K);X(:,2*K+1:3*K)]];

% measured data
[z_measured, u_measured] = cal_real(X,x0,sigma_omega,sigma_v,source,K);

% % filter distance and velocity
[b2,a2] = butter(2,5*dt,'low'); 
filter_d = filtfilt(b2,a2,z_measured);
uwb_v =  [0,0];
for i = 2:K+1
    uwb_v(i) = abs(filter_d(i)-filter_d(i-1))/(dt);
end

%MHE
% x_estimate = MHE(groundtruth,u_measured,uwb_v, z_measured,dt,K,source);
x_estimate = MHE(groundtruth,u_measured,uwb_v, filter_d,dt,K,source);
%plotres
figure(2)
e_OG = plot_result(t,x_estimate,groundtruth,'OG\_based');

% figure(3)
% frequency_analysis(x_estimate,dt)
%% 
figure(4)
subplot(2,1,1)
x_relative = groundtruth(1:3,:) - source;
gdt_d = sqrt(x_relative(1,:).^2+x_relative(2,:).^2+x_relative(3,:).^2);
plot(t,z_measured,t,filter_d,t,gdt_d)
legend('measure','filter','groundtruth')
ylabel('distance')
figure(4)
subplot(2,1,2)
gdt_v = sqrt(groundtruth(4,:).^2+groundtruth(5,:).^2+groundtruth(6,:).^2);
plot(t,uwb_v,t,gdt_v)
legend('measure','groundtruth')
ylabel('velocity')