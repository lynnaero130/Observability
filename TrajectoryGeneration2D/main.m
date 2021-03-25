%% Initialize
clc;clear
K = 100;
x0 = [-1.5;-0.5]; % use prediction as initial guess
xg = [-1;2.25]; % end point
rg = 0.1; % position tolerance
ru = 0.8; % input constraint
source = [0.2;0];
sigma_x0 = [0.025 0.002;0.002 0.025];
sigma_omega = diag([0 0]);%diag([0.3,0.1]); % process noise
sigma_v = 0;%0.1; % observation noise
dt = 0.1;
x_initial_guess = ones(4,K);

%% OG-based Trajectory Planning Problem
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
% [X_OG,fval] = fmincon(@(x)OG(x,x0,source),x_initial_guess,[],[],[],[],[],[],@(x)nonlcon(x,xg,rg,ru,x0,K,dt),options) % det
[X_OG,fval] = fmincon(@(x)OG_cond(x,x0,source),x_initial_guess,[],[],[],[],[],[],@(x)nonlinear_constraint(x,xg,rg,ru,x0,K,dt),options) % condition number
%X_OG(1:2,:) is u, X_OG(3:4,:) is x 
% plot
figure(1)
px = [x0(1) X_OG(3,:)];
py = [x0(2) X_OG(4,:)];
plot(px,py,'s-')
set(gca,'XLim',[-2.5 1]);
set(gca,'YLim',[-0.6 2.5]);
hold on
plot(xg(1),xg(2),'om',x0(1),x0(2),'dg',source(1),source(2),'*r')
%% Cov_based Planning Problem
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
[X_Cov,fval] = fmincon(@(x)TLQG(x,K,source,sigma_x0,sigma_omega,sigma_v),x_initial_guess,[],[],[],[],[],[],@(x)nonlinear_constraint(x,xg,rg,ru,x0,K,dt),options) % condition number

% plot
figure(2)
px = [x0(1) X_Cov(3,:)];
py = [x0(2) X_Cov(4,:)];
plot(px,py,'s-')
set(gca,'XLim',[-2.5 1]);
set(gca,'YLim',[-0.6 2.5]);
hold on
plot(xg(1),xg(2),'om',x0(1),x0(2),'dg',source(1),source(2),'*r')

%% Pt calculation
tr_OG = cal_cov(X_OG,x0,K,sigma_omega,sigma_x0,source,sigma_v,dt);
tr_Cov = cal_cov(X_Cov,x0,K,sigma_omega,sigma_x0,source,sigma_v,dt);
tr_x = 0:K;
figure(3)
plot(tr_x,tr_OG,'b*-.',tr_x,tr_Cov,'kx-')
legend('OG\_based','Cov\_based')

%% MHE
clc;
% Observation result z
[X_OG_real, z_OG, u_OG] = cal_real(X_OG,x0,K,sigma_omega,source,sigma_v,dt);
[X_Cov_real, z_Cov, u_Cov] = cal_real(X_Cov,x0,K,sigma_omega,source,sigma_v,dt);

% estimation
x_OG_estimate = MHE(X_OG_real,u_OG, z_OG,dt,K,source);
x_Cov_estimate = MHE(X_Cov_real,u_Cov,z_Cov, dt,K,source);

%% plot
figure(4)
t = dt*(1:K);
subplot(2,1,1)
plot(t,x_OG_estimate(1,:),'r--',t,X_OG_real(1,1:end-1),'r-')
hold on
plot(t,x_OG_estimate(2,:),'b--',t,X_OG_real(2,1:end-1),'b-')
legend('x_{hat}','x_g','y_{hat}','y_g')
ylabel('Position')
title('OG\_based')
subplot(2,1,2)
error_OG = X_OG_real-x_OG_estimate;
error_OG_norm = sqrt(error_OG(1,:).^2 + error_OG(2,:).^2);
plot(t,error_OG_norm,'k-')
ylabel('estimation error')

figure(5)
subplot(2,1,1)
plot(t,x_Cov_estimate(1,:),'r--',t,X_Cov_real(1,:),'r-')
hold on
plot(t,x_Cov_estimate(2,:),'b--',t,X_Cov_real(2,:),'b-')
legend('x_{hat}','x_g','y_{hat}','y_g')
ylabel('Position')
title('Cov\_based')
set(gca,'YLim',[-4 4]);
subplot(2,1,2)
error_Cov = X_Cov_real-x_Cov_estimate;
error_Cov_norm = sqrt(error_Cov(1,:).^2 + error_Cov(2,:).^2);
plot(t,error_Cov_norm,'k-')
set(gca,'YLim',[0 4]);
ylabel('estimation error')

% plot real distance and uwb distance
% figure(6)
% OG_r = X_OG_real-source;
% d = sqrt(OG_r(1,:).^2+OG_r(2,:).^2);
% plot(t,d,t,z_OG)
% legend('real distance', 'uwb distance')

% plot real position and planned position
% figure(7)
% plot(t,X_OG_real(1,:),'r--',t,X_OG(3,:),'r-')
% hold on
% plot(t,X_OG_real(2,:),'b--',t,X_OG(4,:),'b-')
% legend('x_{real}', 'x_{desired}','y_{real}', 'y_{desired}')

% plot real position and predicted position
figure(8)
plot(t,X_OG_real(1,:),'r--',t,xp(1,:),'r-')
hold on
plot(t,X_OG_real(2,:),'b--',t,xp(2,:),'b-')
legend('x_{real}', 'x_{prediction}','y_{real}', 'y_{prediction}')