%% Initialize
clc;clear
K = 100;
x0 = [-1.5;-0.5]; % use prediction as initial guess
xg = [-1;2.25]; % end point
rg = 0.1; % position tolerance
ru = 0.8; % input constraint
source = [0.2;0];
sigma_x0 = [0.025 0.002;0.002 0.025];
sigma_omega = diag([0.001,0.001]); % process noise
sigma_v = 0.01; % observation noise
dt = 0.1;
t = dt*(0:K);
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
%X_Cov(1:2,:) is u, X_Cov(3:4,:) is x 
% plot
figure(2)
px = [x0(1) X_Cov(3,:)];
py = [x0(2) X_Cov(4,:)];
plot(px,py,'s-')
set(gca,'XLim',[-2.5 1]);
set(gca,'YLim',[-0.6 2.5]);
hold on
plot(xg(1),xg(2),'om',x0(1),x0(2),'dg',source(1),source(2),'*r')

% %% Pt calculation
% tr_OG = cal_cov(X_OG,x0,K,sigma_omega,sigma_x0,source,sigma_v,dt);
% tr_Cov = cal_cov(X_Cov,x0,K,sigma_omega,sigma_x0,source,sigma_v,dt);
% tr_x = 0:K;
% figure(3)
% plot(tr_x,tr_OG,'b*-.',tr_x,tr_Cov,'kx-')
% legend('OG\_based','Cov\_based')

%% sin trajectory
% X_sin = [ones(1,length(t));cos(t);t;sin(t)];
%X_sin(1:2,:) is u, X_sin(3:4,:) is x 
xc = (x0+xg)./2;
r = norm(xg - x0,2)/2;
theta_0 = acos((xg(1)-xc(1))/r)+pi;
X_sin = [-pi*r./10*sin(pi*t./10+theta_0);pi*r./10*cos(pi*t./10+theta_0);xc(1)+r*cos(pi*t./10+theta_0);xc(2)+r*sin(pi*t./10+theta_0)];
figure(3)
plot(X_sin(3,:),X_sin(4,:),'s-')
set(gca,'XLim',[-2.5 1]);
set(gca,'YLim',[-0.6 2.5]);
hold on
plot(xg(1),xg(2),'om',x0(1),x0(2),'dg',source(1),source(2),'*r')
%% 
% X_linear = [ones(1,length(t));cos(t);t;sin(t)];
%% MHE
clc;
% Observation result z
[X_OG_pre, z_OG, u_OG] = cal_real(X_OG,x0,K,sigma_omega,source,sigma_v,dt);
[X_Cov_pre, z_Cov, u_Cov] = cal_real(X_Cov,x0,K,sigma_omega,source,sigma_v,dt);
[X_Sin_pre, z_Sin, u_Sin] = cal_real(X_sin(:,2:end),X_sin(3:4,1),K,sigma_omega,source,sigma_v,dt);

% estimation
x_OG_estimate = MHE([x0 X_OG(3:4,:)],u_OG, z_OG,dt,K,source); % K+1 dimension
x_Cov_estimate = MHE([x0 X_Cov(3:4,:)],u_Cov,z_Cov, dt,K,source);
x_Sin_estimate = MHE(X_sin(3:4,:),u_Sin,z_Sin,dt,K,source);
%% plot
figure(4)
e_OG = plot_result(t,x_OG_estimate,X_OG_pre,[x0 X_OG(3:4,:)],z_OG,source,'OG\_based');

figure(5)
e_Cov = plot_result(t,x_Cov_estimate,X_Cov_pre,[x0 X_Cov(3:4,:)],z_Cov,source,'Cov\_based');

figure(6)
e_sin = plot_result(t,x_Sin_estimate,X_Sin_pre,X_sin(3:4,:),z_Sin,source,'Sinusoid');

figure(7)
plot(t,e_OG,'r',t,e_Cov,'b',t,e_sin,'k')
legend('e\_OG','e\_Cov','e\_orthogonal')
ylabel('Error')

[mean(e_OG) mean(e_Cov) mean(e_sin);
 std(e_OG) std(e_Cov) std(e_sin)]
% plot real distance and uwb distance
% figure(6)
% OG_r = X_OG_real-source;
% d = sqrt(OG_r(1,:).^2+OG_r(2,:).^2);
% plot(t,d,t,z_OG)
% legend('real distance', 'uwb distance')

% plot real position and planned position
% figure(7)
% plot(t(2:end),X_OG_real(1,2:end),'r--',t(2:end),X_OG(3,:),'r-')
% hold on
% plot(t(2:end),X_OG_real(2,2:end),'b--',t(2:end),X_OG(4,:),'b-')
% legend('x_{real}', 'x_{desired}','y_{real}', 'y_{desired}')
