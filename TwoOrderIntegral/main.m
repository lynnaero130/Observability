%% Initialize
clc;clear
K = 100; %measure times
x0 = [0;0;0;0;0;0]; % use prediction as initial guess
xg = [0.2;0.2;.0;0;0;0]; % end point
rg = 0.01; % position tolerance
ru = 2; % input constraint
source = [0.5;0;0]; % the position of uwb
dt = 0.02;
t = dt*(0:K);

x_initial_guess = [zeros(3,K) zeros(3,K) zeros(3,K)];

% x_initial_guess = rand(3,3*K);
% x_initial_guess = ones(3,3*K);
%% OG-based Trajectory Planning Problem
clc;
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
[X,fval] = fmincon(@(x)OG_obj(x,dt,K),x_initial_guess,[],[],[],[],[],[],@(x)nonlinear_constraint(x,x0,xg,rg,ru,K,dt),options)
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
% clc;
% % Observation result z
% [X_OG_pre, z_OG, u_OG] = cal_real(X,x0,K,sigma_omega,source,sigma_v,dt);
% 
% % estimation
% x_OG_estimate = MHE([x0 X(3:4,:)],u_OG, z_OG,dt,K,source); % K+1 dimension
