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
sigma_omega =diag([0.001,0.001,0.001])*100; %diag([0,0,0]); % imu noise
sigma_v = 0.01;%0; % observation noise

%% OG-based Trajectory Planning Problem
clc;
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
[X,fval] = fmincon(@(x)OG_obj(x,dt,K),x_initial_guess,[],[],[],[],[],[],@(x)nonlinear_constraint(x,x0,xg,rg,ru,K,dt),options);
u_OG = X(:,1:K);
p_OG = [x0(1:3) X(:,K+1:2*K)];
v_OG = [x0(4:6) X(:,2*K+1:3*K)];
% x(:,1:K) is u_0 ~ u_(K-1); each variable has three elements.
% x(:,K+1:2K) is p_1 ~ p_K
% x(:,2K+1:3K) is v_1 ~ v_K 
% plot
figure(1)
px = p_OG(1,:);
py = p_OG(2,:);
pz = p_OG(3,:);
plot3(px,py,pz,'s-')
hold on
plot3(xg(1),xg(2),xg(3),'om',x0(1),x0(2),x0(3),'dg',source(1),source(2),source(3),'*r')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
%% straight line
p_line = x0(1:3)+(xg(1:3)-x0(1:3))*sin(pi*t/(2*K*dt));
v_line = (xg(1:3)-x0(1:3))*pi*cos(pi*t/(2*K*dt))/(2*K*dt);
u_line = -(xg(1:3)-x0(1:3))*(pi/(2*K*dt))^2*sin(pi*t/(2*K*dt));
% X_line = [u_line p_line u_line];
figure(2)
plot3(p_line(1,:),p_line(2,:),p_line(3,:),'om',x0(1),x0(2),x0(3),'dg',source(1),source(2),source(3),'*r')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
%% screw
p_screw = [x0(1)+(xg(1)-x0(1))*sin(pi*t/(2*K*dt));
           xg(2)+(x0(2)-xg(2))*cos(pi*t/(2*K*dt));
           x0(3)+(x0(3)-xg(3))*sin(3*pi*t/(2*K*dt));];
v_screw = [(xg(1)-x0(1))*pi*cos(pi*t/(2*K*dt))/(2*K*dt);
           (xg(2)-x0(2))*pi*sin(pi*t/(2*K*dt))/(2*K*dt);
           (x0(3)-xg(3))*3*pi*cos(3*pi*t/(2*K*dt))/(2*K*dt);];
u_screw = [-(xg(1)-x0(1))*pi^2*sin(pi*t/(2*K*dt))/(2*K*dt)^2;
           (xg(2)-x0(2))*pi^2*cos(pi*t/(2*K*dt))/(2*K*dt)^2;
           -(x0(3)-xg(3))*9*pi^2*sin(3*pi*t/(2*K*dt))/(2*K*dt)^2;];
% X_screw = [u_screw p_screw v_screw];     
figure(2)
plot3(p_screw(1,:),p_screw(2,:),p_screw(3,:),'om',x0(1),x0(2),x0(3),'dg',source(1),source(2),source(3),'*r')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
%% MHE
clc;close all
[x_hat_OG,gdt_OG,uwb_v_OG,z_measured_OG,filter_z_OG] = estimate(p_OG,v_OG,u_OG,sigma_omega,sigma_v,source,K,dt);
[x_hat_line,gdt_line,uwb_v_line,z_measured_line,filter_z_line] = estimate(p_line,v_line,u_line,sigma_omega,sigma_v,source,K,dt);
[x_hat_screw,gdt_screw,uwb_v_screw,z_measured_screw,filter_z_screw] = estimate(p_screw,v_screw,u_screw,sigma_omega,sigma_v,source,K,dt);
%plotres
figure(2)
e_OG = plot_result(t,x_hat_OG,gdt_OG,'OG\_based');
e_line = plot_result(t,x_hat_line,gdt_line,'line');
e_screw = plot_result(t,x_hat_screw,gdt_screw,'screw');
%% 
figure(4)
subplot(2,1,1)
x_relative = gdt_OG(1:3,:) - source;
gdt_d = sqrt(x_relative(1,:).^2+x_relative(2,:).^2+x_relative(3,:).^2);
plot(t,z_measured_OG,t,filter_z_OG,t,gdt_d)
legend('measure','filter','groundtruth')
ylabel('distance')
figure(4)
subplot(2,1,2)
gdt_v = sqrt(gdt_OG(4,:).^2+gdt_OG(5,:).^2+gdt_OG(6,:).^2);
plot(t,uwb_v_OG,t,gdt_v)
legend('measure','groundtruth')
ylabel('velocity')