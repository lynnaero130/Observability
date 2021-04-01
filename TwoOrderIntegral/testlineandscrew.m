clc;
%% straight line
p_line = x0(1:3)+(xg(1:3)-x0(1:3))*t/(K*dt);
v_line = 0.5*ones(3,K+1);
u_line = zeros(3,K);
% X_line = [u_line p_line u_line];
figure(1)
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
%% run mhe
[x_hat_line,gdt_line,uwb_v_line,z_measured_line,filter_z_line] = estimate(p_line,v_line,u_line,sigma_omega,sigma_v,source,K,dt);
[x_hat_screw,gdt_screw,uwb_v_screw,z_measured_screw,filter_z_screw] = estimate(p_screw,v_screw,u_screw,sigma_omega,sigma_v,source,K,dt);
%% plotres
figure(3)
e_line = plot_result(t,x_hat_line,gdt_line,'line');
figure(4)
e_screw = plot_result(t,x_hat_screw,gdt_screw,'screw');

