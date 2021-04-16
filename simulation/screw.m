%% 1. Initialize
% Initialize
%% 2. screw
p_screw = [x0(1)+(xg(1)-x0(1))*sin(pi*t/(2*K*dt));
           xg(2)+(x0(2)-xg(2))*cos(pi*t/(2*K*dt));
           (x0(3)+xg(3))/2+(x0(3)-xg(3))*cos(pi*t/(K*dt))/2;];
v_screw = [(xg(1)-x0(1))*pi*cos(pi*t/(2*K*dt))/(2*K*dt);
           (xg(2)-x0(2))*pi*sin(pi*t/(2*K*dt))/(2*K*dt);
           (xg(3)-x0(3))*pi*sin(pi*t/(K*dt))/(2*K*dt);]; 
u_screw = [-(xg(1)-x0(1))*pi^2*sin(pi*t/(2*K*dt))/(2*K*dt)^2;
           (xg(2)-x0(2))*pi^2*cos(pi*t/(2*K*dt))/(2*K*dt)^2;
           (xg(3)-x0(3))*pi^2*cos(pi*t/(K*dt))/2/(K*dt)^2;];

figure(2)
plot3(p_screw(1,:),p_screw(2,:),p_screw(3,:),'om',x0(1),x0(2),x0(3),'dg',xg(1),xg(2),xg(3),'*r')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
% csvwrite('./data/screw.csv',[p_screw'])
%% 3.1 run mhe screw
X_screw = [u_screw(:,1:K) p_screw(:,2:K+1) v_screw(:,2:K+1)];
gdt_screw  = [p_screw;v_screw];
% measured data
[z_measured_screw, u_measured_screw] = cal_real(X_screw,x0,sigma_omega,sigma_v,K);
% % filter distance and velocity
filter_d_screw = filtfilt(b2,a2,z_measured_screw);
uwb_v_screw =  [0,0];
for i = 2:K+1
    uwb_v_screw(i) = abs(filter_d_screw(i)-filter_d_screw(i-1))/(dt);
end

%MHE
x_estimate_screw = MHE(gdt_screw,u_measured_screw,filter_d_screw,uwb_v_screw,dt,K,gain);

figure(4)
[~]  = plot_result(t,x_estimate_screw,gdt_screw,'screw');

%% 3.2 LSR to estimate x (screw)
% clc;
% x_LSR = [];
% num = 15;
% for i = 1:K-num
%     temp = estimate_LSR(u_measured_screw(:,i:i+num-1),filter_d_screw(:,i:i+num),dt);
%     x_LSR(:,i) = temp(1:6);
% end
% figure(5)
% [~] = plot_result(t(:,1:size(x_LSR,2)),x_LSR,gdt_screw(:,1:size(x_LSR,2)),'screw');