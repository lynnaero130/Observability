%% 1. Initialize
% Initialize
%% 2. OG-based Trajectory Planning Problem
clc;
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
[X,fval] = fmincon(@(x)OG_obj(x,dt,K),x_initial_guess,[],[],[],[],[],[],@(x)nonlinear_constraint(x,x0,xg,rg,ru,K,dt),options);
% x(:,1:K) is u_0 ~ u_(K-1); each variable has three elements.
% x(:,K+1:2K) is p_1 ~ p_K
% x(:,2K+1:3K) is v_1 ~ v_K 
figure(1)
px = [x0(1) X(1,K+1:2*K)];
py = [x0(2) X(2,K+1:2*K)];
pz = [x0(3) X(3,K+1:2*K)];
plot3(px,py,pz,'om',x0(1),x0(2),x0(3),'dg',xg(1),xg(2),xg(3),'*r')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
hold on
% plot3(d_OG.px,d_OG.py,d_OG.pz,'r--',l_OG.px,l_OG.py,l_OG.pz,'b--')
% csvwrite('./data/OG.csv',[px' py' pz'])
%% 3.1 preprocess
clc;close all
gtd  = [x0 [X(:,K+1:2*K);X(:,2*K+1:3*K)]];

% measured data
[z_measured, imu] = cal_real(X,x0,sigma_omega,sigma_v,K);

% % filter distance and velocity
uwb = filtfilt(b2,a2,z_measured);
vy =  [0,0];
for i = 2:K+1
    vy(i) = abs(uwb(i)-uwb(i-1))/(dt);
end

%% 3.2 MHE
clc
xt = MHE2(gtd,imu,uwb,vy,dt,K,gain);
%plotres
figure(2)
[~]  = plot_result(t,xt,gtd,'OG\_based');

%% 3.2 LSR to estimate x
% clc;
% x_LSR = [];
% num = 15;
% for i = 1:K-num
%     temp = estimate_LSR(imu(:,i:i+num-1),uwb(:,i:i+num),dt);
%     x_LSR(:,i) = temp(1:6);
% end
% figure(3)
% [~] = plot_result(t(:,1:size(x_LSR,2)),x_LSR,gtd(:,1:size(x_LSR,2)),'OG\_based');
%% 4. plot
% figure(4)
% subplot(2,1,1)
% gdt_d = sqrt(xt(1,:).^2+xt(2,:).^2+xt(3,:).^2);
% plot(t,uwb,t,gdt_d)
% legend('|estimate_p| VS |gtd_p|')
% ylabel('distance')
% subplot(2,1,2)
% plot(t,sqrt(xt(4,:).^2+xt(5,:).^2+xt(6,:).^2),'--',t,sqrt(gtd(4,:).^2+gtd(5,:).^2+gtd(6,:).^2),'-.')  
% legend('estimate_v','gtd_v')
% title('|estimate_v| VS |gtd_v|')