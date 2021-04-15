%% 1. load & Initialize
clc;clear
addpath(genpath('../my_function'));
dt = 1/25;% sampling frequency: 25Hz
K = 5/dt; %measure times
time = dt*(0:K-1);
% mhe gain
gain = [1 1 1 1];
%% 2. load data & preprocess
name = 'OG';
filename = ['./data/0411/' name '_1.mat'];
[gtd_OG,gtd_o,imu_OG,imu_o,y_OG,vy_OG] = preprocess(filename,dt,K);
%% 3. plot before mhe
plot_before_mhe(time,gtd_OG,imu_OG,imu_o,y_OG,vy_OG,dt,K) % run the script
%% 4.1 MHE
clc
xt_OG = MHE(gtd_OG,imu_OG,y_OG,vy_OG,dt,K-1,gain);
%plotres
figure(5)
[~]  = plot_result(time,xt_OG,gtd_OG,name);

%% 4.2 velocity compensation
% temp = (y - sqrt(xt(1,:).^2+xt(2,:).^2+xt(3,:).^2));
% for i = 1:K-5
%     part_a(i) = temp(i+5) - temp(i)
% end
% part_a(K-4:K) = temp(K-4:K);
% part_b = xt(1:3,:)./sqrt(xt(1,:).^2+xt(2,:).^2+xt(3,:).^2);
% delta_x = part_b.*part_a;
% delta_v = delta_x/5/dt;
% for i = 1:K
%    new_v(:,i) =sum(delta_v(:,1:i),2);
% end
% xt(4:6,:) = xt(4:6,:) + new_v;
%% 4.3 LSR to estimate x
% clc;
% x_LSR = [];
% num = 15;
% for i = 1:K-num
%     temp = estimate_LSR(imu(:,i:i+num-1),y(:,i:i+num),dt);
%     x_LSR(:,i) = temp(1:6);
% end
% xt = x_LSR;

%% 5. plot estimated result
% close all
% figure(4)
% error_norm = plot_result(time(1:length(xt)),xt,gtd(:,1:length(xt)),name);
% figure(5)
% plot(time,vy,time,sqrt(xt(4,:).^2+xt(5,:).^2+xt(6,:).^2),'--',time,sqrt(gtd(4,:).^2+gtd(5,:).^2+gtd(6,:).^2),'-.')  
% legend('uwb_v','estimate_v','gtd_v')
% title('radial velocity VS |velocity|')

