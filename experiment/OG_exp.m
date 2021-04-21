%% 1. clear workspace
clc
clearvars -except dt K time gain imu_noise
%% 2. load data & preprocess
name = 'OG';
filename = ['./data/0411/' name '_1.mat'];
[gtd,gtd_o,imu,imu_o,y,vy] = preprocess(filename,dt,K,imu_noise);
%% 3. plot before mhe
plot_before_mhe(time,gtd,imu,imu_o,y,vy,dt,K) % run the script
%% 4.1 MHE
clc
xt = MHE(gtd,imu,y,vy,dt,K-1,gain);
%plotres
figure(5)
[~]  = plot_result(time,xt,gtd,name);

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
clc;
x_LSR = [];
num = 60;
for i = 1:K-num
    temp = estimate_LSR(imu(:,i:i+num-1),y(:,i:i+num),dt);
    x_LSR(:,i) = temp(1:6);
end
figure(6)
[~] = plot_result(time(:,1:size(x_LSR,2)),x_LSR,gtd(:,1:size(x_LSR,2)),'OG\_based');
%% 5. plot estimated result
% close all
% figure(4)
% error_norm = plot_result(time(1:length(xt)),xt,gtd(:,1:length(xt)),name);
% figure(5)
% plot(time,vy,time,sqrt(xt(4,:).^2+xt(5,:).^2+xt(6,:).^2),'--',time,sqrt(gtd(4,:).^2+gtd(5,:).^2+gtd(6,:).^2),'-.')  
% legend('uwb_v','estimate_v','gtd_v')
% title('radial velocity VS |velocity|')

