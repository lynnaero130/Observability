%% Initialize
K = 100; %measure times
x0 = [0;0;0;0;0;0]; % use prediction as initial guess
xg = [0.5;1;1.2;0;0;0]; % end point
rg = 0.01; % position tolerance
ru = 2; % input constraint
source = [0;0;0]; % the position of uwb
% dt = 0.02;
% t = dt*(0:K);
x_initial_guess = [ones(3,K) ones(3,K) zeros(3,K)];
% x_initial_guess = rand(3,3*K);
% x_initial_guess = ones(3,3*K);
%% load data
clc;clear
filename = 'data/bags/02';
gdt = load([filename '/vicon.txt'])';
gtd(1,:) = gdt(2,:) + 3.93; % remove the bias of VICON
gtd(2,:) = gdt(3,:) - 0.18;
gtd(3,:) = gdt(4,:) - 0.47;
imu = load([filename '/imu.txt'])';
imu = imu(2:4,:);
dt = 1/25;
uwb = load([filename '/uwb.txt'])';
uwb = uwb(2,:)
lll = min(size(gdt,2),size(uwb,2));
t = dt*(0:lll-1);

gtd = gtd(:,1:lll);
uwb = uwb(:,1:lll);
imu = imu(:,1:lll);

figure(1)
plot(t,uwb,t,sqrt(gtd(1,:).^2+gtd(2,:).^2+gtd(3,:).^2))
legend('uwb','gdt\_dis')
% figure(2)
% plot3(gtd(1,:),gtd(2,:),gtd(3,:))

%------------- preprocessing-----------------%
[b1,a1] = butter(2,0.04,'low');  % butterworth filter, cutoff frequency: 0.04*25 = 1Hz

% filter ranging measurements
filter_d = filtfilt(b1,a1,uwb);
uwb_v =  [0,0];
for i = 2:lll
    uwb_v(i) = abs(filter_d(i)-filter_d(i-1))/(dt);
end
uwb_v = filtfilt(b1,a1,uwb_v);   
%% MHE
% % filter distance and velocity
[b2,a2] = butter(2,5*dt,'low'); 
%MHE
% x_estimate = MHE(groundtruth,u_measured,uwb_v, z_measured,dt,K,source);
x_estimate = MHE(gtd,imu,uwb_v, filter_d,dt,lll,source);
%plotres
figure(2)
e_OG = plot_result(t,x_estimate,gtd,'zzw');
