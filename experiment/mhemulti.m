%% 1. load & Initialize
clc;clear
name = 'OG';
data = load(['./data/0411/' name '_1.mat']);
i = find(data.counter(2,:)==0);
begin_time = data.counter(1,i(2));
end_time = data.counter(1,i(2)+125);
dt = 1/25;% sampling frequency: 25Hz
K = 5/dt; %measure ti5mes
time = dt*(0:K-1);
[b1,a1] = butter(2,0.04,'low');  % butterworth filter, cutoff frequency: 0.04*25 = 1Hz
[b2,a2] = butter(4,0.2,'low'); % butterworth filter, cutoff frequency: 0.2*25 = 5Hz
[b3,a3] = butter(2,0.2,'low');  % butterworth filter, cutoff frequency: 0.04*25 = 1Hz
lopt = 10;  % set moving horizon

%% 2. preprocess
clc;
%-------------------------------------vicon--------------------------------------%
i = find((data.pos(1,:)>=begin_time)&(data.pos(1,:)<=end_time));
gtd_100 = data.pos(2:4,i); 
j = find((data.vel(1,:)>=begin_time)&(data.vel(1,:)<=end_time));
gtd_100(4:6,:) = data.vel(2:4,j); 
% gtd_100(1,:) = gtd_100(1,:); % remove the bias of VICON
% gtd_100(2,:) = gtd_100(2,:);
% gtd_100(3,:) = gtd_100(3,:);
gtd_100(1,:) = gtd_100(1,:)+3.3068; % remove the bias of VICON
gtd_100(2,:) = gtd_100(2,:) -1.4229;
gtd_100(3,:) = gtd_100(3,:) -0.5397;
for a=1:K
    gtd(:,a) = gtd_100(:,a*4-3);
end

%-------------------------------------imu--------------------------------------%
imu = [];
imu_tag = 0; % 0 denotes three order deviation; 1 denotes four order deviation, accurate
if imu_tag == 0
    for a=1:K
    imu(:,a) = (gtd_100(4:6,a*4)-gtd_100(4:6,a*4-3))/0.03; 
    end
else
    for a=1:K-1
    imu(:,a) = (gtd_100(4:6,a*4+1)-gtd_100(4:6,a*4-3))/0.04;
    end
    imu(:,K) = (gtd_100(4:6,K*4)-gtd_100(4:6,a*4-3))/0.03;
    imu = imu + sqrt(0.01)*randn(3,K);
end
imu_o = imu; % store the imu before filter
    %filter imu
    imu(1,:) = filtfilt(b3,a3,imu_o(1,:));
    imu(2,:) = filtfilt(b3,a3,imu_o(2,:));
    imu(3,:) = filtfilt(b3,a3,imu_o(3,:));   

%-------------------------------------uwb--------------------------------------%
k = find((data.dis(1,:)>=begin_time)&(data.dis(1,:)<=end_time));
uwb_whole = data.dis(2,k); 
uwb_tag = 0;
if uwb_tag == 0
     k_1 = find(uwb_whole==100); 
    uwb_whole(k_1) = uwb_whole(k_1-1);
    uwb = uwb_whole(1:2:2*K-1);
else
    uwb = uwb_whole(1:2:2*K-1);
    k_1 = find(uwb==100); % wipe out invalid value: 100
    gdt_v = sqrt(gtd(1,:).^2+gtd(2,:).^2+gtd(3,:).^2);
    uwb(k_1) = gdt_v(k_1);
end
uwb1 = uwb;
uwb2 = uwb;
    % filter uwb-distance
    %     y = filtfilt(b1,a1,uwb(1:lopt+2));
    %     y1 = filtfilt(b1,a1,uwb1(1:lopt+2));
    %     y2 = filtfilt(b1,a1,uwb2(1:lopt+2));
        y = filtfilt(b1,a1,uwb);
        y1 = filtfilt(b1,a1,uwb1);
        y2 = filtfilt(b1,a1,uwb2);
% radial velocity
vy =  [0,0];
vy1 = [0,0];
vy2 = [0,0];
 for i = 2:K
%     	vy(i) = abs(y(i)-y(i-1))/(dt);
%         vy1(i) = abs(y1(i)-y1(i-1))/(dt); 
%         vy2(i) = abs(y2(i)-y2(i-1))/(dt);
    vy(i) = (y(i)-y(i-1))/(dt);
    vy1(i) = (y1(i)-y1(i-1))/(dt); 
    vy2(i) = (y2(i)-y2(i-1))/(dt);
end
     % filter uwb-velocity
    vy = filtfilt(b1,a1,vy);
    vy1 = filtfilt(b1,a1,vy1);
    vy2 = filtfilt(b1,a1,vy2);
%% 3. plot before mhe
plot_before_mhe % run the script
%% 4.1 MHE
delta = 10;
xt = gtd(:, 1:lopt+delta); % record estimation by MHE, initial
xt_imu = gtd(:, 1:lopt+delta); 
xi = xt(:, delta+2); % \hat{x}_2-, use visual estimation as initial, and then use the MHE estimate
opt_range = 1300;
X = xi;
for i = lopt+delta+1:K  % i: current time-step
    
    % calculate current ranging measurements & radical velocity, i-th
%     yt = filtfilt(b1,a1,uwb(1:i));
%     y(i) = yt(end);
%     yt = filtfilt(b1,a1,uwb1(1:i));
%     y1(i) = yt(end);
%     yt = filtfilt(b1,a1,uwb2(1:i));
%     y2(i) = yt(end);
%     
%     vy(i) = (y(i)-y(i-1))/dt;
%     vy1(i) = (y1(i)-y1(i-1))/dt;
%     vy2(i) = (y2(i)-y2(i-1))/dt;
%     
%     vy = filtfilt(b1,a1,vy);
%     vy1 = filtfilt(b1,a1,vy1);
%     vy2 = filtfilt(b1,a1,vy2);
    
    xi = xt(:,i-lopt+1); % estimated initial value
    disp(['Step ', num2str(i)])

    % used for MHE, 15 points
    MHE_imu = imu(:,i-lopt+1:i);
    MHE_uwb = y(i-lopt+1:i);
    MHE_uwb1 = y1(i-lopt+1:i);
    MHE_uwb2 = y2(i-lopt+1:i);
    MHE_v = vy(i-lopt+1:i);
    MHE_v1 = vy1(i-lopt+1:i);
    MHE_v2 = vy2(i-lopt+1:i);
    
%     x0 = prediction(xi, MHE_imu, dt); % use prediction as initial guess
    x0 = xt(:,i-lopt-1-5:i-1-1-5);
%     x_t = prediction(xt_imu(:,i-lopt), MHE_imu, dt); %??
%     xt_imu(:,i) = x_t(:,end); 
    options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
    X = fmincon(@(x)objmhemulti (x, xi, MHE_imu, MHE_uwb, MHE_uwb1, MHE_uwb2, MHE_v, MHE_v1, MHE_v2),x0,[],[],[],[],[],[],[],options);
%     X = fmincon(@(x)objmhemulti (x, xi, MHE_imu, MHE_uwb, MHE_uwb1, MHE_uwb2, MHE_v, MHE_v1, MHE_v2),x0,[],[],[],[],[],[],@(x)nonlcon(x,MHE_v),options);
    xt(:,i) = X(:,end);
    
    % post processing
%     xt(1,:) = filtfilt(b2,a2,xt(1,:));
%     xt(2,:) = filtfilt(b2,a2,xt(2,:));
%     xt(3,:) = filtfilt(b2,a2,xt(3,:));
% 
%     xt(4,:) = filtfilt(b2,a2,xt(4,:));
%     xt(5,:) = filtfilt(b2,a2,xt(5,:));
%     xt(6,:) = filtfilt(b2,a2,xt(6,:));

end
    xt(1,:) = filtfilt(b2,a2,xt(1,:));
    xt(2,:) = filtfilt(b2,a2,xt(2,:));
    xt(3,:) = filtfilt(b2,a2,xt(3,:));

    xt(4,:) = filtfilt(b2,a2,xt(4,:));
    xt(5,:) = filtfilt(b2,a2,xt(5,:));
    xt(6,:) = filtfilt(b2,a2,xt(6,:));

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
num = 15;
for i = 1:K-num
    temp = estimate_LSR(imu(:,i:i+num-1),y(:,i:i+num),dt);
    x_LSR(:,i) = temp(1:6);
end
xt = x_LSR;

%% 5. plot estimated result
close all
figure(4)
error_norm = plot_result(time(1:length(xt)),xt,gtd(:,1:length(xt)),name);
figure(5)
plot(time,vy,time,sqrt(xt(4,:).^2+xt(5,:).^2+xt(6,:).^2),'--',time,sqrt(gtd(4,:).^2+gtd(5,:).^2+gtd(6,:).^2),'-.')  
legend('uwb_v','estimate_v','gtd_v')
title('radial velocity VS |velocity|')

