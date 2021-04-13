%% process
clc;clear
data = load('./data/0411/OG_1.mat');
i = find(data.counter(2,:)==0);
begin_time = data.counter(1,i(2));
end_time = data.counter(1,i(2)+125);
dt = 1/25;% sampling frequency: 25Hz
K = 5/dt; %measure times
time = dt*(0:K-1);

% pos&vel
i = find((data.pos(1,:)>=begin_time)&(data.pos(1,:)<=end_time));
gtd_100 = data.pos(2:4,i); 
j = find((data.vel(1,:)>=begin_time)&(data.vel(1,:)<=end_time));
gtd_100(4:6,:) = data.vel(2:4,j); 
gtd_100(1,:) = gtd_100(1,:)+3.491; % remove the bias of VICON
gtd_100(2,:) = gtd_100(2,:) - 2.017;
gtd_100(3,:) = gtd_100(3,:)-0.04;

%uwb
k = find((data.dis(1,:)>=begin_time)&(data.dis(1,:)<=end_time));
uwb_whole = data.dis(2,k); 
k_1 = find(uwb_whole==100); % wipe out invalid value: 100
uwb_whole(k_1) = uwb_whole(k_1-1);
uwb = uwb_whole(1:2:2*K-1)+0.4;
% imu
imu = [];
gtd = [];
for a=1:K
    imu(:,a) = (gtd_100(4:6,a*4)-gtd_100(4:6,a*4-3))/0.03;
    gtd(:,a) = gtd_100(:,a*4-3);
end
figure(1)
plot(time,uwb,time,sqrt(gtd(1,:).^2+gtd(2,:).^2+gtd(3,:).^2)) % error:0.5m
legend('uwb\_dis','gtd\_dis')
figure(2)
plot(time,imu)
%% load data
lopt = 8;  % set moving horizon

% ranging measurements of ground robots
uwb1 = uwb;
uwb2 = uwb;

% acceleration
imu(3,:) = imu(3,:)- mean(imu(3,:))+0.02;
imu(2,:) = imu(2,:) - 0.07;

%------------- preprocessing-----------------%
[b1,a1] = butter(2,0.04,'low');  % butterworth filter, cutoff frequency: 0.04*25 = 1Hz
[b2,a2] = butter(4,0.2,'low'); % butterworth filter, cutoff frequency: 0.2*25 = 5Hz
[bi,ai] = butter(3,0.1,'low'); % butterworth filter, cutoff frequency: 0.1*25 = 2.5Hz

    % filter ranging measurements
    y = filtfilt(b1,a1,uwb(1:lopt+2));
    y1 = filtfilt(b1,a1,uwb1(1:lopt+2));
    y2 = filtfilt(b1,a1,uwb2(1:lopt+2));

    % radical velocity
    vy =  [0,0];
    vy1 = [0,0];
    vy2 = [0,0];
    for i = 2:lopt+2
    	vy(i) = abs(y(i)-y(i-1))/(dt);
        vy1(i) = abs(y1(i)-y1(i-1))/(dt); 
        vy2(i) = abs(y2(i)-y2(i-1))/(dt);
    end
     % filter radical velocity
    vy = filtfilt(b1,a1,vy);
    vy1 = filtfilt(b1,a1,vy1);
    vy2 = filtfilt(b1,a1,vy2);

% % filter imu
% imu(1,:) = filtfilt(b1,a1,imu(1,:));
% imu(2,:) = filtfilt(b1,a1,imu(2,:));
% imu(3,:) = filtfilt(b1,a1,imu(3,:));
% figure(3)
% plot(time,imu)
%------------- preprocessing-----------------%
%% initialize
delta = 10;
xt = gtd(:, 1:lopt+delta); % record estimation by MHE, initial
xt_imu = gtd(:, 1:lopt+delta); 
xi = xt(:, delta+2); % \hat{x}_2-, use visual estimation as initial, and then use the MHE estimate
opt_range = 1300;
X = xi;
for i = lopt+delta+1:K  % i: current time-step
    
    % calculate current ranging measurements & radical velocity, i-th
    yt = filtfilt(b1,a1,uwb(1:i));
    y(i) = yt(end);
    yt = filtfilt(b1,a1,uwb1(1:i));
    y1(i) = yt(end);
    yt = filtfilt(b1,a1,uwb2(1:i));
    y2(i) = yt(end);
    
    vy(i) = (y(i)-y(i-1))/dt;
    vy1(i) = (y1(i)-y1(i-1))/dt;
    vy2(i) = (y2(i)-y2(i-1))/dt;
    
    vy = filtfilt(b1,a1,vy);
    vy1 = filtfilt(b1,a1,vy1);
    vy2 = filtfilt(b1,a1,vy2);
    
    xi = xt(:,i-lopt+1); % estimated initial value
    disp(['Step ' , num2str(i)])

    % used for MHE, 15 points
    MHE_imu = imu(:,i-lopt+1:i);
    MHE_uwb = y(i-lopt+1:i);
    MHE_uwb1 = y1(i-lopt+1:i);
    MHE_uwb2 = y2(i-lopt+1:i);
    MHE_v = vy(i-lopt+1:i);
    MHE_v1 = vy1(i-lopt+1:i);
    MHE_v2 = vy2(i-lopt+1:i);
    
    x0 = prediction(xi, MHE_imu, dt); % use prediction as initial guess
%     x0 = [xt(i-lopt+1,i) ];
%     x_t = prediction(xt_imu(:,i-lopt), MHE_imu, dt); %??
%     xt_imu(:,i) = x_t(:,end); %�?
    options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
    X = fmincon(@(x)objmhemulti (x, xi, MHE_imu, MHE_uwb, MHE_uwb1, MHE_uwb2, MHE_v, MHE_v1, MHE_v2),x0,[],[],[],[],[],[],[],options);
    xt(:,i) = X(:,end);
    
    % post processing
    xt(1,:) = filtfilt(b2,a2,xt(1,:));
    xt(2,:) = filtfilt(b2,a2,xt(2,:));
    xt(3,:) = filtfilt(b2,a2,xt(3,:));

    xt(4,:) = filtfilt(b2,a2,xt(4,:));
    xt(5,:) = filtfilt(b2,a2,xt(5,:));
    xt(6,:) = filtfilt(b2,a2,xt(6,:));

end

return
