function [gtd,gtd_o,imu,imu_o,y,vy] = preprocess(filename,dt,K,imu_noise)
% This function is used to load experimental data and preprocess them. :
% gtd_o: without offset
% imu_o: without filter
[b1,a1] = butter(2,0.04,'low');  % butterworth filter, cutoff frequency: 0.04*25 = 1Hz
[b3,a3] = butter(2,0.2,'low');  % butterworth filter, cutoff frequency: 0.2*25 = 5Hz

data = load(filename);
i = find(data.counter(2,:)==0); % The first zero denotes hovering.
begin_time = data.counter(1,i(2)); 
end_time = data.counter(1,i(2)+125);
%-------------------------------------vicon--------------------------------------%
i = find((data.pos(1,:)>=begin_time)&(data.pos(1,:)<=end_time));
gtd_o = data.pos(2:4,i); 
j = find((data.vel(1,:)>=begin_time)&(data.vel(1,:)<=end_time));
gtd_o(4:6,:) = data.vel(2:4,j); 
gtd_100 = gtd_o;
% gtd_100(1,:) = gtd_100(1,:); % remove the bias of VICON
% gtd_100(2,:) = gtd_100(2,:);
% gtd_100(3,:) = gtd_100(3,:);
gtd_100(1,:) = gtd_o(1,:)+3.3068; % remove the bias of VICON
gtd_100(2,:) = gtd_o(2,:) -1.4229;
gtd_100(3,:) = gtd_o(3,:) -0.5397;
for a=1:K
    gtd(:,a) = gtd_100(:,a*4-3);
end
%-------------------------------------imu--------------------------------------%
imu = [];
imu_tag = 1; % 0 denotes three order deviation; 1 denotes four order deviation, accurate
if imu_tag == 0
    for a=1:K
    imu(:,a) = (gtd_o(4:6,a*4)-gtd_o(4:6,a*4-3))/0.03; 
    end
else
    for a=1:K-1
    imu(:,a) = (gtd_o(4:6,a*4+1)-gtd_o(4:6,a*4-3))/0.04;
    end
    imu(:,K) = (gtd_o(4:6,K*4)-gtd_o(4:6,a*4-3))/0.03;
    imu = imu + imu_noise;
end
imu_o = imu; % store the imu before filter
    %filter imu
    imu(1,:) = filtfilt(b3,a3,imu_o(1,:));
    imu(2,:) = filtfilt(b3,a3,imu_o(2,:));
    imu(3,:) = filtfilt(b3,a3,imu_o(3,:));   

%-------------------------------------uwb--------------------------------------%
k = find((data.dis(1,:)>=begin_time)&(data.dis(1,:)<=end_time));
uwb_whole = data.dis(2,k); 
uwb = uwb_whole(1:2:2*K-1);
gdt_r = sqrt(gtd(1,:).^2+gtd(2,:).^2+gtd(3,:).^2); 
for j = 1:length(uwb)  % wipe out invalid value: 100
    if (uwb(j)==100)&(j~=1)
        uwb(j)=uwb(j-1);
    elseif (uwb(j)==100)&(j==1)
        uwb(j)=gdt_r(1);
    else
        uwb(j)=uwb(j);
    end
end
    % filter uwb-distance
    %     y = filtfilt(b1,a1,uwb(1:lopt+2));
        y = filtfilt(b1,a1,uwb);
% radial velocity
vy =  [0,0];
 for i = 2:K
%     	vy(i) = abs(y(i)-y(i-1))/(dt);
    vy(i) = (y(i)-y(i-1))/(dt);
end
     % filter uwb-velocity
    vy = filtfilt(b1,a1,vy);
    
end

