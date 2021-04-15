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