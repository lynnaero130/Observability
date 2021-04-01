function xt = MHE(gdt,u, uwb_v, z, dt,K,source)
% This function is used to accomplish MHE.
% gdt: ground truth
% x0: the second estimate in the last calculate
% u: control input
% z: observation distance
% dt: sampling time

opt_length = 15; % use ten data to estimate one position
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
delta = 2;
init_i = opt_length+delta; % 11
xt(:,1:init_i) = gdt(:,1:init_i); % xt is used to store the estimation
%% filter
[b2,a2] = butter(4,10*dt,'low'); % butterworth filter, cutoff frequency: 5*dt/dt = 5Hz
% filter ranging measurements
y = z;
v = uwb_v;

for i = init_i:K
    % calculate current ranging measurements & radical velocity, i-th 
    x_initial_guess = xt(:,i-opt_length+1:i);
    x0_bar = xt(:,i-opt_length+1); % start from x_1, used to predict x2_bar
    x2_minus = xt(:,i-opt_length+2); % start from x_2
    z_mhe = y(:,i-opt_length+2:i+1); % start from z_2
    v_mhe = v(:,i-opt_length+2:i+1); % start from v_2
    u_mhe = u(:,i-opt_length+1:i); % start from u_1

    % use u_1 ~ u_10 & z_1 ~ z_10 to estimate x2 ~ x11
    X = fmincon(@(x)MHE_obj(x, x0_bar, x2_minus, z_mhe,u_mhe,v_mhe,source,dt),x_initial_guess,[],[],[],[],[],[],[],options);
    % X(1:3,:) is position; X(4:6,:) is velocity.
    xt(:,i+1) = X(1:6,end);
    fprintf('Iteration = %d\n', i)
    
end

    % post processing
    xt(1,:) = filtfilt(b2,a2,xt(1,:));
    xt(2,:) = filtfilt(b2,a2,xt(2,:));
    xt(3,:) = filtfilt(b2,a2,xt(3,:));

    xt(4,:) = filtfilt(b2,a2,xt(4,:));
    xt(5,:) = filtfilt(b2,a2,xt(5,:));
    xt(6,:) = filtfilt(b2,a2,xt(6,:));

end

