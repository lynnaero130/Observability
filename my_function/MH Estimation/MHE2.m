function xt = MHE2(gtd,imu, uwb, uwb_v, dt,K,gain)
% with radial velocity
% gtd: ground truth
% imu: control input
% uwb: observation distance
% dt: sampling time
% gain: gamma1~gamma4

opt_length = 9; % use ten data to estimate one position
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
delta = 2;
init_i = opt_length+delta; % 11
xt(:,1:init_i) = gtd(:,1:init_i); % xt is used to store the estimation
y = uwb;
v = uwb_v;

for i = init_i:K
    % calculate current ranging measurements & radical velocity, i-th 
    x_initial_guess = xt(:,i-opt_length+1:i);
    x0_bar = xt(:,i-opt_length+1); % start from x_1, used to predict x2_bar
    x2_minus = xt(:,i-opt_length+2); % start from x_2
    z_mhe = y(:,i-opt_length+2:i+1); % start from z_2
    v_mhe = v(:,i-opt_length+2:i+1); % start from v_2
    u_mhe = imu(:,i-opt_length+1:i); % start from u_1

    % use u_1 ~ u_10 & z_1 ~ z_10 to estimate x2 ~ x11
%     X = fmincon(@(x)MHE_obj(x, x0_bar, x2_minus, z_mhe,u_mhe,v_mhe,dt,gain),x_initial_guess,[],[],[],[],[],[],[],options);
    X = fmincon(@(x)MHE_obj2(x, x0_bar, x2_minus, z_mhe,u_mhe,v_mhe,dt,gain),x_initial_guess,[],[],[],[],[],[],[],options);
%     X = fmincon(@(x)MHE_obj(x, x0_bar, x2_minus, z_mhe,u_mhe,v_mhe,source,dt,gain),x_initial_guess,[],[],[],[],[],[],@(x)nonlcon(x,v_mhe),options);
    % X(1:3,:) is position; X(4:6,:) is velocity.
    xt(:,i+1) = X(1:6,end);
    fprintf('Iteration = %d\n', i)
    
end

end

