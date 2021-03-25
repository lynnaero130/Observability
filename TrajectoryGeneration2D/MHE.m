function xt = MHE(x_real,u, z, dt,K,source)
% This function is used to accomplish MHE.
% x_real: ground truth
% x0: the second estimate in the last calculate
% u: control input
% z: observation distance
% dt: sampling time
opt_length = 10; % use ten data to estimate one position
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
delta = 1;
init_i = opt_length+delta; % 11
xt(:,1:init_i-1) = x_real(:,1:init_i-1); % xt is used to store the estimation
% xp(:,1:init_i-1) = x_real(:,1:init_i-1); 
for i = init_i:K+1
%     x_initial_guess = [x_real(:,i-opt_length+1:i);x_real(:,i-opt_length+1:i)];
    x_initial_guess = x_real(:,i-opt_length+1:i);
%     x0_bar = xt(:,i-opt_length+1); % used as the initial value of x_bar(predict)
    x0_bar = x_real(:,i-opt_length+1); % start from x_1
    z_mhe = z(:,i-opt_length+1:i); % start from z_1
    u_mhe = u(:,i-opt_length+1:i); % start from u_2
    X = fmincon(@(x)MHE_obj(x, x0_bar, x0_bar, z_mhe,u_mhe,source,dt),x_initial_guess,[],[],[],[],[],[],[],options);
%     X = fmincon(@(x)MHE_obj(x, x0_bar, x0_bar, z_mhe,source),x_initial_guess,[],[],[],[],[],[],@(x)MHE_constraint(x,u_mhe,opt_length,dt),options);
    % X(1:2,:) is estimation; X(3:4,:) is prediction
    xt(:,i) = X(1:2,end);
%     xp(:,i) = X(3:4,end); %prediction
    fprintf('Iteration = %d\n', i)
end

end

