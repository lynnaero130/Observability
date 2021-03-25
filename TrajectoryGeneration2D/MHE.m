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
for i = init_i:K 
%     x_initial_guess = [xt(:,i-opt_length+1:i-1) x_real(:,i); xt(:,i-opt_length+1:i-1) x_real(:,i)]; % used as initial guess of fmincon
    x_initial_guess = [x_real(:,i-opt_length+1:i);x_real(:,i-opt_length+1:i)];
    x0_bar = xt(:,i-opt_length+1); % used as the initial value of x_bar(predict)
%     x2_minus = xt(:,i-opt_length+2);
    z_mhe = z(:,i-opt_length+1:i);
    u_mhe = u(:,i-opt_length+1:i);
    X = fmincon(@(x)MHE_obj(x, x0_bar, x0_bar, z_mhe,source),x_initial_guess,[],[],[],[],[],[],@(x)MHE_constraint(x,u_mhe,opt_length,dt),options);
    xt(:,i) = X(1:2,end);
    fprintf('Iteration = %d\n', i)
end

end

