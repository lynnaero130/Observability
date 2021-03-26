function xt = MHE(gdt,u, z, dt,K,source)
% This function is used to accomplish MHE.
% gdt: ground truth
% x0: the second estimate in the last calculate
% u: control input
% z: observation distance
% dt: sampling time
opt_length = 10; % use ten data to estimate one position
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
delta = 1;
init_i = opt_length+delta; % 11
xt(:,1:init_i) = gdt(:,1:init_i); % xt is used to store the estimation

for i = init_i:K
    x_initial_guess = gdt(:,i-opt_length+2:i+1);
    x0_bar = xt(:,i-opt_length+1); % start from x_1, used to predict x2_bar
    x2_minus = xt(:,i-opt_length+2); % start from x_2
    z_mhe = z(:,i-opt_length+2:i+1); % start from z_2
    u_mhe = u(:,i-opt_length+1:i); % start from u_1
    % use u_1 ~ u_10 & z_1 ~ z_10 to estimate x2 ~ x11
    X = fmincon(@(x)MHE_obj(x, x0_bar, x2_minus, z_mhe,u_mhe,source,dt),x_initial_guess,[],[],[],[],[],[],[],options);
    xt(:,i+1) = X(1:2,end);
    fprintf('Iteration = %d\n', i)
end

end

