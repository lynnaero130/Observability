function [z,u] = cal_real(p,u,sigma_omega,sigma_v,source,K)
% This function is used to calculate the observation result z. 

% measured accelaration
u = u + sqrt(sigma_omega)*randn(3,length(u))+0.05*rand(3,length(u)); % measured by imu, noise

% measured distance
for j = 1:K+1
    x_relative = p(:,j) - source;
    z(1,j) = sqrt(x_relative'*x_relative)+sqrt(sigma_v)*randn(1); % measured by uwb, noise
end

end

