function [z,u] = cal_real(x,x0,sigma_omega,sigma_v,K)
% This function is used to calculate the observation result z. 
global imu_noise uwb_noise
% measured accsqrt(sigma_v)*randn(1)elaration
u = x(:,1:K);
% u = u + sqrt(sigma_omega)*randn(3,K)+0.05*rand(3,K); % measured by imu, noise
u = u + imu_noise; % measured by imu, noise

% measured distance
p = [x0(1:3) x(:,K+1:2*K)]; 
for j = 1:K+1
    x_relative = p(:,j);
%     z(1,j) = sqrt(x_relative'*x_relative)+sqrt(sigma_v)*randn(1); % measured by uwb, noise
    z(1,j) = sqrt(x_relative'*x_relative)+uwb_noise; % measured by uwb, noise
end

end

