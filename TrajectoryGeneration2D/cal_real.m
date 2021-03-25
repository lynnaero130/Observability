function [X_real, z,u] = cal_real(x,x0,K,sigma_omega,source,sigma_v,dt)
% This function is used to calculate the observation result z. 
X_real(:,1) = x0 + x(1:2,1)*dt;
for i = 2:K
   X_real(:,i) = X_real(:,i-1) + x(1:2,i)*dt+[sqrt(sigma_omega(1,1))*randn(1);sqrt(sigma_omega(2,2))*randn(1)];
end

for j = 1:K
    x_relative = X_real(:,i) - source;
    z(1,j) = sqrt(x_relative'*x_relative)+sqrt(sigma_v)*randn(1);
end
u = x(3:4,:);
end

