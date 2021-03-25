function [X_pre, z,u] = cal_real(x,x0,K,sigma_omega,source,sigma_v,dt)
% This function is used to calculate the observation result z. 

u = x(1:2,:);
u = u + sqrt(sigma_omega)*randn(2,K); % measured by imu

X_pre(:,1) = x0;
for i = 1:K
   X_pre(:,i+1) = X_pre(:,i) + u(:,i)*dt;
end

X = [x0 x(3:4,:)]; 
for j = 1:K+1
    x_relative = X(:,j) - source;
    z(1,j) = sqrt(x_relative'*x_relative)+sqrt(sigma_v)*randn(1); % measured by uwb
end

end

