function [z,u] = cal_real(x,x0,sigma_omega,sigma_v,K,dt)
% This function is used to calculate the observation result z. 
global imu_noise uwb_noise
% measured accsqrt(sigma_v)*randn(1)elaration
u = x(:,1:K);
% u = u + sqrt(sigma_omega)*randn(3,K)+0.05*rand(3,K); % measured by imu, noise
u = u + imu_noise; % measured by imu, noise

A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt,;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;];
  
  B = [0.5*dt^2, 0, 0;
     0, 0.5*dt^2, 0;
     0, 0, 0.5*dt^2;
     dt, 0,  0;
     0,  dt, 0;
     0,  0, dt;];
 
 x_relative(1:3,1)=x0(1:3);
 x_relative(4:6,1)=x(:,2*K+1);
z(1,1)=sqrt(x_relative(:,1)'*x_relative(:,1));

% measured distance
% p = [x0(1:3) x(:,K+1:2*K)]; 
for j = 1:K
    x_relative(:,j+1) = A*x_relative(:,j)+B*u(:,j);
%     z(1,j) = sqrt(x_relative'*x_relative)+sqrt(sigma_v)*randn(1); % measured by uwb, noise
    z(1,j+1) = sqrt(x_relative(:,j+1)'*x_relative(:,j+1)) ; % measured by uwb, noise
end

z=z + uwb_noise;


end

