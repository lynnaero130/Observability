function x = estimate_LSR(imu,uwb,dt)
% This function use LSR to estimate the initial p0,v0
K = size(imu,2);
t = dt*(1:K)';
delta_v(:,1) = imu(:,1)*dt;
for i = 2:K
    delta_v(:,i) = delta_v(:,i-1) + imu(:,i)*dt;
end

delta_p(:,1) = 0.5*dt^2*imu(:,1); % the position variance rised from acc.
for i = 2:K
    delta_p(:,i) = delta_p(:,i-1) + delta_v(:,i-1)*dt + 0.5*dt^2*imu(:,i);
end

second_c = zeros(3,K);
for j = 1:K
 second_c(:,j) =  2*delta_p(:,j).*t(j);  
end
r0 = uwb(:,1);
C = [2*delta_p' second_c' 2*t t.^2];
temp = delta_p(1,:).^2+delta_p(2,:).^2+delta_p(3,:).^2;
b = (uwb(:,2:end).^2 - r0^2-temp)';

x = inv(C'*C)*C'*b;
% x = my_TLS(C,b)


