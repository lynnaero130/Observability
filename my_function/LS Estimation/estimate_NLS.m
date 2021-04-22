function x = estimate_NLS(imu,uwb,vy,dt)
% Estimate the initial p0,v0
% Nonlinear Least Square, with nonlinear constraint
% use fmincon.

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
% ra = t(K)./t;

r0 = uwb(:,1);
C = [2*delta_p' second_c' 2*t t.^2];
temp = delta_p(1,:).^2+delta_p(2,:).^2+delta_p(3,:).^2;
b = (uwb(:,2:end).^2 -temp)';
% Row balance, to improve the illneass
% D = diag(1./max(abs(C),[],2));
% C = D*C;
% b = D*b;

x_initial_guess = ones(8,1);
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
x = fmincon(@(x)NSR_obj(x, C, b),x_initial_guess,[],[],[],[],[],[],@(x)NSR_cons(x,r0),options);


