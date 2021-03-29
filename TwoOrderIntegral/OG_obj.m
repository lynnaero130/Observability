function f = OG_obj(x,dt,K)
% This is the cost of tajectory generation.
% K denote the measure times
% x(:,1:K) is u_0 ~ u_(K-1); each variable has three elements.
% x(:,K+1:2K) is p_1 ~ p_K
% x(:,2k+1:3K) is v_1 ~ v_K

t = dt*(1:K)';
delta_v(:,1) = x(:,1)*dt;
for i = 2:K
    delta_v(:,i) = delta_v(:,i-1) + x(:,i)*dt;
end

delta_p(:,1) = 0.5*dt^2*x(:,1); % the position variance rised from acc.
for i = 2:K
    delta_p(:,i) = delta_p(:,i-1) + delta_v(:,i-1)*dt + 0.5*dt^2*x(:,i);%+0.05*rand(3,1);
end

second_c = zeros(3,K);
for j = 1:K
 second_c(:,i) =  2*delta_p(:,i).*t(i);  
end

C = [2*delta_p' second_c' 2*t t.^2];
% obj = -abs(det(C'*C));
% obj = cond(C'*C);
obj = 1/(trace(C'*C)+0.01);
f = obj;

