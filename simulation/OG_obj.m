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
 second_c(:,j) =  2*delta_p(:,j).*t(j);  
end

C = [2*delta_p' second_c' 2*t t.^2];
% obj = -abs(det(C'*C));
% a(:,1) = svd(C'*C)
% a(:,2) = svd(C)
eee = eigs(C'*C,8);
% eee = svd(C'*C);
obj = max(eee)/(min(eee)+0.01);

temp = x(1,1:K).^2+x(2,1:K).^2 + x(3,1:K).^2;
energy = sum(temp,2); % sum by columns

f = obj+ 0*energy;

