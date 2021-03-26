function f = cal_cov(x,x0,K,sigma_omega,sigma_x0,source,sigma_v,dt)
% This function is used to calculate the covariance P_t.
X_OG_real(:,1) = x0 + x(1:2,1)*dt;
for i = 2:K
   X_OG_real(:,i) = X_OG_real(:,i-1) + x(1:2,i)*dt+[sqrt(sigma_omega(1,1))*randn(1);sqrt(sigma_omega(2,2))*randn(1)];
end
Pt_plus = sigma_x0;
tr(1) = trace(Pt_plus);
for i = 1:K
    Pt_minus = Pt_plus + sigma_omega;
    x_relative = X_OG_real(:,i) - source;
    Ht = x_relative'./sqrt(x_relative'*x_relative); % range only
%     Ht = x_relative'; % range-squared only
    St = Ht*Pt_minus*Ht'+sigma_v;
    Pt_plus = (eye(2)-Pt_minus*Ht'*inv(St)*Ht)*Pt_minus;
    tr(i+1) = trace(Pt_plus);
end
f = tr;
end

