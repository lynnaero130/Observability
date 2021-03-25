function f = TLQG(x,K,source,sigma_x0,sigma_omega,sigma_v)
% The cost function of trajectory generation based on Cov
% 3 and 4 row denote x; 1 and 2 row denote u
temp = x(1,:).^2+x(2,:).^2;
energy = sum(temp,2); % sum by columns
% energy = 0;

Pt_plus = sigma_x0;
tr = 0;
for i = 1:K
    Pt_minus = Pt_plus + sigma_omega;
    x_relative = x(3:4,i) - source;
    Ht = x_relative'./sqrt(x_relative'*x_relative); % range only
%     Ht = x_relative'; % range-squared only 
    St = Ht*Pt_minus*Ht'+sigma_v;
    Pt_plus = (eye(2)-Pt_minus*Ht'*inv(St)*Ht)*Pt_minus;
    tr = tr + trace(Pt_plus);
end
obj = tr + 0.3*energy;
f = obj;

