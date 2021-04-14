function [c,ceq] = nonlinear_constraint(x,x0,xg,rg,ru,K,dt)
%The nonlinear constraint of trajectory generation.
% c is inequality, ceq is equality
% x(:,1:K) is u_0 ~ u_(K-1); each variable has three elements.
% x(:,K+1:2K) is p_1 ~ p_K
% x(:,1:K) is v_1 ~ v_K 


% inequality constraint
X_rK = [x(:,2*K);x(:,3*K)]-xg;
c1 = sqrt(X_rK'*X_rK)-rg; % endpoint constraint
c2 = sqrt(x(1,1:K).^2+x(2,1:K).^2+x(3,1:K).^2)-ru; % control input constraint
c = [c1;c2'];

% equality constraint
% prediction process
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
X = [x(:,K+1:2*K);x(:,2*K+1:3*K)];
u = x(:,1:K);
ceq(:,1) = A*x0 + B*u(:,1) - X(:,1); % 6 x 1
for i = 2:K
   ceq(:,i) = A*X(:,i-1) + B*u(:,i) - X(:,i);
end
ceq = reshape(ceq,6*K,1);
end

