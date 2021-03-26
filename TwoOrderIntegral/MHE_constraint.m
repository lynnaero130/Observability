function [c,ceq] = MHE_constraint(x,u,opt_length,dt)
%The nonlinear constraint of MHE.
A = diag([1,1]);
B = diag([dt,dt]);
% prediction process
for i = 1:opt_length
   ceq(:,i) = A*x(1:2,i) + B*u(:, i) - x(3:4,i);
end
c = [];
end

