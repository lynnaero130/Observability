function [c,ceq] = nonlinear_constraint(x,xg,rg,ru,x0,K,dt)
%The nonlinear constraint of trajectory generation.
% x(3:4,1) is the second x, the first one is x0

c1 = norm(x(3:4,end)-xg,2);
c2 = sqrt(x(1,:).^2+x(2,:).^2)-ru;
c = [c1;c2'];

% prediction process
ceq(:,1) = x0 + x(1:2,1)*dt - x(3:4,1);
for i = 2:K
   ceq(:,i) = x(3:4,i-1) + x(1:2,i)*dt - x(3:4,i);
end
ceq = reshape(ceq,1,2*K);
end

