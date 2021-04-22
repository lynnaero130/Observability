function [c,ceq] = NSR_cons(x,r0)
%NSR constraint
% c is equality, ceq is inequality
%     c = [];
%     ceq = norm(x(1:3))^2 - r0^2;
     c(1) = (norm(x(1:3)) - r0)^2-0.05; % dis
     c(2) = (x(1:3)'*x(4:6)-x(7))^2 - 0.01;% p0'v0
     c(3) = (x(4:6)'*x(4:6) - x(8))^2 - 0.01;% p0'v0
     ceq = [];
end






