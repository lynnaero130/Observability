function f = OG(x,x0,source)
% cost function
% 3 and 4 row denote x; 1 and 2 row denote u

temp = x(1,:).^2+x(2,:).^2;
energy = sum(temp,2); % sum by columns

x_plus = [x0 x(3:4,:)];
x_relative = x_plus - source;
rt2 = x_relative(1,:).^2 + x_relative(2,:).^2;
det = sum(x_relative(1,:).^2./rt2,2)*sum(x_relative(2,:).^2./rt2,2)-sum(x_relative(1,:).*x_relative(2,:)./rt2,2)^2;

obj = det + 0.3*energy;
f = obj;

