function f = OG_cond(x,x0,source)
% The cost function of trajectory generation based on OG
% 3 and 4 row denote x; 1 and 2 row denote u

temp = x(1,:).^2+x(2,:).^2;
energy = sum(temp,2); % sum by columns
% energy = 0;
x_plus = [x0 x(3:4,:)]; % K+1 dimension
x_relative = x_plus - source;
rt2 = x_relative(1,:).^2 + x_relative(2,:).^2;
OG = [sum(x_relative(1,:).^2./rt2,2) sum(x_relative(1,:).*x_relative(2,:)./rt2,2);
    sum(x_relative(1,:).*x_relative(2,:)./rt2,2) sum(x_relative(2,:).^2./rt2,2)]; % range only
% OG = [sum(x_relative(1,:).^2,2) sum(x_relative(1,:).*x_relative(2,:),2);
%     sum(x_relative(1,:).*x_relative(2,:),2) sum(x_relative(2,:).^2,2)]; % range-squared only
conditon_num = cond(OG);

obj = conditon_num + 0.3*energy;
f = obj;

