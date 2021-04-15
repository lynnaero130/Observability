% This script is used to calculate the offset of ground truth
% Attention: gtd is the original data without offset!!
x0 = [0;0;0];
for a=1:K
    gtd_lsr(:,a) = gtd_o(:,a*4-3);  % without offset
end
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
X = fmincon(@(x)obj (x,uwb,gtd_lsr),x0,[],[],[],[],[],[],[],options)

function f = obj(x,uwb,gtd)
a = x(1);
b = x(2);
c = x(3);
temp = (uwb - sqrt((gtd(1,:)-a).^2 + (gtd(2,:)-b).^2 + (gtd(3,:)-c).^2)).^2;
f = sum(temp,2);
end


