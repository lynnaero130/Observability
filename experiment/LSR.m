% This script is used to calculate the offset of ground truth
x0 = [0;0;0];
options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',200000);
X = fmincon(@(x)obj (x,uwb,gtd),x0,[],[],[],[],[],[],[],options)

function f = obj(x,uwb,gtd)
a = x(1);
b = x(2);
c = x(3);
temp = (uwb - sqrt((gtd(1,:)-a).^2 + (gtd(2,:)-b).^2 + (gtd(3,:)-c).^2)).^2;
f = sum(temp,2);
end


