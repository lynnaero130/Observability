% This script is used to compare the linear approximation and the real one.
clc;clear
syms x y r r_a
r = sqrt(x^2+y^2);
diff(r,y,2)
first_x = diff(r,x,1);
first_y = diff(r,y,1);
dt = 0.1;
t = 0:dt:1;
x_res = sin(t);
y_res = cos(t);
r_res = subs(r,{x,y},{x_res,y_res});
r_taylor = subs(r,{x,y},{x_res(1),y_res(1)})+subs(diff(r,x,1),{x,y},{x_res(1),y_res(1)})*(x_res-x_res(1))+subs(diff(r,y,1),{x,y},{x_res(1),y_res(1)})*(y_res-y_res(1));
r_iter(1) = subs(r,{x,y},{x_res(1),y_res(1)});
for i = 1:length(t)-1
%     r_iter(i+1) = r_iter(i) + subs(diff(r,x,1),{x,y},{x_res(i),y_res(i)})*(x_res(i+1)-x_res(i)) + subs(diff(r,y,1),{x,y},{x_res(i),y_res(i)})*(y_res(i+1)-y_res(i));
    r_iter(i+1) = r_iter(i) + x_res(i)*(x_res(i+1)-x_res(i))/sqrt(x_res(i)^2+y_res(i)^2) + y_res(i)*(y_res(i+1)-y_res(i))/sqrt(x_res(i)^2+y_res(i)^2);
end

%% 
plot3(x_res,y_res,r_res,x_res,y_res,r_taylor,x_res,y_res,eval(r_iter),'--r')
hold on
plot3(x_res(1),y_res(1),r_res(1),'og')
xlabel('x')
ylabel('y')
zlabel('z')
legend('real','origin approximate','discrete iteration')