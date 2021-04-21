%This scipt is used to calculate linear MHE. 
% Have not been finished.
clc
lopt = 8;
start = 10;
x_hat = gtd(:,start-1:start+lopt-2);
u = imu(:,start:start+lopt-1);
d = uwb(start:start+lopt-1);


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
 
W = zeros((lopt-1)*7);
 Z = zeros((lopt-1)*7,1);
 M = zeros((lopt-1)*7,6*lopt);
for i = 1:lopt-1
    W((i-1)*7+1:7*i,(i-1)*7+1:7*i) = diag([gain(1)*ones(1,6) gain(2)]);
     Z(7*(i-1)+1:7*i-1,1) = B*(u(:,end-i+1)-u(:,end-i));
     Z(7*i) = d(end-i+1)-d(end-i);
     M(7*(i-1)+1:7*(i-1)+6,(i-1)*6+1:(i+1)*6) = [eye(6,6) A];
     H = x_hat(1:3,end-i+1)'/sqrt(x_hat(1,end-i+1)^2+x_hat(2,end-i+1)^2+x_hat(3,end-i+1)^2);
     M(7*i,(i-1)*6+1:i*6) = [H 0 0 0]; 
     
end

X_lsr = inv(M'*W*M)*M'*W*Z;
x_hat = [x_hat x_hat(:,end)+X_lsr(1:6,1)];