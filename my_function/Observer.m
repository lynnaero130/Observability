function [x, xd] = Observer(imu,uwb,x0,dt)
%This function is observer.
A = [zeros(3,3) eye(3,3);
    zeros(3,3) zeros(3,3);];
B = [zeros(3,3);
    eye(3,3);];
% A = [1, 0, 0, dt, 0, 0;
%      0, 1, 0, 0, dt, 0;
%      0, 0, 1, 0, 0, dt,;
%      0, 0, 0, 1, 0, 0;
%      0, 0, 0, 0, 1, 0;
%      0, 0, 0, 0, 0, 1;];
%   
%   B = [0.5*dt^2, 0, 0;
%      0, 0.5*dt^2, 0;
%      0, 0, 0.5*dt^2;
%      dt, 0,  0;
%      0,  dt, 0;
%      0,  0, dt;];
 
 
x(:,1) =x0; %zeros(6,1);
xd(:,1) =x0; %zeros(6,1);
 
 for i = 1:size(imu,2)
    G = eye(6)+A*dt;
    H = (eye(6)*dt+0.5*A*dt^2)*B;
    xd(:,i+1) = G*xd(:,i) + H*imu(:,i) + (uwb(i) - norm(xd(1:3,i)))*[xd(1:3,i)'/norm(xd(1:3,i)) 0 0 0]';  % discrete
%     lamda = (uwb(i)-norm(x(1:3,i)))/norm(x(1:3,i));
%     a = [lamda*eye(3,3) zeros(3,3);zeros(3,6)];
%     x(:,i+1) = (G+a*dt)*x(:,i) + (H+0.5*a*(dt^2)*B)*imu(:,i);  % approximate

    lamda = (uwb(i)-norm(x(1:3,i)))/norm(x(1:3,i));
    G = [exp(lamda*dt)*eye(3) (1/lamda)*(exp(lamda*dt)-1)*eye(3);zeros(3,3) eye(3)];
    H = [(exp(lamda*dt)/lamda^2-1/lamda^2-dt/lamda)*eye(3);dt*eye(3)];
    x(:,i+1) = G*x(:,i) + H*imu(:,i); %exponential
    x(:,i+1) - xd(:,i+1)
 end


