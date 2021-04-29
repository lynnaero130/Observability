function [x,xt] = KF2(imu,uwb,x0,dt,sigma_omega,sigma_v)
%This function is KF2, moving horizon, reset C and y and x_estimate.
A =[zeros(3,3) eye(3) zeros(3,2)
    zeros(5,8)];
B = [zeros(3,3)
  eye(3)
  zeros(2,3)];
G = eye(8)+A*dt;
H = (eye(8)*dt+0.5*A*dt^2)*B;

%---------- use to calculate delta_p----------%
K = size(imu,2);
%--------- noise-----------%
% Q=diag([diag(sigma_omega)' diag(sigma_omega)' sigma_omega(1)*10 sigma_omega(1)]*1000000); 
% Q(1,1) = 1000000;
% Q(2,2) = 1000000;
% Q(3,3) = 1000000;
Q = H*sigma_omega*H';

R = sigma_v;
P{1,1} = diag([0.01 0.01 0.01 0.01 0.01 0.005 1e-10 1e-10]);
% P{1,1} = diag([0.01 0.01 0.01 0.01*ones(1,3) 1e-5 1e-5])*100;
%--------- noise-----------%
  
x(:,1) =[x0(1:3,1); x0(4:6,1); x0(1:3,1)'*x0(4:6,1);x0(4:6,1)'*x0(4:6,1)]; % initial estimation
xt(:,1) =[x0(1:3,1); x0(4:6,1); x0(1:3,1)'*x0(4:6,1);x0(4:6,1)'*x0(4:6,1)]; 
for i=1:K
    xt(:,i+1)=G*xt(:,i) + H*imu(:,i); 
end

period = 1000;
iter = period;
 for i = 1:K
     iter = iter +1;
    if mod(i,period)==1
        if (K-i)>2*period
            iter = iter-period;
            delta_p = cal_delta_p(imu(:,i:i+period),dt);
            y = 0.5*uwb(i+1:i+1+period).^2 - 0.5*uwb(i).^2 + 0.5*(delta_p(1,:).^2 + delta_p(2,:).^2 +delta_p(3,:).^2);
            x(7:8,i) = [x(1:3,i)'*x(4:6,i); x(4:6,i)'*x(4:6,i)];
            P{:,i} = P{1,1}; 
        else
            iter = iter-period;
            delta_p = cal_delta_p(imu(:,i:end),dt);
            y = 0.5*uwb(i+1:end).^2 - 0.5*uwb(i).^2 + 0.5*(delta_p(1,:).^2 + delta_p(2,:).^2 +delta_p(3,:).^2);
            x(7:8,i) = [x(1:3,i)'*x(4:6,i); x(4:6,i)'*x(4:6,i)];
        end
    end

    C = [delta_p(:,iter)' zeros(1,3) iter*dt 0.5*(iter*dt)^2];
  
    % prediction
    x_ = G*x(:,i) + H*imu(:,i); 
%     xt(:,i+1)=G*x(:,i) + H*imu(:,i);  
    P_ = G*P{:,i}*G' + Q;
    
    CT=C;  

    %update
    KK = P_*CT'/(CT*P_*CT' + R);

    Kt=KK;

    x(:,i+1) = x_ + Kt*(y(:,iter) - C*x_);

    P{:,i+1} = (eye(8) - KK*CT)*P_;

    
 end
 figure
 t = dt*(0:K);
 for a = 1:(K+1)
     Px(a) = P{1,a}(1,1);
     Py(a) = P{1,a}(2,2);
     Pz(a) = P{1,a}(3,3);
 end
 subplot(3,1,1)
 plot(t,Px)
 legend('x')
subplot(3,1,2)
 plot(t,Py)
 legend('y')
 subplot(3,1,3)
 plot(t,Pz)
 legend('z')

