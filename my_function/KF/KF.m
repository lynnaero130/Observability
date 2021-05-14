function [x,xt] = KF(imu,uwb,x0,dt,sigma_omega,sigma_v)
%This function is KF.
A =[zeros(3,3) eye(3) zeros(3,2)
    zeros(5,8)];
B = [zeros(3,3)
  eye(3)
  zeros(2,3)];
G = eye(8)+A*dt;
H = (eye(8)*dt+0.5*A*dt^2)*B;

%---------- use to calculate delta_p----------%
K = size(imu,2);
delta_v(:,1) = imu(:,1)*dt;
for i = 2:K
    delta_v(:,i) = delta_v(:,i-1) + imu(:,i)*dt;
end

 delta_p(:,1) = 0.5*dt^2*imu(:,1); % the position variance rised from acc.
for i = 2:K
    delta_p(:,i) = delta_p(:,i-1) + delta_v(:,i-1)*dt + 0.5*dt^2*imu(:,i);
end
%--------- noise-----------%
% Q = H*sigma_omega*H';
% Q(7,7) = 1e-12;
% Q(8,8) = 1e-12;
Q = diag(1e-3*dt*ones(1,6),1e-12,1e-12).^2;
Q(7,7) = 1e-12;
Q(8,8) = 1e-12;


R = 100;
P{1,1} = diag([1 1 1 0.1 0.1 0.1 1 1]);
% P{1,1} = diag([0.01 0.01 0.01 0.01 0.01 0.005 1e-10 1e-10]);
% P{1,1} = diag([0.01 0.01 0.01 0.01*ones(1,3) 1e-5 1e-5])*100;
%--------- noise-----------%
  
x(:,1) =[x0(1:3,1); x0(4:6,1); x0(1:3,1)'*x0(4:6,1);x0(4:6,1)'*x0(4:6,1)]; % initial estimation
xt(:,1) =[x0(1:3,1); x0(4:6,1); x0(1:3,1)'*x0(4:6,1);x0(4:6,1)'*x0(4:6,1)]; 
y = 0.5*uwb(2:end).^2 - 0.5*uwb(1).^2 + 0.5*(delta_p(1,:).^2 + delta_p(2,:).^2 +delta_p(3,:).^2);

 for i = 1:K
    
    C = [delta_p(:,i)' zeros(1,3) i*dt 0.5*(i*dt)^2];
  
    % prediction
    x_ = G*x(:,i) + H*imu(:,i); 
    xt(:,i+1)=G*x(:,i) + H*imu(:,i);  
    P_ = G*P{:,i}*G' + Q;
    
    CT=C;  

    %update
    KK = P_*CT'/(CT*P_*CT' + R);

    Kt=KK;

    x(:,i+1) = x_ + Kt*(y(:,i) - C*x_);

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

