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
Q=diag([diag(sigma_omega)' diag(sigma_omega)' sigma_omega(1)*10 sigma_omega(1)]*1000000); 
Q(1,1) = 1000000;
Q(2,2) = 1000000;
Q(3,3) = 1000000;

R = sigma_v;
P{1,1} = eye(8)*10^(0);
 
%--------- noise-----------%
  
x(:,1) =[x0(1:3,1); x0(4:6,1); x0(1:3,1)'*x0(4:6,1);x0(4:6,1)'*x0(4:6,1)]; % initial estimation
xt(:,1) =[x0(1:3,1); x0(4:6,1); x0(1:3,1)'*x0(4:6,1);x0(4:6,1)'*x0(4:6,1)]; 
y = 0.5*uwb(2:end).^2 - 0.5*uwb(1).^2 + 0.5*(delta_p(1,:).^2 + delta_p(2,:).^2 +delta_p(3,:).^2);

% for i=1:K
%    xt(:,i+1)=G*xt(:,i) + H*imu(:,i);  
% end

 for i = 1:K
    
    C = [delta_p(:,i)' zeros(1,3) i*dt 0.5*(i*dt)^2];
%     if(i>1)
%     R = cov(y(1:i));
%     end
  
    % prediction
    x_ = G*x(:,i) + H*imu(:,i); 
    xt(:,i+1)=G*x(:,i) + H*imu(:,i);  
    PT=P{:,i};
    P_ = G*P{:,i}*G' + Q;
%     P_=P_;
%     if(i>100)
%         P_
%         return
%     end
    
    CT=C;  

    %update
    KK = P_*CT'/(CT*P_*CT' + R);
    if(i==100)
       KK
       C*KK
       P_
       C
    end

    dy =y(:,i) - C*x_;
    Kt=KK;

    x(:,i+1) = x_ + Kt*(y(:,i) - C*x_);

    P{:,i+1} = (eye(8) - KK*CT)*P_;

    
end

