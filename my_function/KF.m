function x = KF(imu,uwb,x0,dt,sigma_omega,sigma_v,gtd)
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
%  Q = diag([zeros(1,3) diag(sigma_omega)' 0 0]);
Q = diag(0.001*[ones(1,8)]);
 R = 0;
% % R = diag([0.001 0.001 0.001]);
 P{1,1} =1e-4*eye(8); 
% Q=diag([diag(sigma_omega)'*20 diag(sigma_omega)'*100 1000 100]); 
% R = sigma_v;
% P{1,1} = eye(8)*10000;
 %--------- noise-----------%
 
x(:,1) =[x0(1:3,1); x0(4:6,1); x0(1:3,1)'*x0(4:6,1);x0(4:6,1)'*x0(4:6,1)]; % initial estimation
y = 0.5*uwb(2:end).^2 - 0.5*uwb(1).^2 + 0.5*(delta_p(1,:).^2 + delta_p(2,:).^2 +delta_p(3,:).^2); % output
% y = x_measured;

 for i = 2:K+1
     
     C = [delta_p(:,i-1)' zeros(1,3) (i-1)*dt 0.5*((i-1)*dt)^2];s
     temp(i-1,:) = C;
%      C = [eye(3) zeros(3,5)];
    % prediction
    x_ = G*x(:,i-1) + H*imu(:,i-1); 
    P_ = G*P{:,i-1}*G' + Q; %8¡Á8

    %update
    Kt = P_*C'*inv(C*P_*C' + R); %8¡Á1 the larger R is, kt ->1, trust uwb 
    x(:,i) = x_ + Kt*(y(:,i-1) - C*x_); %x1
%     x(:,i) = x_;
    P{:,i} = (eye(8) - Kt*C)*P_;
    
end

end
% xx(:,1) = x(:,1) ;
% for i = 2:K+1
%     xx(:,i) = G*xx(:,i-1) + H*imu(:,i-1); 
% end

% t = 0:dt:K*dt;
% for i = 1:K
%     yy(i) = temp(i,:)*[gtd(:,i+1);x0(1:3,1)'*x0(4:6,1);x0(4:6,1)'*x0(4:6,1)];
% end

