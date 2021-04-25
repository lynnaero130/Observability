function x = KF(imu,uwb,x0,dt,sigma_omega,sigma_v)
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
 Q = diag([zeros(1,3) diag(sigma_omega)' 0 0]);
 R = sigma_v;
 P{1,1} = eye(8); 
 %--------- noise-----------%
 
x(:,1) =[x0(1:3,1); x0(4:6,1); x0(1:3,1)'*x0(4:6,1);x0(4:6,1)'*x0(4:6,1)]; % initial estimation
y = 0.5*uwb(2:end).^2 - 0.5*uwb(1).^2 + 0.5*(delta_p(1,:).^2 + delta_p(2,:).^2 +delta_p(3,:).^2);

 for i = 2:K
     
     C = [delta_p(:,i-1)' zeros(1,3) (i-1)*dt 0.5*((i-1)*dt)^2];
  
    % prediction
    x_ = G*x(:,i-1) + H*imu(:,i-1); 
    P_ = G*P{:,i-1}*G' + Q;

    %update
    Kt = P_*C'*inv(C*P_*C' + R);
    x(:,i) = x_ + Kt*(y(:,i-1) - C*x_);
    P{:,i} = (eye(8) - Kt*C)*P_;
    
 end
% disp('???')
%  for j = 1:K
%     temp(j) = P{1,j}(1,1);
% end
end

