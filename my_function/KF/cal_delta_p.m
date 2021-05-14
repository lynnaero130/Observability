function delta_p = cal_delta_p(imu,dt)
% this function is used to calculate delta_p
        K = size(imu,2);
        delta_v(:,1) = imu(:,1)*dt;
        for i = 2:K
            delta_v(:,i) = delta_v(:,i-1) + imu(:,i)*dt;
        end

         delta_p(:,1) = 0.5*dt^2*imu(:,i); % the position variance rised from acc.
        for i = 2:K
            delta_p(:,i) = delta_p(:,i-1) + delta_v(:,i-1)*dt + 0.5*dt^2*imu(:,i);
        end
end

