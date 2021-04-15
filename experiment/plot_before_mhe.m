
figure(1)
plot(time,y,time,sqrt(gtd(1,:).^2+gtd(2,:).^2+gtd(3,:).^2))  
legend('uwb\_dis','gtd\_dis')
title('uwb VS |gdt|')

figure(2)
% [~,~,estimate_v] = cart2sph(gtd(4,:),gtd(5,:),gtd(6,:)); % It is the mochang of velocity.
for i = 1:K
    estimate_v(i) = gtd(1:3,i)'*gtd(4:6,i)/sqrt(gtd(1:3,i)' * gtd(1:3,i));
end
% plot(time,vy,time,sqrt(gtd(4,:).^2+gtd(5,:).^2+gtd(6,:).^2),'--')  
plot(time,vy,time,estimate_v,time,sqrt(gtd(4,:).^2+gtd(5,:).^2+gtd(6,:).^2),'--')  
legend('uwb_vr','real_vr','|r|')
title('radial velocity VS |velocity|')

figure(3)
subplot(3,1,1)
plot(time,imu(1,:),'r',time,imu_o(1,:),'r--')
title('filtered imu VS imu')
legend('ax','ax_o')
subplot(3,1,2)
plot(time,imu(2,:),'r',time,imu_o(2,:),'r--')
legend('ay','ay_o')
subplot(3,1,3)
plot(time,imu(3,:),'r',time,imu_o(3,:),'r--')
legend('az','az_o')

% verify acceleration integrate
delta_v(:,1) = imu(:,1)*dt;
for i = 2:K
    delta_v(:,i) = delta_v(:,i-1) + imu(:,i)*dt;
end
delta_p(:,1) = 0.5*dt^2*imu(:,1); % the position variance rised from acc.
for i = 2:K
    delta_p(:,i) = delta_p(:,i-1) + delta_v(:,i-1)*dt + 0.5*dt^2*imu(:,i);%+0.05*rand(3,1);
end
figure(4)
subplot(3,1,1)
plot(time,delta_p(1,:)+gtd(1,1),'r',time,gtd(1,:),'r--')
legend('acc_x','gtd_x')
title('imu integrate VS gtd')
subplot(3,1,2)
plot(time,delta_p(2,:)+gtd(2,1),'r-',time,gtd(2,:),'r--')
legend('acc_y','gtd_y')
subplot(3,1,3)
plot(time,delta_p(3,:)+gtd(3,1),'r-',time,gtd(3,:),'r--')
legend('acc_z','gtd_z')

