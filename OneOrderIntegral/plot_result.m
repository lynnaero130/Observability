function error_norm = plot_result(t,estimate,pre,groundtruth,z,source,graph_title)
% estimate: estimated result by MHE
% real: real position
% plan: planned position, can be derived by prediction model when giving
% control input

subplot(3,1,1)
plot(estimate(1,:),estimate(2,:),'b--',groundtruth(1,:),groundtruth(2,:),'b-')
legend('x_{hat}','x_g')
ylabel('Trajectory')
title(graph_title)
subplot(3,1,2)
plot(t,estimate(1,:),'r--',t,groundtruth(1,:),'r-')
hold on
plot(t,estimate(2,:),'b--',t,groundtruth(2,:),'b-')
legend('x_{hat}','x_g','y_{hat}','y_g')
ylabel('Position')
subplot(3,1,3)
process_error = pre - groundtruth; % imu measured error
process_error_norm = sqrt(process_error(1,:).^2 + process_error(2,:).^2);

position_r = groundtruth - source;
d = sqrt(position_r(1,:).^2+position_r(2,:).^2); % real distance
uwb_error = z - d; % uwb measured error

error = groundtruth-estimate;
error_norm = sqrt(error(1,:).^2 + error(2,:).^2);
plot(t,error_norm,'k',t,process_error_norm,'r',t,uwb_error,'b')
ylabel('estimation error')
legend('error','imu\_error','uwb\_error')

end

