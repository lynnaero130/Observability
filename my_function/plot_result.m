function error_norm = plot_result(t,estimate,groundtruth,graph_title)
% estimate: estimated result by MHE
% groundtruth: observed by vicon
set(gcf,'Position',[100,20,600,600]);
subplot(4,1,1)
plot3(estimate(1,:),estimate(2,:),estimate(3,:),'b--',groundtruth(1,:),groundtruth(2,:),groundtruth(3,:),'b-','linewidth', 1)
legend('x_{hat}','x_g')
ylabel('Trajectory')
title(graph_title)
subplot(4,1,2)
% plot(t,groundtruth(1,:)-groundtruth(1,1),'r-',t,groundtruth(2,:)-groundtruth(2,1),'m-',t,groundtruth(3,:)-groundtruth(3,1),'b-','linewidth', 1)
plot(t,groundtruth(1,:),'r-',t,groundtruth(2,:),'m-',t,groundtruth(3,:),'b-','linewidth', 1)
hold on
% plot(t,estimate(1,:)-groundtruth(1,1),'r--',t,estimate(2,:)-groundtruth(2,1),'m--',t,estimate(3,:)-groundtruth(3,1),'b--','linewidth', 1)
plot(t,estimate(1,:),'r--',t,estimate(2,:),'m--',t,estimate(3,:),'b--','linewidth', 1)
hl = legend({'x_g','y_g','z_g','x_{hat}','y_{hat}','z_{hat}'},'Location','northwest');
ylabel('Position')
set(hl,'Orientation','horizon','Box','on');
subplot(4,1,3)
plot(t,groundtruth(4,:),'r-',t,groundtruth(5,:),'m-',t,groundtruth(6,:),'b-','linewidth', 1)
hold on
plot(t,estimate(4,:),'r--',t,estimate(5,:),'m--',t,estimate(6,:),'b--','linewidth', 1)
hl = legend({'x_g','y_g','z_g','x_{hat}','y_{hat}','z_{hat}'},'Location','northwest');
ylabel('Velocity')
set(hl,'Orientation','horizon','Box','on');
subplot(4,1,4)
error = groundtruth-estimate;
% error_norm = sqrt(error(1,:).^2 + error(2,:).^2+ error(3,:).^2+ error(4,:).^2+ error(5,:).^2+ error(6,:).^2);
error_norm = sqrt(error(1,:).^2 + error(2,:).^2+ error(3,:).^2);
plot(t,error_norm,'k','linewidth', 1)
xlabel('Time (sec)')
ylabel('Estimation error (m)')
legend('error')

end

