function error_norm = plot_result(t,estimate,groundtruth,graph_title)
% estimate: estimated result by MHE
% groundtruth: observed by vicon

subplot(3,1,1)
plot3(estimate(1,:),estimate(2,:),estimate(3,:),'b--',groundtruth(1,:),groundtruth(2,:),groundtruth(3,:),'b-','linewidth',1)
legend('x_{hat}','x_g')
ylabel('Trajectory')
title(graph_title)
subplot(3,1,2)
plot(t,groundtruth(1,:),'r-',t,groundtruth(2,:),'b-',t,groundtruth(3,:),'g-')
hold on
plot(t,estimate(1,:),'r--',t,estimate(2,:),'b--',t,estimate(3,:),'g--')
hl = legend({'x_g','y_g','z_g','x_{hat}','y_{hat}','z_{hat}'},'Location','northwest');
ylabel('Position')
set(hl,'Orientation','horizon','Box','on');
subplot(3,1,3)
error = groundtruth-estimate;
error_norm = sqrt(error(1,:).^2 + error(2,:).^2+ error(3,:).^2);
plot(t,error_norm,'k')
ylabel('estimation error')
legend('error')

end

