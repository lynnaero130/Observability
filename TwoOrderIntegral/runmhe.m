%% MHE
clc;close all
[x_hat_OG,gdt_OG,uwb_v_OG,z_measured_OG,filter_z_OG] = estimate(p_OG,v_OG,u_OG,sigma_omega,sigma_v,source,K,dt);
%plotres
figure(2)
e_OG = plot_result(t,x_hat_OG,gdt_OG,'OG\_based');

%% 
figure(4)
subplot(2,1,1)
x_relative = gdt_OG(1:3,:) - source;
gdt_d = sqrt(x_relative(1,:).^2+x_relative(2,:).^2+x_relative(3,:).^2);
plot(t,z_measured_OG,t,filter_z_OG,t,gdt_d)
legend('measure','filter','groundtruth')
ylabel('distance')
figure(4)
subplot(2,1,2)
gdt_v = sqrt(gdt_OG(4,:).^2+gdt_OG(5,:).^2+gdt_OG(6,:).^2);
plot(t,uwb_v_OG,t,gdt_v)
legend('measure','groundtruth')
ylabel('velocity')