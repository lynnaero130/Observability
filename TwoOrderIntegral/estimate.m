function [x_estimate,groundtruth,uwb_v,z_measured,filter_z] = estimate(p,v,u,sigma_omega,sigma_v,source,K,dt)
%% measured data
groundtruth  = [p;v];
[z_measured, u_measured] = cal_real(p,u,sigma_omega,sigma_v,source,K);

%% filter distance and velocity
[b2,a2] = butter(2,5*dt,'low'); 
filter_z = filtfilt(b2,a2,z_measured);
uwb_v =  [0,0];
for i = 2:K+1
    uwb_v(i) = abs(filter_z(i)-filter_z(i-1))/(dt);
end

%% MHE
x_estimate = MHE(groundtruth,u_measured,uwb_v, filter_z,dt,K,source);

end

