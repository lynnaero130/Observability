function [x_estimate,groundtruth] = Estimation(x,x0,source,sigma_omega,sigma_v,dt,K)
% cal_real + MHE
temp = [x(:,K+1:2*K);x(:,2*K+1:3*K)];
groundtruth  = [x0 temp];

[z_measured, u_measured] = cal_real(x,x0,sigma_omega,sigma_v,source,K);

% frequency analysis
% frequency_analysis(z_measured,dt)


% radical velocity
uwb_v =  [0,0];
for i = 2:K+1
    uwb_v(i) = abs(z_measured(i)-z_measured(i-1))/(dt);
end

% frequency analysis
% frequency_analysis(uwb_v,dt)
x_estimate = MHE(groundtruth,u_measured,uwb_v, z_measured,dt,K,source);
end

