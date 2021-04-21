function f = MHE_obj2(x, x0_bar, x2_minus, z, u, uwb_v, dt,gain)
% The cost function of MHE
gamma_1 = gain(1);
gamma_2 = gain(2);
gamma_3 = gain(3);
gamma_4 = gain(4);

obj = 0;
A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt,;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;];
  
  B = [0.5*dt^2, 0, 0;
     0, 0.5*dt^2, 0;
     0, 0, 0.5*dt^2;
     dt, 0,  0;
     0,  dt, 0;
     0,  0, dt;];

% use measured acc to predict x
x_bar(:, 1) = A*x0_bar + B*u(:, 1); % x2_bar
for i = 1:length(u)-1
    x_bar(:, i+1) = A*x(:, i) + B*u(:, i+1);  % predict x3_bar ~ x11_bar
end

for i = 1:length(u)
    t_obj = gamma_1 *(x_bar(:, i) - x(:, i))' * (x_bar(:, i) - x(:, i)); 
    
    x_relative = x(1:3,i);
    estimate_d = sqrt(x_relative' * x_relative);
    z_obj =  gamma_2 * (estimate_d - z(i))^2;
    
%     estimate_v = x(1:3,i)'*x(4:6,i)/estimate_d;
%     v_dis(i) = (uwb_v(i)-estimate_v)^2;   
%     v_obj = gamma_4*v_dis(i);
    
    u_lsr = u(:,2:end);
    K = length(u_lsr);
    
    t = dt*(1:K)';
    delta_v(:,1) = u_lsr(:,1)*dt;
    for i = 2:K
        delta_v(:,i) = delta_v(:,i-1) + u_lsr(:,i)*dt;
    end

    delta_p(:,1) = 0.5*dt^2*u_lsr(:,1); % the position variance rised from acc.
    for i = 2:K
        delta_p(:,i) = delta_p(:,i-1) + delta_v(:,i-1)*dt + 0.5*dt^2*u_lsr(:,i);
    end
    second_c = zeros(3,K);
    for j = 1:K
     second_c(:,j) =  2*delta_p(:,j).*t(j);  
    end
    r0 = z(:,1);
    C = [2*delta_p' second_c' 2*t t.^2];
    temp = delta_p(1,:).^2+delta_p(2,:).^2+delta_p(3,:).^2;
    b = (z(:,2:end).^2 - r0^2-temp)';
    x_aug = [x(:,1); x(1:3,1)'*x(4:6,1);norm(x(4:6,1),2)^2];
    v_obj = gamma_4 *norm(b-C*x_aug,2).^2;
    
    obj = obj + t_obj + z_obj + v_obj;    % 1st term + 2nd term + fourth term
end

obj = obj + gamma_3* (x(:,1)-x2_minus)'*(x(:,1)-x2_minus);

f = obj; 

end





