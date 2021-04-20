function f = MHE_obj(x, x0_bar, x2_minus, z, u, uwb_v, dt,gain)
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


x_relative = x(1:3,:);
estimate_d(1) = sqrt(x_relative(:,1)' * x_relative(:,1));
for i = 1 :length(u)-1
    estimate_d(i+1) = estimate_d(i) + x_relative(:,i)'*(x_relative(:,i+1) - x_relative(:,i))/sqrt(x_relative(:,i)' * x_relative(:,i));
end
temp = estimate_d;

for i = 1:length(u)
    t_obj = gamma_1 *(x_bar(:, i) - x(:, i))' * (x_bar(:, i) - x(:, i)); 
    
%     x_relative = x(1:3,i);
%     estimate_d = sqrt(x_relative' * x_relative);

    estimate_d = temp(i);
    z_obj =  gamma_2 * (estimate_d - z(i))^2;
  
    estimate_v = x(1:3,i)'*x(4:6,i)/estimate_d;
    v_dis(i) = (uwb_v(i)-estimate_v)^2;   
    v_obj = gamma_4*v_dis(i);
    obj = obj + t_obj + z_obj + v_obj;    % 1st term + 2nd term + fourth term
end

obj = obj + gamma_3* (x(:,1)-x2_minus)'*(x(:,1)-x2_minus);

f = obj; 

end





