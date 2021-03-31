function f = MHE_obj(x, x0_bar, x2_minus, z, u, uwb_v, source,dt)
% The cost function of MHE
gamma_1 = 1;
gamma_2 = 5;
gamma_3 = 100;
gamma_4 = 0;
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
    
    x_relative = x(1:3,i) - source;
    estimate_d = sqrt(x_relative' * x_relative);
    z_obj =  gamma_2 * (estimate_d - z(i))^2;
    
    estimate_v = sqrt(x(4:6,i)'*x(4:6,i));
    v_dis = ((uwb_v(i)-estimate_v)^2)*exp((uwb_v(i)+0.1)/(estimate_v+0.1));  
    v_obj = gamma_4*v_dis;
    obj = obj + t_obj + z_obj + v_obj;    % 1st term + 2nd term + fourth term
    
end

obj = obj + gamma_3* (x(:,1)-x2_minus)'*(x(:,1)-x2_minus);

f = obj; 
end





