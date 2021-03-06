function f = MHE_obj(x, x0_bar, x2_minus, z, u, source,dt)
% The cost function of MHE
gamma_1 = 1;
gamma_2 = 1;
gamma_3 = 1;

obj = 0;
A = diag([1,1]);
B = diag([dt,dt]);

x_bar(:, 1) = A*x0_bar + B*u(:, 1); % x2_bar
for i = 1:length(u)-1
    x_bar(:, i+1) = A*x(:, i) + B*u(:, i+1);  % predict x3_bar ~ x11_bar
end
for i = 1:length(u)
    t_obj = gamma_1 *(x_bar(:, i) - x(:, i))' * (x_bar(:, i) - x(:, i)); 
    
    x_relative = x(1:2,i) - source;
    estimate_d = sqrt(x_relative' * x_relative);
    z_obj =  gamma_2 * (estimate_d - z(i))^2;
    obj = obj + t_obj + z_obj;    % 1st term + 2nd term
end

% x_bar = [x0_bar x(3:4,1:end-1)]; % x predict, the final column is not used in the objective
% temp = x(1:2,:)-x_bar;
% t_obj = gamma_1 *sum(sqrt(temp(1,:).^2+temp(2,:).^2),2);
% 
% x_relative = x(1:2,:) - source;
% estimate_d = sqrt(x_relative(1,:).^2+x_relative(2,:).^2);
% z_obj = gamma_2 * sum((estimate_d - z).^2,2);
% 
% i_obj = gamma_3* (x(1:2,1)-x2_minus)'*(x(1:2,1)-x2_minus);
 
% f = t_obj + z_obj + i_obj; % 1st term + 2nd term + third term

obj = obj + gamma_3* (x(1:2,1)-x2_minus)'*(x(1:2,1)-x2_minus);

f = obj; 
end





