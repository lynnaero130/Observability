function f = MHE_obj(x, x0_bar, x2_minus, z)
% The cost function of MHE
gamma_1 = 1;
gamma_2 = 1;
gamma_3 = 1;
% A = diag([1,1]);
% B = diag([dt,dt]);
% xt(:, i+1) = x0; % the initial value of x_predict is x0
% for i = 1:K
%     xt(:, i+1) = A  *  x(:, i) + B  *  u(:, i);  % predict
%     t_obj = gamma_1 *(xt(:, i) - x(:, i))' * (xt(:, i) - x(:, i)); 
%     
%     estimate_d = sqrt(x(1:2,i)' * x(1:2,i));
%     z_obj =  gamma_2 * (estimate_d - z(i))^2;
%     obj = obj + t_obj + z_obj;    % 1st term + 2nd term
% end
x_bar = [x0_bar x(3:4,1:end-1)]; % x predict, the final column is not used in the objective
temp = x(1:2,:)-x_bar;
t_obj = gamma_1 *sum(sqrt(temp(1,:).^2+temp(2,:).^2),2);
estimate_d = sqrt(x(1,:).^2+x(2,:).^2);
z_obj = gamma_2 * sum((estimate_d - z).^2,2);
i_obj = gamma_3* (x(1:2,1)-x2_minus)'*(x(1:2,1)-x2_minus);
 
f = t_obj + z_obj + i_obj; % 1st term + 2nd term + third term

end





