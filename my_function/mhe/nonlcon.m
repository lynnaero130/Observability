function [c,ceq] = nonlcon(x,uwb_v)
% This function provides the inequaility of mhe.
% |radial velocity| - |velocity| <=0
% c is equality, ceq is inequality
    x_v = sqrt(x(4, :).^2+x(5, :).^2+x(6, :).^2);
    c = (-x_v + uwb_v)';
    ceq = [];
end

