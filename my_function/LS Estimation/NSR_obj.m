function f = NSR_obj(x, C, b)
%NSR_obj is used to estimate x by nonlinear least square.

err = C*x-b;
f = err'*err;

end






