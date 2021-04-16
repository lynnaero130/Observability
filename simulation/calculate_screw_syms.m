syms t_s z
z = (x0(3)+xg(3))/2 +  (x0(3)-xg(3))*cos(pi*t_s/K/dt)/2;
z_dot = diff(z)
z_ddot = diff(z_dot)
z_d = subs(z, t);
z_vd = subs(z_dot, t);
z_ad = subs(z_ddot, t);
figure(1)
plot(t,z_d)
title('z')
figure(2)
plot(t,z_vd)
title('z_velocity')
figure(3)
plot(t,z_ad)
title('z_acc')
