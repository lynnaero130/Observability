clc;clear
syms dt x1 y1 z1 r1 x2 y2 z2 r2 x3 y3 z3 r3
A = [1, 0, 0, dt, 0, 0;
     0, 1, 0, 0, dt, 0;
     0, 0, 1, 0, 0, dt,;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;];
 Aa = [A zeros(6,6);
  eye(6,6) zeros(6,6)];
%  H = [x/r y/r z/r 0 0 0];
 Hh = [x1/r1 y1/r1 z1/r1 0 0 0 zeros(1,6);
     zeros(3) eye(3)./dt zeros(3) -eye(3)./dt]
 Hh2 = [x2/r2 y2/r2 z2/r2 0 0 0 zeros(1,6);
     zeros(3) eye(3)./dt zeros(3) -eye(3)./dt]
Hh2 = [x2/r2 y2/r2 z2/r2 0 0 0 zeros(1,6);
     zeros(3) eye(3)./dt zeros(3) -eye(3)./dt]
 OG = transpose(Hh*Aa)*Hh*Aa;
 OG = OG + transpose(Hh2*Aa^2)*Hh2*Aa^2;
%  for i = 1:5
% %     temp = transpose(H*A^i)*H*A^i;
%     temp = transpose(Hh*Aa^i)*Hh*Aa^i;
%     OG = OG + temp;
%  end
OG;
det(OG)

%% 
clc;clear
syms t;
syms u1(t) u2(t) u3(t) y(t)
% u = sym('u',[3 1])
u = [u1;u2;u3]
A = [zeros(3) -eye(3) zeros(3) zeros(3,5);
    zeros(3) zeros(3) eye(3)  zeros(3,5);
    zeros(3,14);
    zeros(1,10) -1/y zeros(1,3);
    transpose(u) zeros(1,8) 1 0 0;
    zeros(1,3) -2*transpose(u) zeros(1,6) -3 0;
    zeros(1,6) transpose(u) zeros(1,4) 1;
    zeros(1,14);
    ];
% phi = eye(14)+int(A,'t')+int(A*int(A,'t'),'t')+int(A*int(A*int(A,'t'),'t'),'t')+int(A*int(A*int(A*int(A,'t'),'t'),'t'),'t')phi = eye(14)+int(A,'t')+int(A*int(A,'t'),'t')+int(A*int(A*int(A,'t'),'t'),'t')+int(A*int(A*int(A*int(A,'t'),'t'),'t'),'t')
H = [zeros(1,9) 1 zeros(1,4)];
OG = transpose(H*A)*H*A;
% OG = 0;
%  for i = 1:5
%     temp = transpose(H*A^i)*H*A^i;
%     OG = OG + temp;
%  end
% OG
det(OG)