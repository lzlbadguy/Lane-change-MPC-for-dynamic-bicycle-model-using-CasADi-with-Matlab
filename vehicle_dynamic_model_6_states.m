function x_up = vehicle_dynamic_model_6_states(x,u,Ts)
% x(1)-vx longtudinal speed
% x(2)-vy lateral speed
% x(3)-psi_dot yaw rate
% x(4)-psi yaw angle
% x(5)-X longitudinal position
% x(6)-Y lateral position
% u(1)-ax longitudinal acceleration
% u(2)- delta_f lateral acceleration
lf=1.2;
lr=1.6;
m = 1575;
Iz = 2875;
Cf_0 = -25000;
Cr_0 = -50000;

aopt_f = 20*pi/180;
aopt_r = 11*pi/180;

Fymax_f = Cf_0*aopt_f/2;
Fymax_r = Cr_0*aopt_r/2;

af = u(2) - (x(2)+lf*x(3))./x(1);
ar = -(x(2)-lr*x(3))./x(1);


Cf = Fymax_f*2*aopt_f./(aopt_f.^2+af.^2);
Cr = Fymax_r*2*aopt_r./(aopt_r.^2+ar.^2);


Fcf = -Cf.*af;
Fcr = -Cr.*ar;

f1 = x(3).*x(2) + u(1);
f2 = -x(3).*x(1) + 2/m*(Fcf.*cos(u(2))+Fcr);
f3 = 2/Iz*(lf.*Fcf-lr.*Fcr);
f4 = x(3);
f5 = x(1).*cos(x(4))-x(2).*sin(x(4));
f6 = x(1).*sin(x(4))+x(2).*cos(x(4));

xdot=[f1;f2;f3;f4;f5;f6];

x_up=x+xdot*Ts;
end

