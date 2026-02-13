%传统的位置运动学正解算

%变量
syms phi1 phi2 phi3 phi4;
syms dphi1 dphi4;
l1=0.15;l2=0.27;l3=0.27;l4=0.15;l5=0.15;

syms xc yc xd yd xb yb;
%变量之间对应的关系
xb=l1*cos(phi1);
yb=l1*sin(phi1);
xd=l5+l4*cos(phi4);
yd=l4*sin(phi4);

A0=2*l2*(xd-xb);
B0=2*l2*(yd-yb);
C0=l2^2+(xd-xb)^2+(yd-yb)^2-l3^2;
phi2=2*atan((B0 + sqrt(A0^2+B0^2-C0^2))/(A0+C0));

xc=xb+l2*cos(phi2);
yc=yb+l2*sin(phi2);

l0=sqrt((xc-l5/2)^2+yc^2);
phi0=atan2(yc,xc-l5/2);

position=[l0,phi0];
matlabFunction(position,'File','leg_position_old');

%l0=f1(phi1,phi4)
%phi0=f2(phi1,phi4)

J11=diff(l0,phi1);
J12=diff(l0,phi4);
J21=diff(phi0,phi1);
J22=diff(phi0,phi4);
JacobianMatrix=[J11 J12;J21 J22];

%空间到末端执行器
speed = JacobianMatrix*[dphi1;dphi4];
matlabFunction(speed,'File','leg_speed_old');

syms F Tp;
T = JacobianMatrix'*[F;Tp];
matlabFunction(T,'File','leg_force_old');

leg_force_old(10,0,3*pi/4,pi/4)
