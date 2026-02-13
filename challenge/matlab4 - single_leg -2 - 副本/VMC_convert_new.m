%有问题，但是不知道哪里出了问题
%定义变量
syms phi1 phi2 phi3 phi4 phi0;
syms phi1_dot phi2_dot phi3_dot phi4_dot;
syms F Tp;
%直接定义，不具有移植性
l1=0.15;l2=0.27;l3=0.27;l4=0.15;l5=0.15;

xb = l1*cos(phi1);
yb = l1*sin(phi1);
xd = l5+l4*cos(phi4);
yd = l4*sin(phi4);

A0=2*l2*(xd-xb);
B0=2*l2*(yd-yb);
C0=l2^2+(xd-xb)^2+(yd-yb)^2-l3^2;
%求phi2
phi2=2*atan((B0 + sqrt(A0^2+B0^2-C0^2))/(A0+C0));
%已知phi2求phi3
%phi3 = atan2((yb-yd+l2*sin(phi2)),(xb-xd+l2*cos(phi2)));

xc=xb+l2*cos(phi2);
yc=yb+l2*sin(phi2);

l0=sqrt((xc-l5/2)^2+yc^2);
phi0=atan2(yc,xc-l5/2);

position=[l0,phi0];
matlabFunction(position,'File','leg_position_new');
%---------------VMC求解--------
xb_dot = diff(xb,t);
yb_dot = diff(yb,t);
xc_dot = diff(xc,t);
yc_dot = diff(yc,t);
xd_dot = diff(xd,t);
yd_dot = diff(yd,t);

phi2_dot = ((xd_dot-xb_dot)*cos(phi3) + (yd_dot-yb_dot)*sin(phi3))/(l2*sin(phi3-phi2));
%变量替换
xc_dot = subs(xc_dot,diff(phi2,t),phi2_dot);
xc_dot = subs(xc_dot,[diff(phi1,t),diff(phi4,t)],[phi1_dot,phi4_dot]);
yc_dot = subs(yc_dot,diff(phi2,t),phi2_dot);
yc_dot = subs(yc_dot,[diff(phi1,t),diff(phi4,t)],[phi1_dot,phi4_dot]);

%求雅可比矩阵,有点抽象啊,不知道最后算对没有 
x_dot = [xc_dot;yc_dot];
q_dot = [phi1_dot;phi4_dot];
x_dot = simplify(collect(x_dot,q_dot));
J = simplify(jacobian(x_dot,q_dot));

R = [cos(phi0-pi/2) -sin(phi0-pi/2);
     sin(phi0-pi/2)  cos(phi0-pi/2)];
%这里M矩阵论文里没错，只是他上下方向写反了
M = [0 -1/l0;
     1 0];

P = simplify(J.'*R*M);

T = P*[F;Tp];

matlabFunction(T,'File','leg_force_new');

leg_force_new(90,0,3*pi/4,pi/4)




