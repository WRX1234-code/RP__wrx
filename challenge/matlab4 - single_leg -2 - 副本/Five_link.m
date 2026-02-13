
L0s=0.11:0.01:0.38;%L0的变化范围
%L0s=0.06:0.01:0.15;
Ks=zeros(2,6,length(L0s));%存放不同腿长下的K

for step = 1:length(L0s)%循环求出不同腿长下的k
%符号变量
syms theta theta1 theta2;
syms x x1 x2;
syms phi phi1 phi2;
syms T Tp N P Nm Pm Nf t;

%机械参数,重新针对建模，增加可重复性
R=0.058;L=L0s(step)/2;
Lm=L0s(step)/2;
l=0.02;
l_m = 0.035;%机体距离建系中心的距离

%len = 0.45;h = 0.14;w = 0.36;我发现误用了一次length后会一直报错
%初始仿真用：mw=1.1;mp=1.44;M=20.0;每次改完记得在建模里也改
ml1 = 0.3616;ml2 = 0.692384;ml3 = 0.771;ml4 =  0.3616;
mw=0.55;
mp= 0.96; %ml1+ml2+ml3+ml4;
M=20;

%质心计算
O1 = l_m;
O2 = -((L+Lm)/2 - l);
O3 = -((L+Lm)-l);
O = (M*O1 + mp*O2 + mw*O3)/(M+mw+mp);

Iw = (1/2)*mw*R^2;
Ip = (1/12)*mp*(L+Lm)^2 + mp*(O2-O)^2;
Im = (1/12)*M*(0.4824^2+0.165^2) + M*(O1-O)^2;


%Iw=0.00198;
%Ip=0.021168;
%Im=0.202083;
g=9.8;



% 进行物理计算
        Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
        Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
        N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
        P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));
        
        %二阶导数求解
        equ1=x2-(T-N*R)/(Iw/R+mw*R);% =0求解
        equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;
        equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;
        [x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);
        
        % 求得雅克比矩阵，然后得到状态空间方程
        Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
        Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
        A=vpa(subs(Ja,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
        B=vpa(subs(Jb,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
        
        % 离散化
        [G,H]=c2d(eval(A),eval(B),0.001);



 %Q = diag([500 40 80 1 3500 1]);
 %R = diag([2.8 0.25]);

 Q = diag([500 30 20 1 2300 1]);
 R = diag([2.8 0.23]);

 %Q = diag([500 40 80 1 3000 1]);
 %R = diag([2.2 0.23]);

%求解K
%这一步是保持之前定义状态，同时每个腿长有不同的k
Ks(:,:,step) = dlqr(G,H,Q,R);

end



syms L0_sym;
K_fitted = sym('K', [2, 6]);  % 符号矩阵，用于存储多项式方程

P_all_1 = zeros(12, 4);
count = 1;

% 对 K 的每个元素进行循环 (2x6)
for i = 1:2
    for j = 1:6
        % 提取不同 L0 下的 K_ij 数据
        K_ij_values = reshape(Ks(i, j, :), 1, length(L0s));
        
        % 对数据进行三次多项式拟合
        p = polyfit(L0s, K_ij_values, 3);  % 三次多项式
        
        % 将多项式系数转换为符号多项式，以 L0_sym 为变量
        K_fitted(i, j) = p(1)*L0_sym^3 + p(2)*L0_sym^2 + p(3)*L0_sym + p(4);

        P_all_1(count,:) = p;
        count = count+1;
    end
end

% 显示拟合的符号矩阵 K_fitted
%disp(K_fitted);
matlabFunction(K_fitted,'File','lqr_k');
%Ks(:,:,11)
format long g
format short
Y = round(lqr_k(0.20),4)
%disp(round(P_all_1,4));
