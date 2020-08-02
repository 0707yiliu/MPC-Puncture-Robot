clear;
clc;
d2r=pi/180;  %%转化成弧度
%%%%% 转动惯量 IB
IB=[60,6.0,9.0;5.0,60,7.0;9.0,6.0,60];
%%%%% 转动惯量IB的逆invIB
invIB=inv(IB);

%%%%扰动力矩
A0=0.5e-4;
%%%%% 初始角速度Wwo
Wwo=[0.0015 0.0018 -0.0023]';

%%%（按照ZXY转动得到的Euler姿态角）[Phi Theta Psi]

%%%%  初始姿态四元数（Q00 Q0）
Q00=0.6736;  
Q1=0.2101;  
Q2=0.6325;  
Q3=-0.3194;   
Q0=[Q1 Q2 Q3];

%%%% 目标姿态角（按照ZXY转动得到的Euler姿态角）fei1=[Psi Phi Theta]
fei1=[5,5,5]'*d2r;

%%%% 目标姿态四元数Qt=[qt0 qt]
qt0=cos(fei1(1)/2)*cos(fei1(2)/2)*cos(fei1(3)/2)-sin(fei1(1)/2)*sin(fei1(2)/2)*sin(fei1(3)/2);
qt1=sin(fei1(2)/2)*cos(fei1(1)/2)*cos(fei1(3)/2)-cos(fei1(2)/2)*sin(fei1(1)/2)*sin(fei1(3)/2);
qt2=cos(fei1(2)/2)*sin(fei1(3)/2)*cos(fei1(1)/2)+sin(fei1(1)/2)*cos(fei1(3)/2)*sin(fei1(2)/2);
qt3=sin(fei1(3)/2)*sin(fei1(2)/2)*cos(fei1(1)/2)+cos(fei1(3)/2)*cos(fei1(2)/2)*sin(fei1(1)/2);
Qt=[qt0 qt1 qt2 qt3];

%%%%  初始角动量
HB0=[IB*Wwo]';

%%%% 轨道角速度（Height=300km）
Wio=[0 0.001 0]';

%%%期望角速度
Wc=[0 0 0];
%%%% 角速率陀螺
cong=[1,1,1]/3600*d2r;
Noig=[1,1,1]*1/3*5*10^(-5)*d2r;
Gyquan=[1,1,1]*1.e-10*d2r;
Tscontrol=0.05;
%%%% PD控制的控制参数Kp,Kd，Ki,均为三乘三维
%Kp=[50 0 0;0 50 0;0 0 50];
%Kd=[25 0 0;0 25 0;0 0 25];
%Ki=[1 0 0;0 1 0;0 0 1];

