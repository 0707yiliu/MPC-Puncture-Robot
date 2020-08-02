clear;
clc;
d2r=pi/180;  %%ת���ɻ���
%%%%% ת������ IB
IB=[60,6.0,9.0;5.0,60,7.0;9.0,6.0,60];
%%%%% ת������IB����invIB
invIB=inv(IB);

%%%%�Ŷ�����
A0=0.5e-4;
%%%%% ��ʼ���ٶ�Wwo
Wwo=[0.0015 0.0018 -0.0023]';

%%%������ZXYת���õ���Euler��̬�ǣ�[Phi Theta Psi]

%%%%  ��ʼ��̬��Ԫ����Q00 Q0��
Q00=0.6736;  
Q1=0.2101;  
Q2=0.6325;  
Q3=-0.3194;   
Q0=[Q1 Q2 Q3];

%%%% Ŀ����̬�ǣ�����ZXYת���õ���Euler��̬�ǣ�fei1=[Psi Phi Theta]
fei1=[5,5,5]'*d2r;

%%%% Ŀ����̬��Ԫ��Qt=[qt0 qt]
qt0=cos(fei1(1)/2)*cos(fei1(2)/2)*cos(fei1(3)/2)-sin(fei1(1)/2)*sin(fei1(2)/2)*sin(fei1(3)/2);
qt1=sin(fei1(2)/2)*cos(fei1(1)/2)*cos(fei1(3)/2)-cos(fei1(2)/2)*sin(fei1(1)/2)*sin(fei1(3)/2);
qt2=cos(fei1(2)/2)*sin(fei1(3)/2)*cos(fei1(1)/2)+sin(fei1(1)/2)*cos(fei1(3)/2)*sin(fei1(2)/2);
qt3=sin(fei1(3)/2)*sin(fei1(2)/2)*cos(fei1(1)/2)+cos(fei1(3)/2)*cos(fei1(2)/2)*sin(fei1(1)/2);
Qt=[qt0 qt1 qt2 qt3];

%%%%  ��ʼ�Ƕ���
HB0=[IB*Wwo]';

%%%% ������ٶȣ�Height=300km��
Wio=[0 0.001 0]';

%%%�������ٶ�
Wc=[0 0 0];
%%%% ����������
cong=[1,1,1]/3600*d2r;
Noig=[1,1,1]*1/3*5*10^(-5)*d2r;
Gyquan=[1,1,1]*1.e-10*d2r;
Tscontrol=0.05;
%%%% PD���ƵĿ��Ʋ���Kp,Kd��Ki,��Ϊ������ά
%Kp=[50 0 0;0 50 0;0 0 50];
%Kd=[25 0 0;0 25 0;0 0 25];
%Ki=[1 0 0;0 1 0;0 0 1];

