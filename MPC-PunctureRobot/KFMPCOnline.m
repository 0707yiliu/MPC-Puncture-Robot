function [sys,x0,str,ts,simStateCompliance] = KFMPC(t,x,u,flag)
%������ 
%1.��Ҫͨ��KalmanFilter���߱�ʶϵͳģ�Ͳ�����Ϊ��ǰMPC��ģ�ͣ���Ҫ��������У�
%��ǰģ�Ͳ���A1 A2 B��12������ǰ״̬��X���ٶ��Լ����ٶȹ�8����������20������
%2.���߱�ʶϵͳģ�ͣ�����Ҫ����ģ�Ͳ�����������8������
%�����������ĸ������
%                 sys�������ĳ���Ӻ������ص�ֵ
%                 x0Ϊ����״̬�ĳ�ʼ������
%                 str�Ǳ�������������һ���վ���
%                 Ts����ϵͳ����ʱ��
%�������ĸ�����ֱ�Ϊ����ʱ��t��״̬x������u�ͷ������̿��Ʊ�־����flag
%����������滹���Խ���һϵ�еĸ�������simStateCompliance
switch flag,
  case 0,
      [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1,
    sys=mdlDerivatives(t,x,u);
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u);
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);
  case 9,
    sys=mdlTerminate(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
 
end
%����������
%�����Ǹ����Ӻ������������ص�����
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
%��ʼ���ص��Ӻ���
%�ṩ״̬�����롢���������ʱ����Ŀ�ͳ�ʼ״̬��ֵ
%��ʼ���׶Σ���־����flag���ȱ���Ϊ0��S-function�״α�����ʱ
%���Ӻ������ȱ����ã���ΪS-functionģ���ṩ������Ϣ
%���Ӻ����������
sizes = simsizes;
%����sizes���ݽṹ����Ϣ������������
sizes.NumContStates  = 0;
%����״̬����ȱʡΪ0
sizes.NumDiscStates  = 0;
%��ɢ״̬����ȱʡΪ0
sizes.NumOutputs     = 4;
%���������ȱʡΪ0
sizes.NumInputs      = 12 + 8 + 4;
%ģ�Ͳ��� + ״̬���� + Ŀ��켣
%���������ȱʡΪ0
sizes.DirFeedthrough = 1;
%�Ƿ����ֱ��ͨ����1���ڣ�0������
sizes.NumSampleTimes = 1;
%����ʱ�������������һ��
sys = simsizes(sizes);
%����size���ݽṹ����������Ϣ
x0  = [];
%���ó�ʼ״̬
str = [];
%���������ÿ�
ts  = [0.01 0];
%���ò���ʱ��
global U param;
param = zeros(12,1);
U = [0;0;0;0];
simStateCompliance = 'UnknownSimState';
function sys=mdlDerivatives(t,x,u)
%���㵼���ص��Ӻ���
%����t,x,u��������״̬�ĵ����������ڴ˸���ϵͳ������״̬����
%���Ӻ������Բ�����
sys = [];
%sys��ʾ״̬��������dx
function sys=mdlUpdate(t,x,u)
%״̬���»ص��Ӻ���
%����t��x��u������ɢ״̬�ĸ���
%ÿ�����沽�ڱ�Ȼ���ø��Ӻ����������Ƿ�������
%�����ڴ�����ϵͳ����ɢ״̬�����⣬�������ڴ��������ÿ�����沽�ڶ�����ִ�еĴ���
sys = x;
%sys��ʾ��һ����ɢ״̬����x(k+1)
function sys=mdlOutputs(t,x,u)
%��������ص�����
%����t,x,u��������������ڴ�����ϵͳ���������
%���Ӻ����������
global U param;
Nx = 8; %״̬��������ÿ��������ٶȷ����ͼ��ٶȷ���
Nu = 4; %������������ÿ������ĽǶ�
Np = 40; %Ԥ�ⲽ��
Nc = 20; %���Ʋ���
Row = 0.1; %�ɳ�����
kesi = zeros(Nx + Nu, 1);
for i  = 1:1:Nx
    kesi(i) = u(i+12); %״̬��
end
kesi(9) = U(1); %��������ֵ��С
kesi(10) = U(2);
kesi(11) = U(3);
kesi(12) = U(4);
T = 0.01;
param = u(1:12);
%1-4��A1
%5-8��A2
%9-12:B
u_piao = zeros(Nx,Nu);
Q = 0.1*eye(Nx * Np, Nx * Np);
R = 0.1*eye(Nu * Nc);
a = [0 1 0 0 0 0 0 0;
     -param(5) -param(1) 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0;
     0 0 -param(6) -param(2) 0 0 0 0;
     0 0 0 0 0 1 0 0;
     0 0 0 0 -param(7) -param(3) 0 0;
     0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 -param(8) -param(4)]; %8x8
b = [0 0 0 0;
     param(9) 0 0 0;
     0 0 0 0;
     0 param(10) 0 0;
     0 0 0 0;
     0 0 param(11) 0;
     0 0 0 0;
     0 0 0 param(12)]; % 8x4
A_cell = cell(2,2);
B_cell = cell(2,1);
A_cell{1,1} = a;
A_cell{1,2} = b;
A_cell{2,1} = zeros(Nu,Nx);
A_cell{2,2} = eye(Nu); %A_cell : 8x12 + 4x12
B_cell{1,1} = b;
B_cell{2,1} = eye(Nu); %B_cell : 12x4
A = cell2mat(A_cell);%12x12
B = cell2mat(B_cell);%12x4
C = [0 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0];%8x12 ��������ϵ�����޹�,��Ϊ�������
PHI_cell=cell(Np,1);
THETA_cell=cell(Np,Nc);
for j=1:1:Np
    PHI_cell{j,1}=C*A^j;%8x12
    for k=1:1:Nc
        if k<=j
            THETA_cell{j,k}=C*A^(j-k)*B;%8x4
        else 
            THETA_cell{j,k}=zeros(Nx,Nu);%8x4
        end
    end
end
PHI=cell2mat(PHI_cell);%size(PHI)=[Nx*Np Nx+Nu]
THETA=cell2mat(THETA_cell);%size(THETA)=[Nx*Np Nu*(Nc+1)]
H_cell=cell(2,2);
H_cell{1,1}=THETA'*Q*THETA+R;
H_cell{1,2}=zeros(Nu*Nc,1);
H_cell{2,1}=zeros(1,Nu*Nc);
H_cell{2,2}=Row;
H=cell2mat(H_cell);

error=PHI*kesi;% Nx*Npx1 ÿһ�������
f_cell=cell(1,2);
f_cell{1,1}=2*error'*Q*THETA;
f_cell{1,2}=0;
f=cell2mat(f_cell);

A_t=zeros(Nc,Nc);
for p=1:1:Nc
    for q=1:1:Nc
        if q<=p
            A_t(p,q)=1;
        else 
            A_t(p,q)=0;
        end
    end 
end 
A_I=kron(A_t,eye(Nu));
Ut=kron(ones(Nc,1),U);
umin=[-1;-1;-1;-1];
umax=[1;1;1;1];
delta = 0.0001;
delta_umin=[-delta;-delta;-delta;-delta];
delta_umax=[delta;delta;delta;delta];
Umin=kron(ones(Nc,1),umin);
Umax=kron(ones(Nc,1),umax);
A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
b_cons_cell={Umax-Ut;-Umin+Ut};
A_cons=cell2mat(A_cons_cell);
b_cons=cell2mat(b_cons_cell);
M=1;
delta_Umin=kron(ones(Nc,1),delta_umin);
delta_Umax=kron(ones(Nc,1),delta_umax);
lb=[delta_Umin;0];
ub=[delta_Umax;M];
options = optimset('Algorithm','active-set');
[X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
u_piao(1)=X(1);
u_piao(2)=X(2);
u_piao(3)=X(3);
u_piao(4)=X(4);
U(1)=kesi(9)+u_piao(1);%���ڴ洢��һ��ʱ�̵Ŀ�����
U(2)=kesi(10)+u_piao(2);
U(3)=kesi(11)+u_piao(3);
U(4)=kesi(12)+u_piao(4);
u_real(1) = U(1) + u(21);
u_real(2) = U(2) + u(22);
u_real(3) = U(3) + u(23);
u_real(4) = U(4) + u(24);
sys = u_real;

%sys��ʾ�������y
function sys=mdlGetTimeOfNextVarHit(t,x,u)
%������һ������ʱ��
%����ϵͳ�Ǳ����ʱ��ϵͳʱ����
sampleTime = 1; 
%������һ�β���ʱ������1s�Ժ�
sys = t + sampleTime;
%sys��ʾ��һ������ʱ���
function sys=mdlTerminate(t,x,u)
%�������ʱҪ���õĻص�����
%�ڷ������ʱ�������ڴ���ɷ����������ı�Ҫ����
sys = [];