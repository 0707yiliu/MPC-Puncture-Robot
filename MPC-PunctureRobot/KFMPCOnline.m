function [sys,x0,str,ts,simStateCompliance] = KFMPC(t,x,u,flag)
%主函数 
%1.需要通过KalmanFilter在线辨识系统模型参数作为当前MPC的模型，需要输入参数有：
%当前模型参数A1 A2 B共12个，当前状态量X即速度以及加速度共8个，共输入20个参数
%2.离线辨识系统模型，不需要输入模型参数，共输入8个参数
%主函数包含四个输出：
%                 sys数组包含某个子函数返回的值
%                 x0为所有状态的初始化向量
%                 str是保留参数，总是一个空矩阵
%                 Ts返回系统采样时间
%函数的四个输入分别为采样时间t、状态x、输入u和仿真流程控制标志变量flag
%输入参数后面还可以接续一系列的附带参数simStateCompliance
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
%主函数结束
%下面是各个子函数，即各个回调过程
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
%初始化回调子函数
%提供状态、输入、输出、采样时间数目和初始状态的值
%初始化阶段，标志变量flag首先被置为0，S-function首次被调用时
%该子函数首先被调用，且为S-function模块提供下面信息
%该子函数必须存在
sizes = simsizes;
%生成sizes数据结构，信息被包含在其中
sizes.NumContStates  = 0;
%连续状态数，缺省为0
sizes.NumDiscStates  = 0;
%离散状态数，缺省为0
sizes.NumOutputs     = 4;
%输出个数，缺省为0
sizes.NumInputs      = 12 + 8 + 4;
%模型参数 + 状态变量 + 目标轨迹
%输入个数，缺省为0
sizes.DirFeedthrough = 1;
%是否存在直馈通道，1存在，0不存在
sizes.NumSampleTimes = 1;
%采样时间个数，至少是一个
sys = simsizes(sizes);
%返回size数据结构所包含的信息
x0  = [];
%设置初始状态
str = [];
%保留变量置空
ts  = [0.01 0];
%设置采样时间
global U param;
param = zeros(12,1);
U = [0;0;0;0];
simStateCompliance = 'UnknownSimState';
function sys=mdlDerivatives(t,x,u)
%计算导数回调子函数
%给定t,x,u计算连续状态的导数，可以在此给出系统的连续状态方程
%该子函数可以不存在
sys = [];
%sys表示状态导数，即dx
function sys=mdlUpdate(t,x,u)
%状态更新回调子函数
%给定t、x、u计算离散状态的更新
%每个仿真步内必然调用该子函数，不论是否有意义
%除了在此描述系统的离散状态方程外，还可以在此添加其他每个仿真步内都必须执行的代码
sys = x;
%sys表示下一个离散状态，即x(k+1)
function sys=mdlOutputs(t,x,u)
%计算输出回调函数
%给定t,x,u计算输出，可以在此描述系统的输出方程
%该子函数必须存在
global U param;
Nx = 8; %状态量个数：每个方块的速度分量和加速度分量
Nu = 4; %控制量个数：每个电机的角度
Np = 40; %预测步长
Nc = 20; %控制步长
Row = 0.1; %松弛因子
kesi = zeros(Nx + Nu, 1);
for i  = 1:1:Nx
    kesi(i) = u(i+12); %状态量
end
kesi(9) = U(1); %控制量差值最小
kesi(10) = U(2);
kesi(11) = U(3);
kesi(12) = U(4);
T = 0.01;
param = u(1:12);
%1-4：A1
%5-8：A2
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
     0 0 0 0 0 0 0 1 0 0 0 0];%8x12 与输入联系，但无关,仅为后面计算
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

error=PHI*kesi;% Nx*Npx1 每一步的误差
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
U(1)=kesi(9)+u_piao(1);%用于存储上一个时刻的控制量
U(2)=kesi(10)+u_piao(2);
U(3)=kesi(11)+u_piao(3);
U(4)=kesi(12)+u_piao(4);
u_real(1) = U(1) + u(21);
u_real(2) = U(2) + u(22);
u_real(3) = U(3) + u(23);
u_real(4) = U(4) + u(24);
sys = u_real;

%sys表示输出，即y
function sys=mdlGetTimeOfNextVarHit(t,x,u)
%计算下一个采样时间
%仅在系统是变采样时间系统时调用
sampleTime = 1; 
%设置下一次采样时间是在1s以后
sys = t + sampleTime;
%sys表示下一个采样时间点
function sys=mdlTerminate(t,x,u)
%仿真结束时要调用的回调函数
%在仿真结束时，可以在此完成仿真结束所需的必要工作
sys = [];