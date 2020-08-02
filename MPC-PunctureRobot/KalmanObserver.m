function [sys,x0,str,ts,simStateCompliance] = KalmanObserver(t,x,u,flag)
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
sizes.NumOutputs     = 8;
%输出个数，缺省为0
sizes.NumInputs      = 8;
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
global P Xkf;
value = 0.001;
P = [value,0,0,0,0,0,0,0;
     0,value,0,0,0,0,0,0;
     0,0,value,0,0,0,0,0;
     0,0,0,value,0,0,0,0;
     0,0,0,0,value,0,0,0;
     0,0,0,0,0,value,0,0;
     0,0,0,0,0,0,value,0;
     0,0,0,0,0,0,0,value];
Xkf = [0;0;0;0;0;0;0;0];
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
function sys=mdlOutputs(t,x,u) %1x8
global P Xkf;
%噪声函数
R = 0.01 * eye(8,8); % 8x8
Q = zeros(8,8); %8x8
%参数矩阵
H = eye(8,8); %8x8
Z = H * u + sqrt(R) * randn(8,1);%测量值 8x1
X_pre = Xkf;%上一时刻最优状态值 8x1
P_pre = P + Q; %8x8
Kg = P_pre * H' * inv(H * P_pre * H' + R);%8x8
Xkf = X_pre + Kg * (Z - H * X_pre);
P = (eye(8,8) - Kg * H) * P_pre;
sys = Xkf;

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
