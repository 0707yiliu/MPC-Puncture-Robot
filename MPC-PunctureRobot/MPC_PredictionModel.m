function [sys,x0,str,ts,simStateCompliance] = MPC_PredictionModel(t,x,u,flag)
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
sizes.NumOutputs     = 8;
%���������ȱʡΪ0
sizes.NumInputs      = 4;
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
global y11 y22;
y11 = [0;0;0;0];y22=[0;0;0;0];
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
function sys=mdlOutputs(t,x,u) %1x8
global y11 y22;
A = [0 1 0 0 0 0 0 0;
     0.005 0.045 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0;
     0 0 0.0185 0.0251 0 0 0 0;
     0 0 0 0 0 1 0 0;
     0 0 0 0 0.0065 -0.0258 0 0;
     0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 -0.023 0.0084]; %8x8
b = [0 0 0 0;
     1.8899 0 0 0;
     0 0 0 0;
     0 1.8777 0 0;
     0 0 0 0;
     0 0 1.7655 0;
     0 0 0 0;
     0 0 0 1.7735]; % 8x4
C = [0 1 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0;
     0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 1];
y = [y22(1);y11(1);y22(2);y11(2);y22(3);y11(3);y22(4);y11(4)];
x = y;
output = C * (A * x + b * u);
y22 = y11;
y11 = output;
y = [y22(1);y11(1);y22(2);y11(2);y22(3);y11(3);y22(4);y11(4)];
x = y;
sys = x;

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
