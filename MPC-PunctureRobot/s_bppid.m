function [sys,x0,str,ts,simStateCompliance] = exppidf(t,x,u,flag)

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

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
sizes=simsizes;
sizes.NumContStates=0;
sizes.NumDiscStates=3;
sizes.NumOutputs=4;
sizes.NumInputs=7;
sizes.DirFeedthrough=1;
sizes.NumSampleTimes=1;
sys=simsizes(sizes);
x0=[1,0.1,10];
str=[];
ts=[0 0];
simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)

sys = [x];
function sys=mdlUpdate(t,x,u)
T=0.1;
x=[u(5);x(2)+u(5)*T;(u(5)-u(4))/T];%e(k),e(k)���,e(k)��΢��
sys=[x(1);x(2);x(3)];


function sys=mdlOutputs(t,x,u)
xite=0.25;
alfa=0.04;
IN=3;H=5;OUT=3;
T=0.1;
global wi;
global wo;
global wi_1;
global wi_2;
global wi_3;

global wo_1;
global wo_2;
global wo_3;

global Kp;
global Ki;
global Kd;
%wi=rand(5,3);wo=rand(3,5);
Kup=2;Kui=2;Kud=2;
Oh=zeros(1,5);
I=Oh;
K1=zeros(3,1);K=zeros(1,3);dyu=0;dK=zeros(1,3);delta3=zeros(1,3);
 d_wo=zeros(3,5);segma=zeros(1,5);delta2=zeros(1,5);d_wi=zeros(5,3);dO=zeros(1,5);
xi=[u(1),u(3),u(5)];
epid=[x(1);x(2);x(3)];%���ڴ���pid�����������
I=xi*wi';%�����������
for j=1:1:5
    Oh(j)=(exp(I(j))-exp(-I(j)))/(exp(I(j))+exp(-I(j)));%����������
end
K1=wo*Oh';  %����������3x1
for i=1:1:3
    K(i)=exp(K1(i))/(exp(K1(i))+exp(-K(i)));%���������
end
K(1) = K(1)*Kup;K(2) = K(2)*Kui;K(3) = K(3)*Kud;
u_k=K*epid;
%������Ȩֵ����
%��������������Ȩֵ����
dyu=sign((u(3)-u(2))/(u(7)-u(6)+0.01));
for j=1:1:3
    dK(j)=2/(exp(K1(j))+exp(-K1(j)))^2; %�����������һ�׵�
end
for i=1:1:3
    delta3(i)=u(5)*dyu*epid(i)*dK(i);  %������delta
end
for j=1:1:3
    for i=1:1:5
        d_wo=xite*delta3(j)*Oh(i)+alfa*(wo_1-wo_2);
    end
end
wo=wo_1+d_wo+alfa*(wo_1-wo_2);
%��������������������Ȩֵ����
for i=1:1:5
    dO(i)=4/(exp(I(i))+exp(-I(i)))^2;
end
segma=delta3*wo;
for i=1:1:5
    delta2(i)=dO(i)*segma(i);
end
%for i=1:1:5
   % for j=1:1:3
      %  d_wi=xite*delta2(i)*xi(j)+alfa*(wi_1-wi_2);
   % end
%end
d_wi=xite*delta2'*xi;
wi=wi_1+d_wi+alfa*(wi_1-wi_2);

wo_3=wo_2;
wo_2=wo_1;
wo_1=wo;
wi_3=wi_2;
wi_2=wi_1;
wi_1=wi;
Kp=K(1);Ki=K(2);Kd=K(3);
sys=[u_k,Kp,Ki,Kd];
%sys=[1,2,3,4];

function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 0.1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)

sys = [];
% function [sys,x0,str,ts,simStateCompliance] = exppidf(t,x,u,flag)
% switch flag,
%     case 0,
%         [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
%     case 1,
%         sys=mdlDerivatives(t,x,u);
%     case 2,
%         sys=mdlUpdate(t,x,u);
%     case 3,
%         sys=mdlOutputs(t,x,u);
%     case 4,
%         sys=mdlGetTimeOfNextVarHit(t,x,u);
%     case 9,
%         sys=mdlTerminate(t,x,u);
%     otherwise
%         DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
%         
% end
% function[sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
%     sizes=simsizes;%��������ģ������Ľṹ����simsizes������
%     sizes.NumContStates=0;%ģ������״̬�����ĸ���
%     sizes.NumDiscStates=3;%ģ����ɢ״̬�����ĸ���
%     sizes.NumOutputs=4;%ģ����������ĸ���
%     sizes.NumInputs=7;%ģ����������ĸ���
%     sizes.DirFeedthrough=1;%ģ���Ƿ����ֱ�ӹ�ͨ��1��ʾ����ֱ�ӹ�ͨ����Ϊ0����mdlOutputs�����ﲻ����u
%     sizes.NumSampleTimes=1;%ģ��Ĳ���ʱ�����,������һ��
%     sys=simsizes(sizes);%������󸳸�sys���
%     x0=zeros(3,1);%ϵͳ״̬��������
%     simStateCompliance = 'UnknownSimState';
%     str=[];
%     ts=[0 0];%����������Ϊ0��ʾ������ϵͳ��
% %     ts=[0.001 0];%����������Ϊ0��ʾ������ϵͳ��
% function sys=mdlUpdate(x,u)
% T=0.1;
% x=[u(5);x(2)+u(5)*T;(u(5)-u(4))/T];%e(k),e(k)���,e(k)��΢��
% sys=[x(1);x(2);x(3)];
% 
% function sys=mdlOutputs(t,x,u)
%             xite=0.2;
%             alfa=0.05;
%             IN=3;H=5;OUT=3;
%             wi=rand(5,3);%����һ��5*3�����������������ڣ�0��1������
%             wi_1=wi;wi_2=wi;wi_3=wi;
%             wo=rand(3,5);
%             
%             wo_1=wo;wo_2=wo;wo_3=wo;
%             Oh=zeros(5,1);%����һ��1*5�������(�о���)
%             I=Oh;
%             xi=[u(1),u(3),u(5)];%������ѵ����3�����룬����ֵ������Լ�ʵ��ֵ
%             epid=[x(1);x(2);x(3)];%3��״̬������ƫ�ƫ��͡�ƫ��仯������3*1������������
%             I=xi*wi';%���������
%             for j=1:1:5
%                 Oh(j)=(exp(I(j))-exp(-I(j)))/(exp(I(j))+exp(-I(j)));%��������ֵ��1*5�����о���
%             end
%             K1=wo*Oh;%���������루3*1����
%             for i=1:1:3
%                 K(i)=exp(K1(i))/(exp(K1(i))+exp(-K1(i)));%�õ������������KP��KI��KD����1*3������������
%             end
%             u_k=K*epid;%����õ�������u��1��ֵ
%             %%������Ȩֵ����
%             %��������������Ȩֵ����
%             dyu=sign((u(3)-u(2))/(u(7)-u(6)+0.0001));
%             for j=1:1:3
%                 dK(j)=2/(exp(K1(j))+exp(-K1(j)))^2; %�����������һ�׵�
%             end
%             for i=1:1:3
%                 delta3(i)=u(5)*dyu*epid(i)*dK(i);  %������delta
%             end
%             for j=1:1:3
%                 for i=1:1:5
%                     d_wo=xite*delta3(j)*Oh(i)+alfa*(wo_1-wo_2);
%                 end
%             end
%             wo=wo_1+d_wo;
%             %��������������������Ȩֵ����
%             for i=1:1:5
%                 dO(i)=4/(exp(I(i))+exp(-I(i)))^2;%(1*5����)
%             end
%             segma=delta3*wo;%��1*5������������
%             delta2 = dO.*segma;
%             d_wi = delta2'*xi+alfa*(wi_1-wi_2);
%             wi=wi_1+d_wi;
%             wo_3=wo_2;
%             wo_2=wo_1;
%             wo_1=wo;%��������㱾�ε������Ȩֵ
%             wi_3=wi_2;
%             wi_2=wi_1;
%             wi_1=wi;%�������㱾�ε������Ȩֵ
%          Kp=K(1);Ki=K(2);Kd=K(3);
%          sys=[u_k,Kp,Ki,Kd];       
