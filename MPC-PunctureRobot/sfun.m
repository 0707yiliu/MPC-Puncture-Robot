function [sys,x0,str,ts]=sfun2(t,x,u,flag)
switch flag,
    case 0,
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 2,
        sys=mdlUpdates(x,u);
    case 3,
        sys=mdlOutputs(t,x,u);
    case {1,4,9},
        sys=[];
    otherwise
        error(['unhandled flag=',num2str(flag)]);%�쳣����
end
function[sys,x0,str,ts]=mdlInitializeSizes
    sizes=simsizes;%��������ģ������Ľṹ����simsizes������
    sizes.NumContStates=0;%ģ������״̬�����ĸ���
    sizes.NumDiscStates=3;%ģ����ɢ״̬�����ĸ���
    sizes.NumOutputs=4;%ģ����������ĸ���
    sizes.NumInputs=7;%ģ����������ĸ���
    sizes.DirFeedthrough=1;%ģ���Ƿ����ֱ�ӹ�ͨ��1��ʾ����ֱ�ӹ�ͨ����Ϊ0����mdlOutputs�����ﲻ����u
    sizes.NumSampleTimes=1;%ģ��Ĳ���ʱ�����,������һ��
    sys=simsizes(sizes);%������󸳸�sys���
    x0=zeros(3,1);%ϵͳ״̬��������
    str=[];
    ts=[0 0];%����������Ϊ0��ʾ������ϵͳ��
%     ts=[0.001 0];%����������Ϊ0��ʾ������ϵͳ��
function sys=mdlUpdates(x,u)
        T=0.001;
        x=[u(5);x(2)+u(5)*T;(u(5)-u(4))/T];%3��״̬����ƫ�ƫ����Լ�ƫ��仯������u(5)��ƫ�u(4)����һ�ε�ƫ�x(2)����֮ǰ��ƫ���
        sys=[x(1);x(2);x(3)];
function sys=mdlOutputs(t,x,u)
            xite=0.2;
            alfa=0.05;
            IN=3;H=5;OUT=3;
            wi=rand(5,3);%����һ��5*3�����������������ڣ�0��1������
            wi_1=wi;wi_2=wi;wi_3=wi;
            wo=rand(3,5);
            wo_1=wo;wo_2=wo;wo_3=wo;
            Oh=zeros(5,1);%����һ��1*5�������(�о���)
            I=Oh;
            xi=[u(1),u(3),u(5)];%������ѵ����3�����룬����ֵ������Լ�ʵ��ֵ
            epid=[x(1);x(2);x(3)];%3��״̬������ƫ�ƫ��͡�ƫ��仯������3*1������������
            I=xi*wi';%���������
            for j=1:1:5
                Oh(j)=(exp(I(j))-exp(-I(j)))/(exp(I(j))+exp(-I(j)));%��������ֵ��1*5�����о���
            end
            K1=wo*Oh;%���������루3*1����
            for i=1:1:3
                K(i)=exp(K1(i))/(exp(K1(i))+exp(-K1(i)));%�õ������������KP��KI��KD����1*3������������
            end
            u_k=K*epid;%����õ�������u��1��ֵ
            %%������Ȩֵ����
            %��������������Ȩֵ����
            dyu=sign((u(3)-u(2))/(u(7)-u(6)+0.0001));
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
            wo=wo_1+d_wo;
            %��������������������Ȩֵ����
            for i=1:1:5
                dO(i)=4/(exp(I(i))+exp(-I(i)))^2;%(1*5����)
            end
            segma=delta3*wo;%��1*5������������
            delta2 = dO.*segma;
            d_wi = delta2'*xi+alfa*(wi_1-wi_2);
            wi=wi_1+d_wi;
            wo_3=wo_2;
            wo_2=wo_1;
            wo_1=wo;%��������㱾�ε������Ȩֵ
            wi_3=wi_2;
            wi_2=wi_1;
            wi_1=wi;%�������㱾�ε������Ȩֵ
         Kp=K(1);Ki=K(2);Kd=K(3);
         sys=[u_k,Kp,Ki,Kd];       