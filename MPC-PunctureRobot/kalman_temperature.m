%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ����˵����Kalman�˲������¶Ȳ�����ʵ��
% ��ϸԭ����ܼ�����ע����ο���
% ���������˲�ԭ��Ӧ��-MATLAB���桷�����ӹ�ҵ�����磬��Сƽ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function main
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N=1200;
CON=25;
Xexpect=CON*ones(1,N);
X=zeros(1,N);  
Xkf=zeros(1,N);
Z=zeros(1,N);
P=zeros(1,N); 
X(1)=0;
P(1)=0.01;
Z(1)=0;
Xkf(1)=Z(1);
Q=0.01;
R=0.25;
W=sqrt(Q)*randn(1,N);
V=sqrt(R)*randn(1,N);
F=1;
G=1;
H=1;
I=eye(1); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=2:N
    X(k)=F*X(k-1)+G*W(k-1);
%     X(k)= 25;
    Z(k)=H*X(k)+V(k);
    X_pre=F*Xkf(k-1);           
    P_pre=F*P(k-1)*F'+Q;        
    Kg=P_pre*inv(H*P_pre*H'+R); 
    e=Z(k)-H*X_pre;            
    Xkf(k)=X_pre+Kg*e;         
    P(k)=(I-Kg*H)*P_pre;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Err_Messure=zeros(1,N);
Err_Kalman=zeros(1,N);
for k=1:N
    Err_Messure(k)=abs(Z(k)-X(k));
    Err_Kalman(k)=abs(Xkf(k)-X(k));
end
t=1:N;
figure('Name','Kalman Filter Simulation','NumberTitle','off');
plot(t,Xexpect,'-b',t,X,'-r',t,Z,'-k',t,Xkf,'-g');
legend('expected','real','measure','kalman extimate');         
xlabel('sample time');
ylabel('temperature');
title('Kalman Filter Simulation');
figure('Name','Error Analysis','NumberTitle','off');
plot(t,Err_Messure,'-b',t,Err_Kalman,'-k');
legend('messure error','kalman error');         
xlabel('sample time');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

