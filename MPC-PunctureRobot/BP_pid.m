function Y = BP_pid( u )  %����uΪ����ʸ��
nh=5;    %nh��������ڵ�����xite��ѧϰ����,alfa�ǹ���ϵ��,kF1,kF2�ֱ��������㡢����㼤�������
kF1=1;
kF2=2;
xite=0.1;
alfa=0.05;
wi_2=reshape(u(8:7+4*nh),nh,4);  %��Ϊÿ��������ڵ��Ӧ4������(r,y,e,1)��3�����(kp,ki,kd)����7�����ټ��Ϸ��ص�·������14����nh���ڵ㣬��Ϊ7*nh��
wo_2=reshape(u(8+4*nh:7+7*nh),3,nh);
wi_1=reshape(u(8+7*nh:7+11*nh),nh,4);
wo_1=reshape(u(8+11*nh:7+14*nh),3,nh);
xi=[u([6,4,1])',1];   %����������xi=(r,y,e,1)
xx=[u(1)-u(2); u(1); u(1)+u(3)-2*u(2)];
I=xi*wi_1';         %������������
Oh=non_transfun(I,kF1);  %���������
K=non_transfun(wo_1*Oh',kF2);  %����������K=(kp;ki;kd)��
uu=u(7)+K'*xx; %����PID���������
%ay/au,Ȩϵ������
dyu=sign((u(4)-u(5))/(uu-u(7)+0.0000001));  
dK=non_transfun(K,3);
delta3=u(1)*dyu*xx.*dK;
wo=wo_1+xite*delta3*Oh+alfa*(wo_1-wo_2)+alfa*(wo_1-wo_2);  %�����Ȩϵ������

dO=2*non_transfun(I,3);
wi=wi_1+xite*(dO.*(delta3'*wo))'*xi+alfa*(wi_1-wi_2);  %������Ȩϵ������
wi=wi(:);
wo=wo(:);
Y=[uu;K;wi;wo];
K  
% �����
function W1=non_transfun(W,key)
switch key
   case 1, W1=(exp(W)-exp(-W))./(exp(W)+exp(-W));
   case 2, W1=exp(W)./(exp(W)+exp(-W));
   case 3, W1=2./(exp(W)+exp(-W)).^2;
end





