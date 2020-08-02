function Y = BP_pid( u )  %其中u为输入矢量
nh=5;    %nh是隐含层节点数，xite是学习速率,alfa是惯性系数,kF1,kF2分别是隐含层、输出层激活函数类型
kF1=1;
kF2=2;
xite=0.1;
alfa=0.05;
wi_2=reshape(u(8:7+4*nh),nh,4);  %因为每个隐含层节点对应4个输入(r,y,e,1)和3个输出(kp,ki,kd)，共7个，再加上返回的路径，共14个，nh个节点，就为7*nh个
wo_2=reshape(u(8+4*nh:7+7*nh),3,nh);
wi_1=reshape(u(8+7*nh:7+11*nh),nh,4);
wo_1=reshape(u(8+11*nh:7+14*nh),3,nh);
xi=[u([6,4,1])',1];   %神经网络输入xi=(r,y,e,1)
xx=[u(1)-u(2); u(1); u(1)+u(3)-2*u(2)];
I=xi*wi_1';         %算隐含层输入
Oh=non_transfun(I,kF1);  %隐含层输出
K=non_transfun(wo_1*Oh',kF2);  %输出层输出（K=(kp;ki;kd)）
uu=u(7)+K'*xx; %计算PID控制器输出
%ay/au,权系数调整
dyu=sign((u(4)-u(5))/(uu-u(7)+0.0000001));  
dK=non_transfun(K,3);
delta3=u(1)*dyu*xx.*dK;
wo=wo_1+xite*delta3*Oh+alfa*(wo_1-wo_2)+alfa*(wo_1-wo_2);  %输出层权系数调整

dO=2*non_transfun(I,3);
wi=wi_1+xite*(dO.*(delta3'*wo))'*xi+alfa*(wi_1-wi_2);  %隐含层权系数调整
wi=wi(:);
wo=wo(:);
Y=[uu;K;wi;wo];
K  
% 激活函数
function W1=non_transfun(W,key)
switch key
   case 1, W1=(exp(W)-exp(-W))./(exp(W)+exp(-W));
   case 2, W1=exp(W)./(exp(W)+exp(-W));
   case 3, W1=2./(exp(W)+exp(-W)).^2;
end





