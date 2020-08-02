clc;
clear;
%% ������
T = 0.01; %sample time
d = 3; %block distance
h = 5; %length of end-effector
o = 0.2; %pitch
n = o / (2 * pi);
ToDeg = 180 / pi;
ToRad = pi / 180;
Nx = 5; %number of environment condition
Nu = 4; %number of controller
Np = 40; %prediction step size
Tsim = Np;
%Nc = 20; %control step size
Row = 10; %relax
%% �ο�ģ�͵Ĺ켣����ģ��
%x1,y1���Ϸ����xyƽ������Գ�ʼ���·������Ĺ̶�������ƶ�����
%x2,y2���·����xyƽ������Գ�ʼ���·������Ĺ̶�������ƶ�����
%theta��end-effector����·���ƽ��ĽǶ�
sim('MPC_traj');
x1_re = x1.data;x2_re = x2.data; y1_re = y1.data;y2_re = y2.data;theta = theta.data;
Xout = [x1_re y1_re x2_re y2_re theta]; % ����״̬
%% MPC��������
%mpc���������ȶ�ϵͳ��Ч���ڴ˽��������˶�ѧģ�;��������Ի�-��ɢ��-����Ԥ��-�Ż��Ĳ������
%���Ի������MPC�Ż�Ŀ��Ϊ״̬��ֵ�Ϳ��Ʋ�ֵ
%��ز��������󣩳�ʼ��
X = [0 0 0 0 0]; %��ʼ״̬
U = [0 0 0 0]; %��ʼ������
[Nr,Nc] = size(Xout);
%״̬���д洢
x_real = zeros(Nr,Nc); x_real(1,:) = X; %��ʼ��ʵ״̬����
x_piao = zeros(Nr,Nc); x_piao(1,:) = x_real(1,:) - Xout(1,:);  %״̬��ƫ�MPCԪ��
%״̬���MPC����
X_PIAO = zeros(Nr,Nx * Tsim); %ÿ��MPCԤ��ƫ������
%�������д洢
u_real = zeros(Nr,Nu); %������
u_piao = zeros(Nr,Nu); %���������
XXX = zeros(Nr,Nx * Tsim);
q = [1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 1 0;0 0 0 0 0.5 ];
Q_cell = cell(Tsim,Tsim);
for i = 1:1:Tsim
    for j = 1:1:Tsim
        if i == j
            Q_cell{i,j}=q;
        else
            Q_cell{i,j}=zeros(Nx,Nx);
        end
    end
end
Q = cell2mat(Q_cell); %Ȩ�ؾ���
R = 0.1*eye(Nu*Tsim,Nu*Tsim); %Ȩ�ؾ���
xx = zeros(4,1); %��ǰ���ת���µĸ�����
%Ԥ������
for i = 1:1:Nr
    a = [1 0 0 0 0;
         0 1 0 0 0;
         0 0 1 0 0;
         0 0 0 1 0;
         0 0 0 0 1]; %Ŀ�����ٶ�Ϊ�㣬���ٶ�Ϊ�㣬�̶�����a
     b = [n*T 0 0 0;
          0 n*T 0 0;
          0 0 n*T 0;
          0 0 0 n*T;
          0 0 0 0];
      A_cell = cell(Tsim,1);
      B_cell = cell(Tsim,Tsim);
      for j = 1:1:Tsim
          A_cell{j,1} = a^j;
          for k = 1:1:Tsim
              if k <= j
                  B_cell{j,k} = (a^(j-k)) * b;
              else
                  B_cell{j,k} = zeros(Nx,Nu);
              end
          end
      end
      A = cell2mat(A_cell);
      B = cell2mat(B_cell);
      H = 2 * (B' * Q * B + R);
      f = 2 * B'* Q * A * x_piao(i,:)';
      A_cons = [];
      b_cons = [];
      v_cons = ToRad * 360;
      ub = [v_cons; v_cons; v_cons; v_cons];
      lb = -ub;
      [X,fval(i,1),exitflag(i,1),output(i,1)] = quadprog(H,f,A_cons,b_cons,[],[],lb,ub);
      X_PIAO(i,:) = (A*x_piao(i,:)'+B*X)';
      
%       if i+j < Nr
%           for k = 1:1:Tsim
%               for num = 1:1:Nx
%                 XXX(i,num+Nx*(k-1)) = X_PIAO(i,num+Nx*(k-1)) + Xout(i+k,num);
%               end
%           end
%       else
%           for  k = 1:1:Tsim
%               for num = 1:1:Nx
%                   XXX(i,num+Nx*(k-1)) = X_PIAO(i,num+Nx*(k-1)) + Xout(Nr,num);
%               end
%           end
%       end
%       x00 = x_real(i,:); %��ǰ˿��λ��״̬
%       for num = 1:1:Nu
%           u_piao(i,Nu) = X(Nu,1);
%           u_real(i,Nu) = u_piao(i,Nu); %��ǰʱ�̵Ŀ���������һʱ�̵Ŀ�������ͬ��
%           x_real(i+1,Nu) = x00(Nu) + n * T * u_real(i,Nu);  %��һʱ�̵�״̬
%       end
%       theta = RobotForward(x_real(i+1,1:4),d);
%       x_real(i+1,5) = theta;
%       if (i<Nr)
%         x_piao(i+1,:) = x_real(i+1,:) - Xout(i+1,:);
%       end
end



























