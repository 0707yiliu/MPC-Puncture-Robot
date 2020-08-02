clc;
clear;
%% 超参数
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
%% 参考模型的轨迹生成模块
%x1,y1：上方块的xy平面轴相对初始的下方块中心固定坐标的移动距离
%x2,y2：下方块的xy平面轴相对初始的下方块中心固定坐标的移动距离
%theta：end-effector相对下方块平面的角度
sim('MPC_traj');
x1_re = x1.data;x2_re = x2.data; y1_re = y1.data;y2_re = y2.data;theta = theta.data;
Xout = [x1_re y1_re x2_re y2_re theta]; % 期望状态
%% MPC方法跟踪
%mpc方法仅对稳定系统有效，在此将机器人运动学模型经过：线性化-离散化-递推预测-优化的步骤进行
%线性化后定义的MPC优化目标为状态差值和控制插值
%相关参数（矩阵）初始化
X = [0 0 0 0 0]; %初始状态
U = [0 0 0 0]; %初始控制量
[Nr,Nc] = size(Xout);
%状态序列存储
x_real = zeros(Nr,Nc); x_real(1,:) = X; %初始真实状态序列
x_piao = zeros(Nr,Nc); x_piao(1,:) = x_real(1,:) - Xout(1,:);  %状态的偏差，MPC元素
%状态误差MPC序列
X_PIAO = zeros(Nr,Nx * Tsim); %每次MPC预测偏差序列
%控制序列存储
u_real = zeros(Nr,Nu); %控制量
u_piao = zeros(Nr,Nu); %控制量误差
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
Q = cell2mat(Q_cell); %权重矩阵
R = 0.1*eye(Nu*Tsim,Nu*Tsim); %权重矩阵
xx = zeros(4,1); %当前电机转速下的给进量
%预测主体
for i = 1:1:Nr
    a = [1 0 0 0 0;
         0 1 0 0 0;
         0 0 1 0 0;
         0 0 0 1 0;
         0 0 0 0 1]; %目标电机速度为零，角速度为零，固定矩阵a
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
%       x00 = x_real(i,:); %当前丝杆位置状态
%       for num = 1:1:Nu
%           u_piao(i,Nu) = X(Nu,1);
%           u_real(i,Nu) = u_piao(i,Nu); %当前时刻的控制量和下一时刻的控制量相同？
%           x_real(i+1,Nu) = x00(Nu) + n * T * u_real(i,Nu);  %下一时刻的状态
%       end
%       theta = RobotForward(x_real(i+1,1:4),d);
%       x_real(i+1,5) = theta;
%       if (i<Nr)
%         x_piao(i+1,:) = x_real(i+1,:) - Xout(i+1,:);
%       end
end



























