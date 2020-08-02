%% 生成目标轨迹
clear;
clc;
N = 200; % 目标轨迹点的数量
T = 0.05; % 采样周期
Xout = zeros(N,3); % Xout用于存储目标轨迹点上的车辆状态；车辆共有3个状态变量：X轴坐标、Y轴坐标、航向角
Tout = zeros(N,1); % Tout用于存储离散的时间序列
for k = 1:1:N
    Xout(k,1) = k*T; % 设置目标轨迹点的X轴坐标
    Xout(k,2) = 1+k*T; % 设置目标轨迹点的Y轴坐标
    Xout(k,3) = 0; % 设置目标轨迹点的航向角
    Tout(k,1) = (k-1)*T; % 由此可见，目标轨迹点对应的时刻，和采样时刻保持一致；整个仿真时长为100*0.5共计5秒（默认单位时长为1秒）
    % 由上可知，车辆在目标轨迹上，每两个轨迹点间的X轴的坐标相差0.05，每两个轨迹点对应的时刻相差0.05，因此车辆在X轴上的速度为1m/s
end

%% 描述系统的基本情况
Nx = 3; % 状态量共有3个：X轴坐标、Y轴坐标、航向角；对应于Xout的每一列
Nu = 2; % 控制量共有2个：车辆的纵向速度，前轮偏角
[Nr,Nc] = size(Xout); % 检测目标轨迹点车辆状态矩阵的维度，Nr代表目标点的数目（100），Nc代表车辆状态的数目（3）
Tsim = 20; % MPC的预测时域，即当车辆处于时刻k时，对k+1,k+2,k+3,...,k+Tsim时刻的车辆状态进行预测
X0 = [0 0 pi/6]; % 车辆初始状态，X轴坐标为0，Y坐标为0，航向角pi/3
L = 1; % 车辆轴距
vd1 = 1; % 目标纵向速度
vd2 = 0; % 目标前轮偏角

%% 定义相关矩阵
% x_real矩阵，用于存储每一个仿真时刻，车辆的位置状态，初始状态为上述定义的X0
x_real = zeros(Nr,Nc); x_real(1,:) = X0;
% x_piao矩阵，用于存储每一个仿真时刻，车辆的位置状态与目标位置状态之间的偏差
x_piao = zeros(Nr,Nc); x_piao(1,:) = x_real(1,:)-Xout(1,:);
% X_PIAO的每一行代表一个仿真时刻（对应一个轨迹点）
% 在每一个仿真时刻，都需要对未来20个时刻的状态偏差进行预测，所以X_PIAO每一行的数据可以分为20（Tsim）个组
% 每一组有3（Nx）列，分别对应车辆3个状态的偏差
X_PIAO = zeros(Nr,Nx*Tsim);
% u_real矩阵，用于存储每一个仿真时刻，车辆的实际控制量（运动状态）（纵向速度、前轮偏角）
u_real = zeros(Nr,Nc);
% u_piao矩阵，用于存储每一个仿真时刻，车辆的实际控制量（实际运动状态）与目标控制量（运动状态）之间的偏差
u_piao = zeros(Nr,Nc);
% XXX的每一行代表一个仿真时刻（对应一个轨迹点）
% 在每一个仿真时刻，都需要对未来20个时刻的状态值进行预测，所以XXX每一行的数据可以分为20（Tsim）个组
% 每一组有3（Nx）列，分别对应车辆3个状态
XXX = zeros(Nr,Nx*Tsim);% 用于保存每个时刻预测的所有状态值
% q为加权矩阵，用于调节每个状态量（X轴坐标、Y轴坐标、航向角）在目标函数中比重
q = [1 0 0;0 1 0;0 0 0.5];
Q_cell = cell(Tsim,Tsim); % Q_cell的作用是，将q作用于预测的每一个点上（共计Tsim个点）
for i = 1:1:Tsim
    for j = 1:1:Tsim
        if i == j
            Q_cell{i,j}=q;
        else
            Q_cell{i,j}=zeros(Nx,Nx);
        end
    end
end
Q = cell2mat(Q_cell); % 最终的状态加权矩阵，用于调节每个预测点上，三个状态量各自在目标函数中比重
R = 0.1*eye(Nu*Tsim,Nu*Tsim); % 最终的状态加权矩阵，用于调节“控制量偏差”和“状态量偏差”在目标函数中的比重

%% 模型预测控制的主体函数
for i = 1:1:Nr
    t_d = Xout(i,3); % t_d为i时刻的期望航向角
    % 下一预测点的状态偏差 = a*当前点的状态偏差 + b*当前点的控制量偏差   （状态偏差，即车辆位置偏差；控制量偏差，即车辆运动偏差）
    a = [1 0 -vd1*sin(t_d)*T;   % 矩阵a，是目标车速和目标航向角的函数；
        0 1 vd1*cos(t_d)*T; % 由于目标车速和目标航向角保持恒定
        0 0 1;]; % 所以，矩阵a恒定不变
    b = [cos(t_d)*T 0; % 与矩阵a相似，矩阵b也保持恒定
        sin(t_d)*T 0;
        tan(vd2)*T/L vd1*T/(L*cos(vd2)^2);];% 此处已经对书中的错误进行了更正
    % 目标函数，是预测时域内预测点方差之和；  预测时域为Tsim，即存在20个预测点；  预测点方差为“状态方差（位置方差）” + “控制量方差（运动方差）”；
    A_cell = cell(Tsim,1);
    B_cell = cell(Tsim,Tsim);
    for j = 1:1:Tsim
         % 因为目标速度vd1、目标航向角保持不变，所以A(k|k)、A(k+1|k)、...全都相同
        A_cell{j,1} = a^j;
        for k = 1:1:Tsim
            if k<=j
                B_cell{j,k} = (a^(j-k))*b;
            else
                B_cell{j,k} = zeros(Nx,Nu);
            end
        end
    end
    
    A = cell2mat(A_cell);
    B = cell2mat(B_cell);
    
    H = 2*(B'*Q*B + R);
    f = 2*B'*Q*A*x_piao(i,:)';
    A_cons = [];
    b_cons = [];
    % 每一步， -2.2 <= v - vr <= 0.2
    % 每一步， -0.64 <= 前轮偏角 - 目标前轮偏角 <= 0.64
    ub = [0.2; 0.64];    lb = [-2.2; -0.64]; 
    % 通过二次规划求解，得到的X，为最优控制偏差矩阵[Dlt_u1,Dlt_u2,Dlt_u3,...,Dlt_u19]
    [X,fval(i,1),exitflag(i,1),output(i,1)] = quadprog(H,f,A_cons,b_cons,[],[],lb,ub);
    % 通过运动学公式，计算得到在控制偏差矩阵[Dlt_u1,Dlt_u2,Dlt_u3,...,Dlt_u19]作用下的状态偏差矩阵
    % [Dlt_x1_1,Dlt_x2_1,Dlt_x3_1, Dlt_x1_2,Dlt_x2_2,Dlt_x3_2, ..., Dlt_x1_19,Dlt_x2_19,Dlt_x3_19]
    % 并将其存储在 X_PIAO(i,:) 中
    X_PIAO(i,:) = (A*x_piao(i,:)'+B*X)';
    
    % 下面的j，虽然是上面for循环的变量，但是MATLAB不会将j自动清零；所以，j的大小即为Tsim
    if i+j < Nr % i代表当前时刻，j代表相对于当前时刻向前预测的时刻；此句代表预测的时刻没有超出整个仿真时长
        for k = 1:1:Tsim
            % 存储预测到的状态值（预测值 = 在最优控制偏差下得到的状态偏差值 + 目标状态值
            XXX(i,1+3*(k-1)) = X_PIAO(i,1+3*(k-1)) + Xout(i+k,1); 
            XXX(i,2+3*(k-1)) = X_PIAO(i,2+3*(k-1)) + Xout(i+k,2);
            XXX(i,3+3*(k-1)) = X_PIAO(i,3+3*(k-1)) + Xout(i+k,3);
        end
    else
        for k = 1:1:Tsim
            XXX(i,1+3*(k-1)) = X_PIAO(i,1+3*(k-1)) + Xout(Nr,1);
            XXX(i,2+3*(k-1)) = X_PIAO(i,2+3*(k-1)) + Xout(Nr,2);
            XXX(i,3+3*(k-1)) = X_PIAO(i,3+3*(k-1)) + Xout(Nr,3);
        end
    end
    u_piao(i,1) = X(1,1); % 取最有解序列的第一个元素,纵向速度
    u_piao(i,2) = X(2,1); % 取最有解序列的第一个元素,前轮转角
    
    Tvec = [0:0.05:4];
    x00 = x_real(i,:); % x00为当前时刻的位置状态
    vd11 = vd1 + u_piao(i,1); % 依据MPC求得的最优控制偏移u_piao(i,1)，以及当前的控制量vd1，计算出下一时刻的控制量vd11
    vd22 = vd2 + u_piao(i,2); % 依据MPC求得的最优控制偏移u_piao(i,2)，以及当前的控制量vd2，计算出下一时刻的控制量vd22
    % 知道了当前时刻的位置状态、运动状态（控制量）
    % 下面根据运动学方程，求解下一时刻的位置状态
    XOUT = dsolve('Dx - vd11*cos(z) = 0',...
        'Dy - vd11*sin(z) = 0',...
        'Dz - vd22 = 0',...
        'x(0) = x00(1)',...
        'y(0) = x00(2)',...
        'z(0) = x00(3)');
    t = T; % T为采样周期
    x_real(i+1,1) = eval(XOUT.x);
    x_real(i+1,2) = eval(XOUT.y);
    x_real(i+1,3) = eval(XOUT.z);
    if (i<Nr)
        x_piao(i+1,:) = x_real(i+1,:) - Xout(i+1,:);
    end
    % 根据目标运动状态（目标速度和目标前轮转角），以及上述求得的最优控制偏差（运动量偏差）
    % 计算当前最佳的控制量（当前速度、当前转向角）
    u_real(i,1) = vd1 + u_piao(i,1); % vd1为目标控制量：目标速度
    u_real(i,2) = vd2 + u_piao(i,2); % vd1为目标控制量：目标前轮转向角
    
    figure(1); 
    plot(Xout(1:Nr,1),Xout(1:Nr,2));    
    hold on;
    plot(x_real(i,1),x_real(i,2),'r *')
    xlabel('X[m]'); % axis([-15 -13]);
    ylabel('Y[m]');
    hold on;
    
    for k = 1:1:Tsim
        X(i,k+1) = XXX(i,1+3*(k-1));
        Y(i,k+1) = XXX(i,2+3*(k-1));
    end
    X(i,1) = x_real(i,1);
    Y(i,1) = x_real(i,2);
    plot(X(i,:),Y(i,:),'y')
    hold on    
end
