function [sys,x0,str,ts] = mympc(t,x,u,flag,T,d,h,Nx,Nu,Np,Nc,Row)
switch flag,
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes(t,x,u,T,d,h,Nx,Nu,Np,Nc,Row); % Initialization
  
 case 2
  sys = mdlUpdates(t,x,u,T,d,h,Nx,Nu,Np,Nc,Row); % Update discrete states
  
 case 3
  sys = mdlOutputs(t,x,u,T,d,h,Nx,Nu,Np,Nc,Row); % Calculate outputs
 
 case {1,4,9} % Unused flags
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes(t,x,u,T,d,h,Nx,Nu,Np,Nc,Row)

% Call simsizes for a sizes structure, fill it in, and convert it 
% to a sizes array.

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 3;
sizes.NumOutputs     = Nu;
sizes.NumInputs      = Nx;
sizes.DirFeedthrough = 1; % Matrix D is non-empty.
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 =[0;0;0];   
global U X;
U = [0;0;0;0];
X = [0;0;0;0];
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [0.01 0];       % sample time: [period, offset]
%End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u,T,d,h,Nx,Nu,Np,Nc,Row)
  
sys = x;
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u,T,d,h,Nx,Nu,Np,Nc,Row)
    global a b u_piao;
    global U;%控制量
    global kesi;%状态量和控制量的组合，滚动优化用kasi进行综合优化
    kesi=zeros(Nx+Nu,1);
    kesi(1)=u(1)-r(1);%u(1)==X(1)
    kesi(2)=u(2)-r(2);%u(2)==X(2)
    kesi(3)=u(3)-r(3); %u(3)==X(3)
    kesi(4)=u(4);
    n = 0.2 / pi;
    kesi(5)=U(1);  %第一个控制量分量
    kesi(6)=U(2);  %第二个控制量分量
    kesi(7)=U(3); %...
    kesi(8)=U(4); %...

    T_all=40;%临时设定，总的仿真时间，主要功能是防止计算期望轨迹越界
    
%矩阵初始化   
    u_piao=zeros(Nx,Nu);  
    Q=100*eye(Nx*Np,Nx*Np); 
    R=5*eye(Nu*Nc);
    a=[1  0  0  0;
       0  1  0  0;
       0  0  1  0;
       0  0  0  1];
    b=[n*T 0 0 0;
       0 n*T 0 0;
       0 0 n*T 0;
       0 0 0 n*T];
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;
    A_cell{1,2}=b;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=b;
    B_cell{2,1}=eye(Nu);
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
    C=[1 0 0 0 0 0 0 0;
       0 1 0 0 0 0 0 0;
       0 0 1 0 0 0 0 0;
       0 0 0 1 0 0 0 0;];
    PHI_cell=cell(Np,1);
    THETA_cell=cell(Np,Nc);
    for j=1:1:Np
        PHI_cell{j,1}=C*A^j;
        for k=1:1:Nc
            if k<=j
                THETA_cell{j,k}=C*A^(j-k)*B;
            else 
                THETA_cell{j,k}=zeros(Nx,Nu);
            end
        end
    end
    PHI=cell2mat(PHI_cell);%size(PHI)=[Nx*Np Nx+Nu]
    THETA=cell2mat(THETA_cell);%size(THETA)=[Nx*Np Nu*(Nc+1)]
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;  %60X60
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell);

     error=PHI*kesi;
    f_cell=cell(1,2);
    f_cell{1,1}=2*error'*Q*THETA;
    f_cell{1,2}=0;
%     f=(cell2mat(f_cell))';
    f=cell2mat(f_cell);
    
  %% 以下为约束生成区域
%  %不等式约束
%     A_t=zeros(Nc,Nc);%见falcone论文 P181
%     for p=1:1:Nc
%         for q=1:1:Nc
%             if q<=p 
%                 A_t(p,q)=1;
%             else 
%                 A_t(p,q)=0;
%             end
%         end 
%     end 
%     A_I=kron(A_t,eye(Nu));%对应于falcone论文约束处理的矩阵A,求克罗内克积
%     Ut=kron(ones(Nc,1),U);%此处感觉论文里的克罗内科积有问题,暂时交换顺序
%     umin=[-0.2;-0.54;];%维数与控制变量的个数相同
%     umax=[0.2;0.332];
%     delta_umin=[0.05;-0.0082;];
%     delta_umax=[0.05;0.0082];
%     Umin=kron(ones(Nc,1),umin);
%     Umax=kron(ones(Nc,1),umax);
%     A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
%     b_cons_cell={Umax-Ut;-Umin+Ut};
%     A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
%     b_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值
%    % 状态量约束
%     M=10; 
%     delta_Umin=kron(ones(Nc,1),delta_umin);
%     delta_Umax=kron(ones(Nc,1),delta_umax);
%     lb=[delta_Umin;0];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
%     ub=[delta_Umax;M];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子
    
    %% 开始求解过程
    A_cons = [];
    b_cons = [];
    options = optimset('Algorithm','active-set');
    %options = optimset('Algorithm','interior-point-convex'); 
    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
    %% 计算输出
    u_piao(1)=X(1);
    u_piao(2)=X(2);
    u_piao(3)=X(3);
    u_piao(4)=X(4);
    U(1)=kesi(5)+u_piao(1);%用于存储上一个时刻的控制量
    U(2)=kesi(6)+u_piao(2);
    U(3)=kesi(7)+u_piao(3);
    U(4)=kesi(8)+u_piao(4);
    u_real(1)=U(1);
    u_real(2)=U(2);
    u_real(3)=U(3);
    u_real(4)=U(4);
    sys= u_real;
% End of mdlOutputs.