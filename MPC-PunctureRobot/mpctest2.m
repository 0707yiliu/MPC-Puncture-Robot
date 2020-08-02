clc;
clear;
%% ������
T = 0.01; %sample time
d = 3; %block distance
h = 5; %length of end-effector
o = 0.2; %pitch
n = o / (2 * pi);
ToDeg = 180 / pi;kalman_temperature
ToRad = pi / 180;
Nx = 4; %number of environment condition
Nu = 4; %number of controller
Np = 40; %prediction step size
Tsim = Np;
Ncc = 20; %control step size
Row = 10; %relax
%% �ο�ģ�͵Ĺ켣����ģ��
%x1,y1���Ϸ����xyƽ������Գ�ʼ���·������Ĺ̶�������ƶ�����
%x2,y2���·����xyƽ������Գ�ʼ���·������Ĺ̶�������ƶ�����
%theta��end-effector����·���ƽ��ĽǶ�
sim('MPC_traj');
x1_re = x1.data;x2_re = x2.data; y1_re = y1.data;y2_re = y2.data;theta = theta.data;
Xout = [x1_re y1_re x2_re y2_re]; % ����״̬
%% MPC��������
%mpc���������ȶ�ϵͳ��Ч���ڴ˽��������˶�ѧģ�;��������Ի�-��ɢ��-����Ԥ��-�Ż��Ĳ������
%���Ի������MPC�Ż�Ŀ��Ϊ״̬��ֵ�Ϳ��Ʋ�ֵ
%��ز��������󣩳�ʼ��
X = [0 0 0 0]; %��ʼ״̬
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
% q = [1 0 0 0 ;0 1 0 0 ;0 0 1 0 ;0 0 0 1];
% Q_cell = cell(Tsim,Tsim);
kesi = zeros(Nx+Nu,1);
% 
% for i = 1:1:Tsim
%     for j = 1:1:Tsim
%         if i == j
%             Q_cell{i,j}=q;
%         else
%             Q_cell{i,j}=zeros(Nx,Nx);
%         end
%     end
% end
% Q = cell2mat(Q_cell); %Ȩ�ؾ���
% R = 0.1*eye(Nu*Tsim,Nu*Tsim); %Ȩ�ؾ���
% xx = zeros(4,1); %��ǰ���ת���µĸ�����
%Ԥ������
for i = 1:1:Nr
    kesi(1) = x_real(i,1) - Xout(i,1);
    kesi(2) = x_real(i,2) - Xout(i,2);
    kesi(3) = x_real(i,3) - Xout(i,3);
    kesi(4) = x_real(i,4) - Xout(i,4);
    kesi(5) = u_real(i,1);
    kesi(6) = u_real(i,2);
    kesi(7) = u_real(i,3);
    kesi(8) = u_real(i,4); %�Ż�Ŀ��kesi
    u_piao = zeros(Nr,Nu);
    Q=100*eye(Nx*Np,Nx*Np);
    R=5*eye(Nu*Ncc);
    a = [1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1]; %Ŀ�����ٶ�Ϊ�㣬���ٶ�Ϊ�㣬�̶�����a
    b = [n*T 0 0 0;
         0 n*T 0 0;
         0 0 n*T 0;
         0 0 0 n*T];
    A_cell = cell(2,2);
    B_cell = cell(2,1);
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
    THETA_cell=cell(Np,Ncc);
    for j=1:1:Np
        PHI_cell{j,1}=C*A^j;
        for k=1:1:Ncc
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
    H_cell{1,1}=THETA'*Q*THETA+R;  
    H_cell{1,2}=zeros(Nu*Ncc,1);
    H_cell{2,1}=zeros(1,Nu*Ncc);
    H_cell{2,2}=Row;
    H=cell2mat(H_cell); 
    error=PHI*kesi;
    f_cell=cell(1,2);
    f_cell{1,1}=2*error'*Q*THETA;
    f_cell{1,2}=0;
    f=cell2mat(f_cell);
    A_t=zeros(Ncc,Ncc);%��falcone���� P181
    for p=1:1:Ncc
        for q=1:1:Ncc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));
%     size(A_I)
    Ut=kron(ones(Ncc,1),x_real(i,:)');
%     size(Ut)
    uminmax = 360;
    delta_uminmax = 100;
    umin=[-ToRad * uminmax;-ToRad *uminmax;-ToRad * uminmax;-ToRad * uminmax];
    umax=[ToRad * uminmax;ToRad * uminmax;ToRad * uminmax;ToRad *uminmax];
    delta_umin=[-ToRad * delta_uminmax;-ToRad * delta_uminmax;-ToRad * delta_uminmax;-ToRad * delta_uminmax];
    delta_umax=[ToRad * delta_uminmax;ToRad * delta_uminmax;ToRad * delta_uminmax;ToRad * delta_uminmax];
    Umin=kron(ones(Ncc,1),umin);
    Umax=kron(ones(Ncc,1),umax);
%     size(Umax)
    A_cons_cell={A_I zeros(Nu*Ncc,1);-A_I zeros(Nu*Ncc,1)};
    b_cons_cell={Umax-Ut;-Umin+Ut};
    A_cons=cell2mat(A_cons_cell);
    b_cons=cell2mat(b_cons_cell);
%     A_cons = [];
%     b_cons = [];
%     v_cons = ToRad * 180;
%     ub = [v_cons; v_cons; v_cons; v_cons];
%     lb = -ub;
    M=10; 
    delta_Umin=kron(ones(Ncc,1),delta_umin);
%     size(delta_Umin)
    delta_Umax=kron(ones(Ncc,1),delta_umax);
    lb=[delta_Umin;0];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
    ub=[delta_Umax;M];
%     pause();
%     options = optimset('Algorithm','active-set');
    %options = optimset('Algorithm','interior-point-convex'); 
    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub);
    u_piao(1)=X(1);
    u_piao(2)=X(2);
    u_piao(3)=X(3);
    u_piao(4)=X(4);
    u_real(i+1,1)=kesi(5)+u_piao(1);%���ڴ洢��һ��ʱ�̵Ŀ�����
    u_real(i+1,2)=kesi(6)+u_piao(2);
    u_real(i+1,3)=kesi(7)+u_piao(3);
    u_real(i+1,4)=kesi(8)+u_piao(4);
    x_real(i+1,1) = x_real(i,1) + n * u_real(i+1,1);
    x_real(i+1,2) = x_real(i,2) + n * u_real(i+1,2);
    x_real(i+1,3) = x_real(i,3) + n * u_real(i+1,3);
    x_real(i+1,4) = x_real(i,4) + n * u_real(i+1,4);
end
figure(1)
plot(u_real(:,1));grid
figure(2)
plot(x_real(:,1));hold on
plot(Xout(:,1));grid

























