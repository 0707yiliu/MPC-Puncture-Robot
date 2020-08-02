clc;
clear;
close all;
load('MPC_KalmanOfflineData.mat');
BlockMovementTarget = BlockMovementTarget.data;
BlocksMovement = BlocksMovement.data;
BlocksMovementReality = BlocksMovementReality.data;
%四个电机控制两个Block，两Blocks存在XY共4组数据（Nx4）
%前两组数据是Block1的x1,y1
%后两组数据是Block2的x2,y2
X1_Target = BlockMovementTarget(:,1);X1_MPC = BlocksMovement(:,1);X1_reality = BlocksMovementReality(:,1);
Y1_Target = BlockMovementTarget(:,2);Y1_MPC = BlocksMovement(:,2);Y1_reality = BlocksMovementReality(:,2);
X2_Target = BlockMovementTarget(:,3);X2_MPC = BlocksMovement(:,3);X2_reality = BlocksMovementReality(:,3);
Y2_Target = BlockMovementTarget(:,4);Y2_MPC = BlocksMovement(:,4);Y2_reality = BlocksMovementReality(:,4);

figure(1);
plot(X1_reality,'LineWidth',1);hold on;
plot(X1_MPC,'LineWidth',3);hold on;
plot(X1_Target,'LineWidth',3);grid;
legend('X1-reality','X1-MPC','X1-Target');

%% 针末端坐标信息
%线性模型：利用MPC方法比较主要利用模型辨识方法离线或者在线（有一定的问题）辨识线性系统模型，带入MPC控制器做闭环控制
%闭环：通过实际机器人模型输出，再经过状态观测器反馈回MPC控制器更新控制率u
%已获得控制率u以及方块输出轨迹，在此考虑方块移动后对针的方向以及end-effector位置影响。

%方向角度暂时不考虑，考虑end-effector位置轨迹变化，理由：针和球之间机械硬件条件足够的情况下（金属构件，特殊轴承润滑保持最小摩擦，针的刚度够大），用三角函数直接求位置

[N,P] = size(X1_MPC);
d = 3;
H = 5;
for i = 1:1:N
    [X,Y,Z] = MPC_KalmanOfflineDataPlot_Output(X1_Target(i),X2_Target(i),Y1_Target(i),Y2_Target(i));
    X_Target(i) = X;
    Y_Target(i) = Y;
    Z_Target(i) = Z;
    [X,Y,Z] = MPC_KalmanOfflineDataPlot_Output(X1_MPC(i),X2_MPC(i),Y1_MPC(i),Y2_MPC(i));
    X_MPC(i) = X;
    Y_MPC(i) = Y;
    Z_MPC(i) = Z;
    [X,Y,Z] = MPC_KalmanOfflineDataPlot_Output(X1_reality(i),X2_reality(i),Y1_reality(i),Y2_reality(i));
    X_reality(i) = X;
    Y_reality(i) = Y;
    Z_reality(i) = Z;
end
figure(2);
plot(Z_reality,'LineWidth',1);hold on;
plot(Z_MPC,'LineWidth',3);hold on;
plot(Z_Target,'LineWidth',3);grid;
legend('Z-reality','Z-MPC','Z-Target');
