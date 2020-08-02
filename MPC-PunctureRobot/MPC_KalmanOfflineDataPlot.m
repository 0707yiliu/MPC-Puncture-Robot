clc;
clear;
close all;
load('MPC_KalmanOfflineData.mat');
BlockMovementTarget = BlockMovementTarget.data;
BlocksMovement = BlocksMovement.data;
BlocksMovementReality = BlocksMovementReality.data;
%�ĸ������������Block����Blocks����XY��4�����ݣ�Nx4��
%ǰ����������Block1��x1,y1
%������������Block2��x2,y2
X1_Target = BlockMovementTarget(:,1);X1_MPC = BlocksMovement(:,1);X1_reality = BlocksMovementReality(:,1);
Y1_Target = BlockMovementTarget(:,2);Y1_MPC = BlocksMovement(:,2);Y1_reality = BlocksMovementReality(:,2);
X2_Target = BlockMovementTarget(:,3);X2_MPC = BlocksMovement(:,3);X2_reality = BlocksMovementReality(:,3);
Y2_Target = BlockMovementTarget(:,4);Y2_MPC = BlocksMovement(:,4);Y2_reality = BlocksMovementReality(:,4);

figure(1);
plot(X1_reality,'LineWidth',1);hold on;
plot(X1_MPC,'LineWidth',3);hold on;
plot(X1_Target,'LineWidth',3);grid;
legend('X1-reality','X1-MPC','X1-Target');

%% ��ĩ��������Ϣ
%����ģ�ͣ�����MPC�����Ƚ���Ҫ����ģ�ͱ�ʶ�������߻������ߣ���һ�������⣩��ʶ����ϵͳģ�ͣ�����MPC���������ջ�����
%�ջ���ͨ��ʵ�ʻ�����ģ��������پ���״̬�۲���������MPC���������¿�����u
%�ѻ�ÿ�����u�Լ���������켣���ڴ˿��Ƿ����ƶ������ķ����Լ�end-effectorλ��Ӱ�졣

%����Ƕ���ʱ�����ǣ�����end-effectorλ�ù켣�仯�����ɣ������֮���еӲ�������㹻������£�������������������󻬱�����СĦ������ĸնȹ��󣩣������Ǻ���ֱ����λ��

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
