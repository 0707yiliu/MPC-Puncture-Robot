clc;
clear;
close all;
load('MPC-PID-Data-50ms');
BlockMovementTarget = BlocksMovementDesire.data;
BlocksMovement = BlocksMovement.data;
BlocksMovementReality = BlocksMovementReality.data;
BlocksMovementPID = BlocksMovementPID.data;
X1_Target = BlockMovementTarget(:,1);X1_MPC = BlocksMovement(:,1);X1_reality = BlocksMovementReality(:,1);X1_pid = BlocksMovementPID(:,1);
Y1_Target = BlockMovementTarget(:,2);Y1_MPC = BlocksMovement(:,2);Y1_reality = BlocksMovementReality(:,2);Y1_pid = BlocksMovementPID(:,2);
X2_Target = BlockMovementTarget(:,3);X2_MPC = BlocksMovement(:,3);X2_reality = BlocksMovementReality(:,3);X2_pid = BlocksMovementPID(:,3);
Y2_Target = BlockMovementTarget(:,4);Y2_MPC = BlocksMovement(:,4);Y2_reality = BlocksMovementReality(:,4);Y2_pid = BlocksMovementPID(:,4);

figure(1);
plot(X1_reality,'LineWidth',1);hold on;
plot(X1_MPC,'LineWidth',3);hold on;
plot(X1_pid,'LineWidth',3);hold on;
plot(X1_Target,'LineWidth',3);grid;
legend('X1-OpenLoop','X1-MPC','X1-PID','X1-Desire');

[N,P] = size(X1_MPC);
d = 3;
H = 5;
for i = 1:1:N
    [X_Target(i),Y_Target(i),Z_Target(i)] = MPC_KalmanOfflineDataPlot_Output(X1_Target(i),X2_Target(i),Y1_Target(i),Y2_Target(i));
    [X_MPC(i),Y_MPC(i),Z_MPC(i)] = MPC_KalmanOfflineDataPlot_Output(X1_MPC(i),X2_MPC(i),Y1_MPC(i),Y2_MPC(i));
    [X_reality(i),Y_reality(i),Z_reality(i)] = MPC_KalmanOfflineDataPlot_Output(X1_reality(i),X2_reality(i),Y1_reality(i),Y2_reality(i));
    [X_pid(i),Y_pid(i),Z_pid(i)] = MPC_KalmanOfflineDataPlot_Output(X1_pid(i),X2_pid(i),Y1_pid(i),Y2_pid(i));
end
figure(2);
plot(Y_reality,'LineWidth',1);hold on;
plot(Y_MPC,'LineWidth',3);hold on;
plot(Y_pid,'LineWidth',3);hold on;
plot(Y_Target,'LineWidth',3);grid;
legend('Y-OpenLoop','Y-MPC','Y-pid','Y-Target');