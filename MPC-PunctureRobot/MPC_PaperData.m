clc;
close all;
clear;

load('MPC-PID-Data-250ms');
Y1_MPC_250ms = Y1_MPC;
Y1_pid_250ms = Y1_pid;
Y1_reality_250ms = Y1_reality;
Y_MPC_250ms = Y_MPC;
Y_pid_250ms = Y_pid;
Y_reality_250ms = Y_reality;
Z_MPC_250ms = Z_MPC;
Z_pid_250ms = Z_pid;

load('MPC-PID-Data-150ms');
Y1_MPC_150ms = Y1_MPC;
Y1_pid_150ms = Y1_pid;
Y1_reality_150ms = Y1_reality;
Y_MPC_150ms = Y_MPC;
Y_pid_150ms = Y_pid;
Y_reality_150ms = Y_reality;
Z_MPC_150ms = Z_MPC;
Z_pid_150ms = Z_pid;
load('MPC-PID-Data-50ms');
Y1_MPC_50ms = Y1_MPC;
Y1_pid_50ms = Y1_pid;
Y1_reality_50ms = Y1_reality;
Y_MPC_50ms = Y_MPC;
Y_pid_50ms = Y_pid;
Y_reality_50ms = Y_reality;
Z_MPC_50ms = Z_MPC;
Z_pid_50ms = Z_pid;
load('MPC-PID-Data-0ms');
Y1_MPC_0ms = Y1_MPC;
Y1_pid_0ms = Y1_pid;
Y1_reality_0ms = Y1_reality;
Y_MPC_0ms = Y_MPC;
Y_pid_0ms = Y_pid;
Y_reality_0ms = Y_reality;
Z_MPC_0ms = Z_MPC;
Z_pid_0ms = Z_pid;
Z_reality_0ms = Z_reality;

figure(1);
% plot(Y1_reality_250ms,'LineWidth',1);hold on;
plot(Y1_MPC_250ms,'-^','LineWidth',1);hold on;
plot(Y1_pid_250ms,'LineWidth',1);hold on;
% plot(Y1_reality_150ms,'LineWidth',1);hold on;
plot(Y1_MPC_150ms,'--','LineWidth',2);hold on;
plot(Y1_pid_150ms,'LineWidth',2);hold on;
% plot(Y1_reality_50ms,'LineWidth',1);hold on;
plot(Y1_MPC_50ms,'-.','LineWidth',1);hold on;
plot(Y1_pid_50ms,'LineWidth',2);hold on;
plot(Y1_reality_0ms,'LineWidth',2);hold on;
plot(Y1_MPC_0ms,'LineWidth',2);hold on;
plot(Y1_pid_0ms,'LineWidth',2);hold on;
plot(Y1_Target,'black','LineWidth',2);grid;
legend('Y1-MPC-TD250ms','Y1-pid-TD250ms',...
       'Y1-MPC-TD150ms','Y1-pid-TD150ms',...
       'Y1-MPC-TD50ms','Y1-pid-TD50ms',...
       'Y1-OpenLoop-noTD','Y1-MPC-noTD','Y1-pid-noTD','Desire');
set(gca,'XTick',0:100:1000);
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'});
xlabel('time(in Seconds)');
ylabel('Y Coordinate(cm)');
axis([0,1000,-5,5]);

   
figure(2);
% plot(Y1_reality_250ms,'LineWidth',1);hold on;
plot(Y_MPC_250ms,'-^','LineWidth',1);hold on;
plot(Y_pid_250ms,'LineWidth',1);hold on;
% plot(Y1_reality_150ms,'LineWidth',1);hold on;
plot(Y_MPC_150ms,'--','LineWidth',2);hold on;
plot(Y_pid_150ms,'LineWidth',2);hold on;
% plot(Y1_reality_50ms,'LineWidth',1);hold on;
plot(Y_MPC_50ms,'-.','LineWidth',1);hold on;
plot(Y_pid_50ms,'LineWidth',2);hold on;
plot(Y_reality_0ms,'LineWidth',2);hold on;
plot(Y_MPC_0ms,'LineWidth',2);hold on;
plot(Y_pid_0ms,'LineWidth',2);hold on;
plot(Y_Target,'black','LineWidth',2);grid;
legend('Y-MPC-TD250ms','Y-pid-TD250ms',...
       'Y-MPC-TD150ms','Y-pid-TD150ms',...
       'Y-MPC-TD50ms','Y-pid-TD50ms',...
       'Y-OpenLoop-noTD','Y-MPC-noTD','Y-pid-noTD','Desire');
set(gca,'XTick',0:100:1000);
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'});
xlabel('time(in Seconds)');
ylabel('Y Coordinate(cm)');
axis([0,1000,-10,10]);
   
   
figure(3);
% plot(Y1_reality_250ms,'LineWidth',1);hold on;
plot(Z_MPC_250ms,'-^','LineWidth',1);hold on;
plot(Z_pid_250ms,'LineWidth',1);hold on;
% plot(Y1_reality_150ms,'LineWidth',1);hold on;
plot(Z_MPC_150ms,'--','LineWidth',2);hold on;
plot(Z_pid_150ms,'LineWidth',2);hold on;
% plot(Y1_reality_50ms,'LineWidth',1);hold on;
plot(Z_MPC_50ms,'-.','LineWidth',1);hold on;
plot(Z_pid_50ms,'LineWidth',2);hold on;
plot(Z_reality_0ms,'LineWidth',2);hold on;
plot(Z_MPC_0ms,'LineWidth',2);hold on;
plot(Z_pid_0ms,'LineWidth',2);hold on;
plot(Z_Target,'black','LineWidth',2);grid;
legend('Z-MPC-TD250ms','Z-pid-TD250ms',...
       'Z-MPC-TD150ms','Z-pid-TD150ms',...
       'Y-MPC-TD50ms','Z-pid-TD50ms',...
       'Z-OpenLoop-noTD','Z-MPC-noTD','Z-pid-noTD','Desire');
set(gca,'XTick',0:100:1000);
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'});
xlabel('time(in Seconds)');
ylabel('Z Coordinate(cm)');
axis([0,1000,-6,-1]);

load('KalmanPredictionData');
X2 = simout1.data;
outparameters = simout.data;
a1 = outparameters(:,1);
a2 = outparameters(:,2);
b = outparameters(:,3);
X2_Target = X2(:,1);
X2_Prediction = X2(:,2);
Err = X2_Prediction - X2_Target;
figure(4);
plot(a1,'LineWidth',3);hold on;
plot(a2,'LineWidth',3);hold on;
plot(b,'LineWidth',3);grid;
legend('a1','a2','b');
set(gca,'XTick',0:100:1000);
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'});
xlabel('time(in Seconds)');
ylabel('SystemParamters');
axis([0,1000,-1,3]);

figure(5);
plot(X2_Target,'LineWidth',1.5);hold on;
plot(X2_Prediction,'LineWidth',1.5);hold on;
plot(Err,'LineWidth',1.5);grid;
legend('Target','Estimate','Error');
set(gca,'XTick',0:100:1000);
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'});
xlabel('time(in Seconds)');
ylabel('Judgement Standard(cm)');
axis([0,1000,-3,3]);

figure(6);
% plot(Y1_reality_250ms,'LineWidth',1);hold on;
plot(Y1_MPC_250ms-Y1_Target,'-^','LineWidth',1);hold on;
plot(Y1_pid_250ms-Y1_Target,'LineWidth',1);hold on;
% plot(Y1_reality_150ms,'LineWidth',1);hold on;
plot(Y1_MPC_150ms-Y1_Target,'--','LineWidth',2);hold on;
plot(Y1_pid_150ms-Y1_Target,'LineWidth',2);hold on;
% plot(Y1_reality_50ms,'LineWidth',1);hold on;
plot(Y1_MPC_50ms-Y1_Target,'-.','LineWidth',1);hold on;
plot(Y1_pid_50ms-Y1_Target,'LineWidth',2);hold on;
plot(Y1_reality_0ms-Y1_Target,'LineWidth',2);hold on;
plot(Y1_MPC_0ms-Y1_Target,'LineWidth',2);hold on;
plot(Y1_pid_0ms-Y1_Target,'LineWidth',2);hold on;
plot(Y1_Target-Y1_Target,'black','LineWidth',2);grid;
legend('Y1-MPC-TD250ms','Y1-pid-TD250ms',...
       'Y1-MPC-TD150ms','Y1-pid-TD150ms',...
       'Y1-MPC-TD50ms','Y1-pid-TD50ms',...
       'Y1-OpenLoop-noTD','Y1-MPC-noTD','Y1-pid-noTD','Desire');
set(gca,'XTick',0:100:1000);
set(gca,'XTickLabel',{'0','1','2','3','4','5','6','7','8','9','10'});
xlabel('time(in Seconds)');
ylabel('Y Coordinate Error(cm)');
axis([0,1000,-4,4]);

Y1_MPC_250 = Y1_MPC_250ms-Y1_Target;
Y1_pid_250 = Y1_pid_250ms-Y1_Target;
Y1_MPC_150 = Y1_MPC_150ms-Y1_Target;
Y1_pid_150 = Y1_pid_150ms-Y1_Target;
Y1_MPC_50 = Y1_MPC_50ms-Y1_Target;
Y1_pid_50 = Y1_pid_50ms-Y1_Target;
Y1_reality_0 = Y1_reality_0ms-Y1_Target;
Y1_MPC_0 = Y1_MPC_0ms-Y1_Target;
Y1_pid_0 = Y1_pid_0ms-Y1_Target;

Y1_MPC_250 = std(Y1_MPC_250)
Y1_pid_250 = std(Y1_pid_250)
Y1_MPC_150 = std(Y1_MPC_150)
Y1_pid_150 = std(Y1_pid_150)
Y1_MPC_50 = std(Y1_MPC_50)
Y1_pid_50 = std(Y1_pid_50)
Y1_reality_0 = std(Y1_reality_0)
Y1_MPC_0 = std(Y1_MPC_0)
Y1_pid_0 = std(Y1_pid_0)
