% desied_dataX = x1.Data;
% 
% out_bppid_dataX = x.Data;
% 
% out_pid_dataX = xx.Data;
% 
% desied_dataZ = z1.Data;
% 
% out_bppid_dataZ = z.Data;
% 
% out_pid_dataZ = zz.Data;
% 
% desied_dataY = y1.Data;
% 
% out_bppid_dataY = y.Data;
% 
% out_pid_dataY = yy.Data;
% 
% 

% save Data
desied_dataX = x1.Data;
out_bppid_dataX = x.Data;
desied_dataY = y1.Data;
out_bppid_dataY = y.Data;
desied_dataZ = z1.Data;
out_bppid_dataZ = z.Data;
plot(desied_dataX,'LineWidth',3);hold on;
plot(out_bppid_dataX,'LineWidth',3);hold on;grid
figure(2)
plot(desied_dataY,'LineWidth',3);hold on;
plot(out_bppid_dataY,'LineWidth',3);hold on;grid
figure(3)
plot(desied_dataZ,'LineWidth',3);hold on;
plot(out_bppid_dataZ,'LineWidth',3);hold on;grid



% plot(desied_dataX,'LineWidth',3);hold on;
% plot(desied_dataY,'LineWidth',3);hold on;
% plot(desied_dataZ,'LineWidth',3);hold on;
% plot(out_bppid_dataX,'LineWidth',3);hold on;
% plot(out_bppid_dataY,'LineWidth',3);hold on;
% plot(out_bppid_dataZ,'LineWidth',3);hold on;
% plot(out_pid_dataX,'LineWidth',3);hold on;
% plot(out_pid_dataY,'LineWidth',3);hold on;
% plot(out_pid_dataZ,'LineWidth',3);hold on;

% %%
% desied_dataY = desiredY.Data;
% 
% out_pid_dataY = out_pid_y1.Data;
% out_pre_dataY = out_pre1.Data;
% % smooth(out_pid_dataY);
% figure(1);
% 
% plot(out_pre_dataY,'LineWidth',3);hold on;
% plot(smooth(out_pid_dataY),'LineWidth',3);hold on;
% plot(desied_dataY,'LineWidth',1.5);grid;hold on
% legend('out-pre','out-pid','desired');
% desied_dataZ = desiredZ.Data;
% axis([0 500/2 0 4]);
% xlabel('Time');ylabel('Position of the End-effector(Y-axis)/cm');
% %%
% out_pid_dataZ = out_pid_z1.Data;
% out_pre_dataZ = out_prez1.Data;
% 
% figure(2);
% 
% plot(out_pre_dataZ,'LineWidth',3);hold on;
% plot(out_pid_dataZ,'LineWidth',3);hold on;
% plot(desied_dataZ,'LineWidth',1.5);grid;hold on
% legend('out-pre','out-pid','desired');
% axis([0 500/2 0 1.5]);
% 
% xlabel('Time');ylabel('Position of the End-effector(Z-axis)/cm');
