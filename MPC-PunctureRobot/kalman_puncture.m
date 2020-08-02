clc;
clear;
%线性系统 4输入4输出，用Kalman-Filter辨识系统状态转移矩阵和输出矩阵
%通过系统阶次辨识得到电机单轴2阶单输入单输出系统
N=300;
% sim('kalman_data');
% m1 = m1.data;m2 = m2.data;m3 = m3.data;m4 = m4.data;
% x1 = x1.data;x2 = x2.data;y1 = y1.data;y2 = y2.data;
X = zeros(8,N);%状态观测
% X(8,1) = [];%初始值确定--难点
Xkf=zeros(8,N);%状态辨识结果
Z=zeros(4,N); %输出观测
P=zeros();%测量协方差（方差根据实际观测器误差改变）
Q=0.01;%过程噪声方差
R=0.25;%测量噪声方差
W=sqrt(Q)*randn(8,N);%过程噪声
V=sqrt(R)*randn(1,N);%测量噪声
F=eye(8,8);%8x8状态转移矩阵
G=1;%过程噪声状态转移矩阵
% H=eye(8,8);%4x8状态输出矩阵
