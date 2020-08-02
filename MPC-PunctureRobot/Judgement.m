close all
clear
clc
sim('robot_data');
m1 = m1.data;m2 = m2.data;m3 = m3.data;m4 = m4.data;
x1 = x1.data;x2 = x2.data;y1 = y1.data;y2 = y2.data;
%²úÉúMÐòÁÐ
% x=[0 1 0 1 1 0 1 1 1];
% n=1003;
% M=[];
% for i =1:n
%     temp=xor(x(4),x(9));
%     M(i)=x(9);
%     for j=9:-1:2
%         x(j)=x(j-1);
%     end
%     x(1)=temp;
% end

% v=randn(1,1003);
% z=[];
% z(1)=-1;
% z(2)=0;
% L=1000;
% for i=3:L+3
%     z(i)=1.63*z(i-1)-0.68*z(i-2)+0.04*M(i-1)-0.0025*M(i-2)+v(i);
% end
z=x1;M=m1;
%n=1
L=900;
for i=1:L
    H1(i,1)=z(i);
    H1(i,2)=M(i);
end
A=H1'*H1/L;
%n=2
for i =1 :L
    H2(i,1)=z(i+1);
    H2(i,2)=z(i);
    H2(i,3)=M(i+1);
    H2(i,4)=M(i);
end
B=H2'*H2/L;
%n=3
for i = 1:L
    H3(i,1)=z(i+2);
    H3(i,2)=z(i+1);
    H3(i,3)=z(i);
    H3(i,4)=M(i+2);
    H3(i,5)=M(i+1);
    H3(i,6)=M(i);
end
C=H3'*H3/L;
%n=4
for i = 1:L
    H4(i,1)=z(i+3);
    H4(i,2)=z(i+2);
    H4(i,3)=z(i+1);
    H4(i,4)=z(i);
    H4(i,5)=M(i+3);
    H4(i,6)=M(i+2);
    H4(i,7)=M(i+1);
    H4(i,8)=M(i);
end
D=H4'*H4/L;
DR(1)=det(A)/det(B);
DR(2)=det(B)/det(C);
DR(3)=det(C)/det(D);
DR

