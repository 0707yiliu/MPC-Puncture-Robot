clear;
clc;
h = 5;
dh = 3;
n = 0;
% u = [-2,0,-2,0];
% % point = [];
% % interval = 0.2;
% % for x1 = -2:interval:2
% %     for x2 = -2:interval:2
% %         for y1 = -2:interval:2
% %             for y2 = -2:interval:2
% %                 n = n + 1;
% %                 point(n,1) = y2 + h * sin(atan2((y1 + y2),d));
% %                 point(n,2) = -x1 - h * cos(atan2((y1 + y2),d)) * sin(atan2((x1 + x2),d));
% %                 point(n,3) = - h * cos(atan2((x1 + x2),d)) * cos(atan2((y1 + y2),d));
% %             end
% %         end
% %     end
% % end
% % scatter3(point(:,1),point(:,2),point(:,3),'.');
% 
% % surf(point(:,1),point(:,2),point(:,3));
% % % 
% % T=[       0,         -cos(a2),          sin(a2),           d2 + h*sin(a2);
% %  cos(a1), -sin(a1)*sin(a2), -cos(a2)*sin(a1), - d1 - h*cos(a2)*sin(a1);
% %  sin(a1),  cos(a1)*sin(a2),  cos(a1)*cos(a2),        h*cos(a1)*cos(a2);
% %        0,                0,                0,                        1];
% 
% interval = 0.2;
% for i = -2:interval:2
%     for j = -2:interval:2
%         for k = -2:interval:2
%             for l = -2:interval:2
%                 if i <= 0 && j > 0 
%                     k=abs(k);l=-abs(l);
%                 elseif i<=0 && j <= 0
%                     k=abs(k);l=abs(l);
%                 elseif i>0 && j <= 0
%                     k=-abs(k);l=abs(l);
%                 else
%                     k=-abs(k);l=-abs(l);
%                 end
%         u=[i,k,j,l];
%         x1 = u(1);x2 = u(2);y1 = u(3); y2 = u(4);
%         n=n+1;
% % T01=[1,0,0,0;0,cos(atan2((x1+x2),d)),-sin(atan2((x1+x2),d)),0;0,sin(atan2((x1+x2),d)),cos(atan2((x1+x2),d)),x1;0,0,0,1];
% % T1m=[0,0,1,0;1,0,0,0,;0,1,0,0;0,0,0,1];
% % Tmm1=[0,1,0,0;-1,0,0,0;0,0,1,0;0,0,0,1];
% % Tm12=[1,0,0,0;0,cos(atan2((y1+y2),d))];
% forwardx = sign(x1);forwardy = sign(y1);
% 
% X = abs(x1) + abs(x2);
% Y = abs(y1) + abs(y2);
% L = sqrt(X^2+Y^2);
% theta = atan2(L,d);
% H = h * sin(theta);
% z = -h * cos(theta);
% beta = atan2(Y,X);
% y = -forwardy * (H * sin(beta) + abs(y2));
% x = -forwardx * (H * cos(beta) + abs(x2));
% hold(n,1)=x;
% hold(n,2)=y;
% hold(n,3)=z;
%             end
%         end
%     end
% end
% scatter3(hold(:,1),hold(:,2),hold(:,3),'.');
% xlabel('x');ylabel('y');zlabel('z');

X = 2.719; Y = 4.102; Z = -3.102;
forwardx = sign(X);forwardy=sign(Y);
y1 = -forwardy * 2;x1 = -forwardx * 2;
B = h^2 - Z^2;
D = real((dh * tan(acos(abs(Z)/h))))^2;
a = x1; b = y1;d = Y; c = X;
E = (B - D - c^2 + a^2 - d^2 + b^2) / 2 ;
F = a - c;
G = b - d;


EA = (F^2 + G^2);
EB = (2*G*d*F - 2*(G^2)*c - 2*E*F);
EC = (G^2)*(c^2) + (G^2)*(d^2) - 2*G*d*E + (E^2) - B*(G^2);
x2 = (-EB + real(sqrt(EB^2 - 4*EA*EC))) / (2*EA)
y2 = (E-F*x2)/G
EB^2 - 4*EA*EC
