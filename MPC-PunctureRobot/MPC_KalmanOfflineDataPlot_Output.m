function [x,y,z] = MPC_KalmanOfflineDataPlot_Output(X1_Target,X2_Target,Y1_Target,Y2_Target)
    d = 3;
    h = 5;
    forwardx = sign(X1_Target);forwardy = sign(Y1_Target);
    X = abs(X1_Target) + abs(X2_Target);
    Y = abs(Y2_Target) + abs(Y1_Target);
    L = sqrt(X^2+Y^2);
    theta = atan2(L,d);
    H = h * sin(theta);
    z = -h * cos(theta);
    beta = atan2(Y,X);
    y = -forwardy * (H * sin(beta) + abs(Y2_Target));
    x = -forwardx * (H * cos(beta) + abs(X2_Target));
end
