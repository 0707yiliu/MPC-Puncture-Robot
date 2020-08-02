function theta = RobotForward(u,d)
    x1 = u(1);y1 = u(2);
    x2 = u(3);y2 = u(4);
    X = abs(x1) + abs(x2);
    Y = abs(y1) + abs(y2);
    L = sqrt(X^2+Y^2);
    theta = atan2(L,d);