function dy = cartpend(y, m1, m2, M, l1, l2, g, d, u)
% Non-linear ODE's for the system

S1y = sin(y(3));
S2y = sin(y(5));
C1y = cos(y(3));
C2y = cos(y(5));

D = M + m2*(S2y^2) + m1*(S1y^2);

dy(1,1) = y(2);
dy(2,1) = (1/D)*(u - m1*g*C1y*S1y - m2*g*C2y*S2y - m1*l1*S1y*(y(4)^2) - m2*l2*S2y*(y(5)^2));
dy(3,1) = y(4);
dy(4,1) = (dy(2,1)*C1y - g*S1y)/l1;
dy(5,1) = y(6);
dy(6,1) = (dy(2,1)*C2y - g*S2y)/l2;