%% LQR Controller
clear all, close all, clc

% Variables
M = 1000;
m1 = 100;
m2= 100;
l1 = 20;
l2 = 10;
g = 10;
d = 1;


A = [0 1 0 0 0 0;
    0 0 (-m1*g)/M 0 (-m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 ((-m1*g)/(M*l1))-(g/l1) 0 (-m2*g)/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 (-m1*g)/(M*l2) 0 ((-m2*g)/(M*l2))-(g/l2) 0];

B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];

C = eye(6);

D = zeros(size(C,1), size(B,2));

eig(A)

rank(ctrb(A,B))  % is it controllable ?

%% LQR controller
Q = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1000 0 0 0;
    0 0 0 100 0 0
    0 0 0 0 1000 0
    0 0 0 0 0 10000];

R = .0001;

K = lqr(A,B,Q,R);
disp(K)

%% Controller applied to linearized system
Ac = A-B*K;
controlled_system = ss(Ac,B,C,D);
tspan = 0:.1:100;
y0 = [0; 0.5; 0; 0.2; 0; 0.2];
[y, t] = initial(controlled_system,y0);


%% Controller applied to non-linear system
% tspan = 0:.1:100;
% y0 = [0; 0.5; 0; 0.2; 0; 0.2];
% [t,y] = ode45(@(t,y)cartpend(y,m1,m2,M,l1,l2,g,d,-K*(y-[0; 0; 0; 0; 0; 0])),tspan,y0);


%% System animation

% for k=1:length(t)
%     drawcartpend(y(k,:),m1,M,l1,l2);
% end


%% Plotting the system response
hold on
plot(t,y)
% plot(t,y(:,5))
% plot(t,y(:,6))
legend('x', 'x-dot', 'theta1', 'theta1-dot', 'theta2', 'theta2-dot')
title 'LQR Controller Response';


