%% LQG controller
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

% smallest observable output vector
C = [1 0 0 0 0 0];

D = zeros(size(C,1), size(B,2));

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

%%  Augment system with disturbances and noise
Vd = .001*eye(size(A));  % disturbance covariance
Vn = .001;       % noise covariance

%%  Build Kalman filter
[L,P,E] = lqe(A,Vd,C,Vd,Vn);  % design Kalman filter
sysKF = ss(A-L*C,[B L],eye(size(A)),0*[B L]);  % Kalman filter estimator

%%  LQG applied to linearized system 
A_final = [(A-B*K) B*K; 
            zeros(size(A)),(A*L*C)];

B_final = [B; zeros(size(B))];
C_final = [C zeros(size(C))];
D_final = 0;

lqg = ss(A_final, B_final, C_final, D_final);

tspan = 0:.1:100;
y0 = [0; 0.5; 0; 0.2; 0; 0.2];  % initial state
input = ones(size(tspan));      % Step input
[y, t, x] = lsim(lqg, input, tspan, [y0;y0]);

num_states = 6;
out_state = x(:, 1:num_states);
error = x(:, num_states+1:end);
state_estimate = out_state - error;

%% System animation

% for k=1:length(t)
%     drawcartpend(out_state(k,:),m1,M,l1,l2);
% end

%% Plotting the system response
hold on
subplot(3,1,1);
plot(t,out_state(:,1),'r', t,state_estimate(:,1),'--b')
legend('x response', 'x estimate')
title 'LQG Controller reponse';

subplot(3,1,2);
plot(t,out_state(:,3),'r', t,state_estimate(:,3),'--b')
legend('theta1 response', 'theta1 estimate')
title 'LQG Controller reponse';

subplot(3,1,3);
plot(t,out_state(:,5),'r', t,state_estimate(:,5),'--b');
legend('theta2 response', 'theta2 estimate')
title 'LQG Controller reponse';
