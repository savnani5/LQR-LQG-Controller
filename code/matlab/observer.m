%% Luenberger Observer
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

C_init = eye(6);

D = zeros(size(C_init,1), size(B,2));

ss_openloop = ss(A,B,C_init,D);

%% Testing observer with different output vectors
% Case 1 - x
% C = [1 0 0 0 0 0];

% % Case 2 - theta1, theta2 - system not observable
% C = [0 0 1 0 0 0;
%      0 0 0 0 1 0];
% 
% % Case 3 - x, theta2
C = [1 0 0 0 0 0;
     0 0 0 0 1 0];

% % Case 4 - x, theta1, theta2
% C = [1 0 0 0 0 0;
%      0 0 1 0 0 0;
%      0 0 0 0 1 0];

D = zeros(size(C,1), size(B,2));

rank(obsv(A,C))   % is it observable ?
%%  Pole placement

% p is a vector of desired eigenvalues
% p = [-.01; -.02; -.03; -.04; -.05; -.06];   % Delayed Tracking 
p = [-1.1; -1.2; -1.3; -1.4; -1.5; -1.6];     % Good Tracking

L = place(A',C',p)';

AO = A-L*C;
BO = [B L];
CO = eye(size(A));
DO = zeros(size(CO,1), size(BO,2));

%% Observer applied to linearized system
ss_open_loop_buffer = ss(A, B, C, D);
ss_with_feedback = ss(AO, BO, CO, DO);


tspan = 0:.1:100;
y0 = [0; 0.5; 0; 0.2; 0; 0.2];  % initial state
input = ones(size(tspan));      % Step input
[y_openloop, t] = lsim(ss_openloop, input, tspan, y0);
[y_openloop_buffer, t] = lsim(ss_open_loop_buffer, input, tspan, y0);
[x_estimated, t] = lsim(ss_with_feedback, [input; y_openloop_buffer'], tspan, y0);


% Checking volume of the gramian to see which C is the most observable
% det(gram(ss_with_feedback,'o'))
%% System animation

for k=1:length(t)
    drawcartpend(y_openloop_buffer ,m1,M,l1,l2);
end

%% Plotting the system response
% hold on
% subplot(3,1,1);
% plot(t,y_openloop(:,1),'r', t,x_estimated(:,1),'--b')
% legend('x response', 'x-estimate')
% title 'Luenberg Observer';
% 
% subplot(3,1,2);
% plot(t,y_openloop(:,3),'r', t,x_estimated(:,3),'--b')
% legend('theta1', 'theta1-estimate')
% title 'Luenberg Observer';
% 
% subplot(3,1,3);
% plot(t,y_openloop(:,5),'r', t,x_estimated(:,5),'--b');
% legend('theta2', 'theta2-estimate')
% title 'Luenberg Observer';
