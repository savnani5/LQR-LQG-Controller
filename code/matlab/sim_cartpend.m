clear all, close all, clc

M = 1000;
m1 = 100;
m2= 100;
l1 = 20;
l2 = 10;
g = 10;
d = 1;

tspan = 0:.1:100;
y0 = [0; 0.5; 0; 0.2; 0; 0.2];
[t,y] = ode45(@(t,y)cartpend(y, m1, m2, M, l1, l2, g, 0, 0),tspan,y0);

for k=1:length(t)
    drawcartpend(y(k,:),m1,M,l1,l2);
end
