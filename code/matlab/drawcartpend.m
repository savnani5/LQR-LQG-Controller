function drawcartpend(y,m1,M,l1,l2)
% kinematics
x = y(1);
th1 = y(3);
th2 = y(5);


% dimensions
% L = 2;  % pendulum length
W = 1*sqrt(M/2);  % cart width
H = .5*sqrt(M/5); % cart height
wr = .3; % wheel radius
mr = .4*sqrt(m1/2); % mass radius

% positions
% y = wr/2; % cart vertical position
y = wr/2+H/2; % cart vertical position
w1x = x-.9*W/2;
w1y = 0;
w2x = x+.9*W/2-wr;
w2y = 0;

px1 = x + l1*sin(th1);
py1 = y - l1*cos(th1);
px2 = x + l2*sin(th2);
py2 = y - l2*cos(th2);

plot([-100 100],[0 0],'k','LineWidth',2)
hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1])
rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])
rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])

plot([x px1],[y py1],'k','LineWidth',2)
plot([x px2],[y py2],'k','LineWidth',2)


rectangle('Position',[px1-mr/2,py1-mr/2,mr,mr],'Curvature',1,'FaceColor',[.1 0.1 1])
rectangle('Position',[px2-mr/2,py2-mr/2,mr,mr],'Curvature',1,'FaceColor',[.1 0.1 1])

% set(gca,'YTick',[])
% set(gca,'XTick',[])
xlim([-100 100]);
ylim([-30 20]);
set(gcf,'Position',[100 100 500 500])
% title LQR Controller
% title LQG Controller
title Luenberger Observer
% box off
drawnow
hold off