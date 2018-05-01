%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
% maze
circ = -pi : 0.01 : pi;
figure(1);
line(20*cos(circ),20*sin(circ),'linewidth',1.5);
line(15*cos(circ(1:160)),15*sin(circ(1:160)),'linewidth',1.5);
line(15*cos(circ(211:370)),15*sin(circ(211:370)),'linewidth',1.5);
line(15*cos(circ(421:580)),15*sin(circ(421:580)),'linewidth',1.5);
line(10*cos(circ(51:210)),10*sin(circ(51:210)),'linewidth',1.5);
line(10*cos(circ(261:420)),10*sin(circ(261:420)),'linewidth',1.5);
line(10*cos(circ(471:629)),10*sin(circ(471:629)),'linewidth',1.5);
line(5*cos(circ(26:604)),5*sin(circ(26:604)),'linewidth',1.5);
hold on; 
figure(1);
scatter(0,0,'linewidth',10);
hold on;

% ball
ball_x = [];
ball_y = [];
ball_x = [ball_x,-(17.5-linspace(0,1.5,20)).*cos(linspace(0,pi/2,20))];
ball_y = [ball_y,-(17.5-linspace(0,1.5,20)).*sin(linspace(0,pi/2,20))];

x_1 = ball_x(end);
y_1 = ball_y(end);
r_1 = sqrt(x_1^2+y_1^2);
ball_x = [ball_x,-(r_1-linspace(0,3,20)).*cos(linspace(pi/2,pi/2+pi/3,20))];
ball_y = [ball_y,-(r_1-linspace(0,3,20)).*sin(linspace(pi/2,pi/2+pi/3,20))];

x_2 = ball_x(end);
y_2 = ball_y(end);
r_2 = sqrt(x_2^2+y_2^2);
ball_x = [ball_x,-(r_2-linspace(0,2,20)).*cos(linspace(pi/2+pi/3,pi/2+pi/3+pi/2,20))];
ball_y = [ball_y,-(r_2-linspace(0,2,20)).*sin(linspace(pi/2+pi/3,pi/2+pi/3+pi/2,20))];

x_3 = ball_x(end);
y_3 = ball_y(end);
r_3 = sqrt(x_3^2+y_3^2);
ball_x = [ball_x,-(r_3-linspace(0,4,15)).*cos(linspace(pi/2+pi/3+pi/2,pi/2+pi/3+pi-pi/12,15))];
ball_y = [ball_y,-(r_3-linspace(0,4,15)).*sin(linspace(pi/2+pi/3+pi/2,pi/2+pi/3+pi-pi/12,15))];

x_4 = ball_x(end);
y_4 = ball_y(end);
r_4 = sqrt(x_4^2+y_4^2);
ball_x = [ball_x,-(r_4-linspace(0,1.5,50)).*cos(linspace(7/4*pi,7/4*pi+2*pi+pi/6,50))];
ball_y = [ball_y,-(r_4-linspace(0,1.5,50)).*sin(linspace(7/4*pi,7/4*pi+2*pi+pi/6,50))];

x_5 = ball_x(end);
y_5 = ball_y(end);
r_5 = sqrt(x_5^2+y_5^2);
ball_x = [ball_x,-(r_5-linspace(0,r_5,15)).*cos(linspace(7/4*pi+2*pi+pi/6,7/4*pi+2*pi+pi/6+pi,15))];
ball_y = [ball_y,-(r_5-linspace(0,r_5,15)).*sin(linspace(7/4*pi+2*pi+pi/6,7/4*pi+2*pi+pi/6+pi,15))];

ball_path = [ball_x;ball_y];
ball_size = length(ball_path);
figure(1);
scatter(ball_path(1,:),ball_path(2,:));
%%
[ legs_path ] = ball2leg( ball_path );

figure(2);
subplot(3,1,1);
plot(legs_path(1,:));
title('h_1');
subplot(3,1,2);
plot(legs_path(2,:));
title('h_2');
subplot(3,1,3);
plot(legs_path(3,:));
title('h_3');
