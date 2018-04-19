% Lab3
% Peiguang Wang, Sichao Zhang

function [  ] = simulateRR(  )
% MECH 498 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function should run a dynamic simulation of the RR
%    robot for this assignment, and then play a video of the resulting
%    robot motion.
%
%
%    ADDITIONAL CODE NEEDED: lots
%    

close all;

% Initialize robot
robot = RRInit();

m_1 = robot.m_1;
m_2 = robot.m_2;
m_r1 = robot.m_r1;
m_r2 = robot.m_r2;
l_1 = robot.l_1;
l_2 = robot.l_2;
g = robot.g;

M1 = m_1 + m_r1; 
M2 = m_2 + m_r2;

Lc1 = ((m_1+1/2*m_r1)*l_1)/(M1);
Lc2 = ((m_2+1/2*m_r2)*l_2)/(M2);

% calculate inertial mass
I1 = 1/12*m_r1*l_1^2 + m_1*(l_1/2)^2;
I2 = 1/12*m_r2*l_2^2 + m_2*(l_2/2)^2;

% Joint Torque Limit
tau_max = 20; % [N-m] (Scalar)

% Time
dt = 0.08; % [s]
t_f = 10; % [s]

% Initial Conditions
X_0 = [pi/3, pi/2, 0, 0]; % [p1, p2, v1, v2]

% Control Gains (Scalar)
K_p = 100;
K_v = 50;

% Control variables
theta_d = [0,pi/2];

% Numerical Integration
t = 0:dt:t_f;
X = zeros(length(t),4); % initialize variable to hold state vector
X_dot = zeros(length(t),4); % initialize variable to hold state vector derivatives

k = zeros(1,length(t));
u = zeros(1,length(t));
energy = zeros(1,length(t));

for i = 1:length(t)
    if i == 1
        X(i,:) = X_0;
    else
        X(i,:) = X(i-1,:);
    end
    theta = X(i,1:2);
    theta_dot = X(i,3:4);
    
    % Control torques
    tau = -K_p*(theta - theta_d)' - K_v*(theta_dot');
    
    % Apply joint torque limits
    tau(tau>tau_max) = tau_max;
    tau(tau<-tau_max) = -tau_max;
    
    % Dynamic Model
    M = [M1*Lc1^2 + M2*(l_1 + Lc2*cos(X(i,2)))^2 + (I1 + I2), 0;
        0                                                  , M2*Lc2^2 + I2];
    C = [-2*M2*Lc2*sin(X(i,2))*(l_1 + Lc2*cos(X(i,2)))*X(i,3)*X(i,4);
        M2*Lc2*sin(X(i,2))*(l_1 + Lc2*cos(X(i,2)))*X(i,3)^2];
    G = [0;
        M2*g*Lc2*cos(X(i,2))];

    theta_dot2 = inv(M)*(tau - C - G);
    
    X_dot(i,1:2) = [X(i,3),X(i,4)];
    X_dot(i,3:4) = theta_dot2';
    
    % Trapezoidal Integration
    if i > 1
        X(i,1:2) = X(i-1,1:2) + 0.5*(X_dot(i-1,1:2) + X_dot(i,1:2))*dt;
        X(i,3:4) = X(i-1,3:4) + 0.5*(X_dot(i-1,3:4) + X_dot(i,3:4))*dt;   
    end
    
    % Plot Energy
    k(i) = 0.5*X_dot(i,1:2)*M*X_dot(i,1:2)';
    u(i) = M2*g*Lc2*sin(X(i,2)) + M2*g*Lc2;
    energy(i) = k(i) + u(i);
    
end



% Graphical Simulation
angles1 = X(:,1);
angles2 = X(:,2);
robot.handles = drawRR([X_0(1),X_0(2)],robot);
for i = 2:length(t)
    setRR([angles1(i),angles2(i)],robot);
    pause(1e-6); % adjustable pause in seconds
end

% Plot Output
figure();
subplot(311);
plot(t,k);
ylabel('Kinematic Energy');
xlabel('t');
title('Kinematic Energy');

subplot(312);
plot(t,u);
ylabel('Potential Energy');
xlabel('t');
title('Potential Energy')

subplot(313);
plot(t,energy);
xlabel('t');
ylabel('Total Energy');
title('Total Energy');


end

