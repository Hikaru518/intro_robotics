function [ trajectory ] = createRobTrajectory( via, rob )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - Generate a joint position and velocity trajectory to be
%    used by the controller.
%
%    The input via is a 3x4 matrix containing via points for the end
%    effector. Each column contains a point in 3D space. The trajectory
%    should move sequentially through each via point. The best approach is
%    to simply move in a straight line in cartesian space between the
%    points. Use robIK() to find the corresponding trajectory in joint
%    space.
%
%    The output trajectory is a 7xn matrix. The first row will be
%    equally-spaced time stamps starting at time zero and finishing at time
%    t_f given to you below. Rows 2 through 4 contain joint angles,
%    and rows 5 7 should contain joint velocities.

t_f = 30; % final time (do not change) [s]

dt = 0.1;
t = 0:dt:t_f;

trajectory(1,:) = t; %Time

%
%% first generate dense joint positions
d1 = abs(norm(via(:,1) - via(:,2)));
d2 = abs(norm(via(:,2) - via(:,3)));
d3 = abs(norm(via(:,3) - via(:,4)));
totalD = d1 + d2 + d3;
t1 = d1/totalD*t_f;        % Travelling time between first and second point
t2 = t1 + d2/totalD*t_f;   % Travelling time between first and third point 
discTime = [0, t1, t2, t_f];

x_values = interp1(discTime, via(1,:),t);
y_values = interp1(discTime, via(2,:),t);
z_values = interp1(discTime, via(3,:),t);
ik_points = [x_values; y_values; z_values];

% Inverse Kinematics
prev_joint_angles = zeros(3,1);
for tStep = 1:length(t)
    %trajectory(2:4,:) = []; %Joint angles
    [~, trajectory(2:4,tStep)] = robIK(ik_points(:,tStep),prev_joint_angles, rob);
    prev_joint_angles = trajectory(2:4,tStep);
end

% Forward difference
trajectory(5:7,1) = (trajectory(2:4,2) - trajectory(2:4,1))/dt;
% Backwards difference
trajectory(5:7,end) = (trajectory(2:4,end) - trajectory(2:4,end-1))/dt;
% Central difference
for tStep = 2:length(t)-1
    % trajectory(5:7,:) = []; %Joiint velocities
    trajectory(5:7, tStep) = (trajectory(2:4,tStep+1) - trajectory(2:4,tStep-1))/(2*dt);
end


end

