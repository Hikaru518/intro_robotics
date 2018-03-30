% Name:
%   Peiguang Wang
%   Sichao Zhang

function fanucDraw3D( path_file )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 2 - Inverse Kinematics
%
%    DESCRIPTION - Plot a graphical representation of the FANUC S-500
%    Industrial robot with attached coordinate frames as it moves through a
%    series of poses defined by path_file.
%    
%    ADDITIONAL CODE NEEDED: lots

% Initialize the fanuc struct
fanuc = fanucInit();

% Get path position and color data
data = load(path_file);
s = data.s; % position
c = data.c; % color

% Draw FANUC initially in zero position (do not change)
prev_angles = zeros(1,6);
fanuc.handles = drawFanuc(prev_angles,fanuc);
hold on;

% Draw in 3D
for t = 1:size(s,2)
    
    % Set desired brush color from path file (think about how to handle
    % changes in color)
    fanuc.brush = c(t);
    ...
    
    % Select desired orientation for the tool (your choice)
    T = diag(ones(4,1)); T(2,2) = -1; T(3,3) = -1; % make z downwards
    ...
    
    % Set desired position for the tool from path file (not your choice)
    position = s(:,t);
    T_tmp = fanuc.tool{c(t)};
    T(1:3,4) = position;
    T06 = T*inv(T_tmp);
    ...
    
    % Solve inverse kinematics for nearest solution
    [is_solution,joint_angles] = fanucIK(T06,prev_angles,fanuc);
    ...
    
    % Move robot using setFanuc() if solution exists
    if(is_solution)
        setFanuc(joint_angles,fanuc);
    elseif(is_solution == false)
        disp('NO SOLUTION!')
    end
    ...
    
    % Plot a point at the tool brush tip with the appropriate color
    % (unless the brush selection is zero)
    if(fanuc.brush > 0)
        scatter3(position(1),position(2),position(3)+1000,...
            'MarkerFaceColor',fanuc.brush_colors{c(t)},...
            'MarkerEdgeColor',fanuc.brush_colors{c(t)});
    end
    ...
    
    % Update previous joint angles
    prev_angles = joint_angles;
    ...

end




end

