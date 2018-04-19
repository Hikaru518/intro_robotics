function draw3D_3(path_file)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Get path position and color data
data = load(path_file);
s = data.s; % position

% Draw FANUC initially in zero position (do not change)
initPos = [0,0,170]';
[is_solution,initThetas] = NovintFalcon_IK(zeros(3,3),initPos);
prev_angles_1 = initThetas;
prev_angles_2 = initThetas;
prev_angles_3 = initThetas;
%drawNovintFalcon(initThetas);
%hold on;

% Draw in 3D
v = VideoWriter('Dancing3.avi');
open(v);

% for t = 1:size(s,2)
for t = 1:30
    disp(t);
    % Set desired position for the tool from path file (not your choice)
    position_1 = s(1:3,t);
    [is_solution_1,thetas_1] = NovintFalcon_IK(prev_angles_1,position_1);
    position_2 = s(4:6,t);
    [is_solution_2,thetas_2] = NovintFalcon_IK(prev_angles_2,position_2);
    position_3 = s(7:9,t);
    [is_solution_3,thetas_3] = NovintFalcon_IK(prev_angles_3,position_3);
    
    % Move robot using setFanuc() if solution exists
    if(is_solution_1)&&(is_solution_2)&&(is_solution_3)
        drawNovintFalcon_3(thetas_1,thetas_2,thetas_3);
        hold on;
    else
        disp('NO SOLUTION!');
        break;
    end
    ...
    
    % Plot a point at the tool brush tip with the appropriate color
    % (unless the brush selection is zero)
    
    scatter3(position_1(1),position_1(2),position_1(3),...
        'MarkerFaceColor',[1,0,0],...
        'MarkerEdgeColor',[1,0,0]);
    hold on;
    
    frame = getframe(gcf); % leaving gcf out crops the frame in the movie. 
    writeVideo(v,frame);
    ...
    
    % Update previous joint angles
    prev_angles_1 = thetas_1;
    prev_angles_2 = thetas_2;
    prev_angles_3 = thetas_3;
    ...
    
end

close(v);


end