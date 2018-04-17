function draw3D(path_file)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Get path position and color data
data = load(path_file);
s = data.s; % position

% Draw FANUC initially in zero position (do not change)
initPos = [0,0,170]';
[is_solution,initThetas] = NovintFalcon_IK(zeros(3,3),initPos);
prev_angles = initThetas;
%drawNovintFalcon(initThetas);
%hold on;

% Draw in 3D
v = VideoWriter('WaveMovie.avi');
open(v);

% for t = 1:size(s,2)
for t = 1:50
    disp(t);
    % Set desired position for the tool from path file (not your choice)
    position = s(:,t);
    [is_solution,joint_angles] = NovintFalcon_IK(prev_angles,position);
    
    % Move robot using setFanuc() if solution exists
    if(is_solution)
        drawNovintFalcon(joint_angles);
        hold on;
    elseif(is_solution == false)
        disp('NO SOLUTION!');
        break;
    end
    ...
    
    % Plot a point at the tool brush tip with the appropriate color
    % (unless the brush selection is zero)
    
    scatter3(position(1),position(2),position(3),...
        'MarkerFaceColor',[1,0,0],...
        'MarkerEdgeColor',[1,0,0]);
    hold on;
    
    frame = getframe(gcf); % leaving gcf out crops the frame in the movie. 
    writeVideo(v,frame);
    ...
    
    % Update previous joint angles
    prev_angles = joint_angles;
    ...
    
end

close(v);


end

