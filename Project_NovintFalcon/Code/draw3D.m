%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
function draw3D(path_file)

data = load(path_file);
s = data.s; % position

initPos = [0,0,170]';
[is_solution,initThetas] = NovintFalcon_IK(zeros(3,3),initPos);
prev_angles = initThetas;

% Draw in 3D
% v = VideoWriter('WaveMovie.avi');
% open(v);

for t = 1:size(s,2)
    disp(t);
    
    position = s(:,t);
    [is_solution,joint_angles] = NovintFalcon_IK(prev_angles,position);
    
    if(is_solution)
        drawNovintFalcon(joint_angles);
        hold on;
    elseif(is_solution == false)
        disp('NO SOLUTION!');
        break;
    end
    ...
    
    frame = getframe(gcf); % leaving gcf out crops the frame in the movie. 
%     writeVideo(v,frame);
    ...
    
    % Update previous joint angles
    prev_angles = joint_angles;
    ...
    
end

% close(v);


end

