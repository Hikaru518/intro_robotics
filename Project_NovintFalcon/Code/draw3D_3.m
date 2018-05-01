%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
function draw3D_3(path_file)

data = load(path_file);
s = data.s; % position

initPos = [0,0,170]';
[is_solution,initThetas] = NovintFalcon_IK(zeros(3,3),initPos);
prev_angles_1 = initThetas;
prev_angles_2 = initThetas;
prev_angles_3 = initThetas;

% Draw in 3D
% v = VideoWriter('Dancing3.avi');
% open(v);

for t = 1:size(s,2)
    disp(t);

    position_1 = s(1:3,t);
    [is_solution_1,thetas_1] = NovintFalcon_IK(prev_angles_1,position_1);
    position_2 = s(4:6,t);
    [is_solution_2,thetas_2] = NovintFalcon_IK(prev_angles_2,position_2);
    position_3 = s(7:9,t);
    [is_solution_3,thetas_3] = NovintFalcon_IK(prev_angles_3,position_3);
    
    if(is_solution_1)&&(is_solution_2)&&(is_solution_3)
        drawNovintFalcon_3(thetas_1,thetas_2,thetas_3);
        hold on;
    else
        disp('NO SOLUTION!');
        break;
    end
    ...
    
    frame = getframe(gcf); % leaving gcf out crops the frame in the movie. 
%     writeVideo(v,frame);
    ...
    
    % Update previous joint angles
    prev_angles_1 = thetas_1;
    prev_angles_2 = thetas_2;
    prev_angles_3 = thetas_3;
    ...
    
end

% close(v);

end