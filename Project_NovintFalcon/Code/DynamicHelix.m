%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
data = load('helix.mat');
s = data.s; % position

initPos = [0,0,170]';
[is_solution,initThetas] = NovintFalcon_IK(zeros(3,3),initPos);
prev_angles = initThetas;

for t = 1:30
    position = s(:,t);
    [is_solution,joint_angles] = NovintFalcon_IK(prev_angles,position);
    
    if(is_solution)
        thetas(:,t) = joint_angles(:,1);
    elseif(is_solution == false)
        disp('NO SOLUTION!');
        break;
    end

    % Update previous joint angles
    prev_angles = joint_angles;

end

plot(thetas(1,:));
hold on;
plot(thetas(2,:));
hold on;
plot(thetas(3,:));
hold on;
title('\theta_1_i in 3D Helix Line');
legend('\theta_1_1','\theta_1_2','\theta_1_3');
