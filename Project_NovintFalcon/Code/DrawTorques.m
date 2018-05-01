%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
dt = 0.5;
g = 9.8;

data = load('helix.mat');
endeff_pos = data.s; % position
num = length(endeff_pos);

initPos = [0,0,170]';
[is_solution,initThetas] = NovintFalcon_IK(zeros(3,3),initPos);
prev_angles = initThetas;

Jacob = cell(1,num);

for nn = 1 : num
    position = endeff_pos(:,nn);
    [is_solution,joint_angles] = NovintFalcon_IK(prev_angles,position);
    
    if(is_solution)
        thetas_1(:,nn) = joint_angles(:,1);
        Jacob{nn} = NovintFalcon_Jacob( joint_angles );
    elseif(is_solution == false)
        disp('NO SOLUTION!');
        break;
    end

    prev_angles = joint_angles;    
end

Jacobinv = cell(1,num);
for nn = 1 : num
    Jacobinv{nn} = (Jacob{nn});
end

thetas_1dot = firstderiv(thetas_1,dt);
thetas_1dotdot = secondderiv(thetas_1,dt);
Jacobinv_dot = firstderiv_cell(Jacobinv,dt);

for nn = 1 : num
    a_p(:,nn) = Jacobinv{nn}*thetas_1dotdot(:,nn) + Jacobinv_dot{nn}*thetas_1dot(:,nn);
end

% taus = -a*g*(1/2*m_a+m_b)*cos(thetas_1)+(Jacob')\m*(a_p+[0;g;0])+c_d*thetas_1dot+I_A*thetas_1dotdot;

for nn = 1 : num
    taus(:,nn) = -0.0298*cos(thetas_1(:,nn))+(Jacob{nn}')\(0.8653*(a_p(nn)+[0;0;0])) ...
        -0.0021*thetas_1dot(:,nn)+0.0004*thetas_1dotdot(:,nn);
end

plot(taus(1,:));
hold on;
plot(taus(2,:));
hold on;
plot(taus(3,:));
hold on;
title('\tau_i in 3D Helix Line');
legend('\tau_1','\tau_2','\tau_3');

