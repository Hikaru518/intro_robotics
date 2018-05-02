%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
% dancing 3
NF_1 = [zeros(1,30);zeros(1,30);150+30*sin(linspace(0,pi,30))];
NF_2 = [zeros(1,30);zeros(1,30);150+30*sin(linspace(pi/6,pi+pi/6,30))];
NF_3 = [zeros(1,30);zeros(1,30);150+30*sin(linspace(pi/2,pi+pi/2,30))];

s = [NF_1;NF_2;NF_3];