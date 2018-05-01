%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
function [ T ] = dhtf( alpha,a,d,theta )
%Accepts 4 DH parameters and returns corresponding transformation matrix

T = [cos(theta),        -sin(theta)           , 0          , a;
    sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
    sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha) , cos(alpha)*d;
    0                    , 0                    , 0          , 1];

end

