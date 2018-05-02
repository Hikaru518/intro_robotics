%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
function [ jacobian ] = NovintFalcon_Jacob( thetas )

phi(1) = 0*pi/180;
phi(2) = 120*pi/180;
phi(3) = 240*pi/180;

a = 60; %mm
b = 103; %mm
c = 16.3; %mm
d = 12; %mm
e = 12; % mm
r = 37; %mm

for ii = 1 : 3 % leg
    JF(ii,1) = cos(thetas(ii,2))*sin(thetas(ii,3))*cos(phi(ii)) ...
        - cos(thetas(ii,3))*sin(phi(ii));
    JF(ii,2) = cos(thetas(ii,3))*cos(phi(ii)) ...
        + cos(thetas(ii,2))*sin(thetas(ii,3))*sin(phi(ii));
    JF(ii,3) = sin(thetas(ii,2))*sin(thetas(ii,3));
    JI(:,ii) = a*sin(thetas(ii,2)-thetas(ii,1))*thetas(ii,3);
end

JI = diag(JI);
jacobian = JI\JF;

end