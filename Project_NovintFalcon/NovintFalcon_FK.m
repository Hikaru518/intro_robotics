function [ NovintFalcon_T ] = NovintFalcon_FK( thetas )

% parameters
phi(1) = 0*pi/180;
phi(2) = 120*pi/180;
phi(3) = 240*pi/180;

a = 60; %mm
b = 103; %mm
c = 16.3; %mm
d = 12; %mm
e = 12; % mm
r = 37; %mm

NF_T = cell(3,6);
for ii = 1 : 3
    theta_1 = thetas(ii,1);
    theta_2 = thetas(ii,2);
    theta_3 = thetas(ii,3);
    NF_T{ii,1} = [cos(phi(ii)),-sin(phi(ii)),0,r*cos(phi(ii)); 
                  sin(phi(ii)),cos(phi(ii)),0,r*sin(phi(ii));
                  0,0,1,0;
                  0,0,0,1];
    NF_T{ii,2} = dhtf(pi/2,0,0,theta_1);
    NF_T{ii,3} = dhtf(0,a,0,theta_2-theta_1);
    NF_T{ii,4} = dhtf(pi/2,e,0,-pi/2+theta_3);
    NF_T{ii,5} = dhtf(0,b,0,pi/2-theta_3);
    NF_T{ii,6} = dhtf(-pi/2,d,0,pi-theta_2);
end

NovintFalcon_T = NF_T;

end
