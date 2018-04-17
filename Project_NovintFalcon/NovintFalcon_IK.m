function [ is_solution, thetas ] = NovintFalcon_IK( prev_thetas,position )
%%

is_solution = true;
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

p_xyz = position;

% loop: leg 1,2,3
for ii = 1 : 3
% ii = 2;
    p_uvw = [cos(phi(ii)),sin(phi(ii)),0; ...
             -sin(phi(ii)),cos(phi(ii)),0; ...
             0,0,1]*p_xyz + [-r;0;0];
    p_u = p_uvw(1);
    p_v = p_uvw(2);
    p_w = p_uvw(3);
    % theta_3
    theta_3(1) = acos(p_v/b);
    theta_3(2) = -acos(p_v/b);
    %%%%%%  check theta_3 within boundary %%%%%%
    % theta_1
    for jj = 1 : 2
        l_0 = p_w^2 + p_u^2 + 2*c*p_u - 2*a*p_u - b^2*sin(theta_3(jj))^2 ...
            - 2*b*e*sin(theta_3(jj)) - 2*b*d*sin(theta_3(jj)) - 2*d*e - 2*a*c ...
            + a^2 + c^2 -d^2 -e^2;
        l_1 = - 4*a*p_w;
        l_2 = p_w^2 + p_u^2 + 2*c*p_u + 2*a*p_u - b^2*sin(theta_3(jj))^2 ...
            - 2*b*e*sin(theta_3(jj)) - 2*b*d*sin(theta_3(jj)) - 2*d*e + 2*a*c ...
            + a^2 + c^2 -d^2 -e^2;
        t_1 = roots([l_2,l_1,l_0]);
    %%%%%%  check t_1 valuable %%%%%%
        theta_1(:,jj) = acos((1-t_1.^2)./(1+t_1.^2));
    %%%%%%  check theta_1 within boundary %%%%%%
    end
    for kk = 1 : 2 % theta_1
        for jj = 1 : 2 % theta_3
           theta_2(kk,jj) = acos((p_u-a*cos(theta_1(kk,jj))+c)/(d+e+b*sin(theta_3(jj))));
           %%%%%%  check theta_2 within boundary %%%%%%
        end
    end
    % four solutions for each leg
    solutions = [];
    for kk = 1 : 2 
        if isreal(theta_1(1,kk)) == 1
            for jj = 1 : 2 
                if theta_2(jj,kk) >= theta_1(jj,kk)
                    solutions = [solutions;theta_1(jj,kk),theta_2(jj,kk),theta_3(:,kk)];
                end
            end
        end
    end
    %%%%%%  check which solution is the nearest %%%%%%
    bestSolution = findNearestSolution(prev_thetas,solutions);
    thetas(ii,:) = bestSolution;
    

    
end
%

% check whether there is a solution
zeroVec = zeros(1,3);
for ii = 1:3
    diffVec = thetas(ii,:) - zeroVec;
    if(norm(diffVec)<0.0001)  % no solution
        is_solution = false;
    end
end

end

