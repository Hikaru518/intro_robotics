%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
function [ legs_path ] = ball2leg( ball_path )
% input: the rountine of the ball in the maze
% output: the movements of three legs

ball_vec(1,:) = diff(ball_path(1,:));
ball_vec(2,:) = diff(ball_path(2,:));

count = zeros(1,size(ball_vec,2));
for ii = 1 : size(ball_vec,2)
    if ball_vec(1,ii) == 0
        if ball_vec(2,ii) == 0
            ball_vec(:,ii) = ones(2,1)*1e-9;
            count(ii) = 1;
        end
    end
end

for ii = 1 : size(ball_vec,2)
     theta(:,ii) = generate_theta(ball_vec(:,ii));
     legs_path(:,ii) = generate_path(theta(:,ii));
end

for ii = 1 : size(legs_path,2)
    if count(ii) == 1;
        legs_path(:,ii) = 1/2*(legs_path(:,ii-1)+legs_path(:,ii+1));
    end
end

function [theta] = generate_theta(vec)
    if vec(1) == 0 
        if vec(2) > 0
            theta = pi/2;
        elseif vec(2) < 0
            theta = -pi/2;
        end
    elseif vec(1) < 0
        if vec(2) >= 0
            theta = atan(vec(2)/vec(1))+pi;
        elseif vec(2) < 0
            theta = atan(vec(2)/vec(1))-pi;
        end
    elseif vec(1) > 0
        theta = atan(vec(2)/vec(1));
    end

    if theta < 0
        theta = theta + 2*pi;
    end
end

function [path] = generate_path(theta)
    h = 10; %mm
    r = 37; %mm        
    if theta == 0 
        h_1 = -h;
        h_2 = 0;
        h_3 = 0;
    elseif theta > 0 && theta < pi/3
        ext = sqrt(3)*r*cos(theta+pi/6)/cos(pi/2-theta);
        h_1 = -h;
        h_2 = -h+h*ext/(ext+sqrt(3)*r);
        h_3 = 0;
    elseif theta >= pi/3 && theta < pi/3*2
        ext = sqrt(3)*r*cos(5*pi/6-theta)/cos(theta-pi/6);
        h_1 = -h+h*ext/(ext+sqrt(3)*r);
        h_2 = -h;
        h_3 = 0;

    elseif theta == pi/3*2 
        h_1 = 0;
        h_2 = -h;
        h_3 = 0;
    elseif theta > pi/3*2 && theta < pi/3*3
        ext = sqrt(3)*r*cos(theta-pi/2)/cos(7*pi/6-theta);
        h_1 = 0;
        h_2 = -h;
        h_3 = -h+h*ext/(ext+sqrt(3)*r);
    elseif theta >= pi/3*3 && theta < pi/3*4
        ext = sqrt(3)*r*cos(3*pi/2-theta)/cos(theta-5*pi/6);
        h_1 = 0;
        h_2 = -h+h*ext/(ext+sqrt(3)*r);
        h_3 = -h;

    elseif theta == pi/3*4 
        h_1 = 0;
        h_2 = 0;
        h_3 = -h;
    elseif theta > pi/3*4 && theta < pi/3*5
        ext = sqrt(3)*r*cos(theta-7*pi/6)/cos(11*pi/6-theta);
        h_1 = -h+h*ext/(ext+sqrt(3)*r);
        h_2 = 0;
        h_3 = -h;
    elseif theta >= pi/3*5 && theta < pi/3*6
        ext = sqrt(3)*r*cos(13*pi/6-theta)/cos(theta-3*pi/2);
        h_1 = -h;
        h_2 = 0;
        h_3 = -h+h*ext/(ext+sqrt(3)*r);
    end
    path = [h_1;h_2;h_3];
end

end

