%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
function [ x_dotdot ] = secondderiv( x,dt )

num = size(x,2);

for nn = 1 : num
    if nn == 1 || nn == 2
        x_dotdot(:,nn) = (15/4*x(:,nn)-77/6*x(:,nn+1)+107/6*x(:,nn+2)-13*x(:,nn+3)+61/12*x(:,nn+4)-5/6*x(:,nn+5))/dt^2; 
    elseif nn == num || nn == num-1
        x_dotdot(:,nn) = -(15/4*x(:,nn)-77/6*x(:,nn-1)+107/6*x(:,nn-2)-13*x(:,nn-3)+61/12*x(:,nn-4)-5/6*x(:,nn-5))/dt^2; 
    else
        x_dotdot(:,nn) = (-1/12*x(:,nn-2)+4/3*x(:,nn-1)-5/2*x(:,nn)+4/3*x(:,nn+1)-1/12*x(:,nn+2))/dt^2;
    end
end

end