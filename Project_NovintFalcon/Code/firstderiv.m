%% MECH 598-Project
%% Sichao Zhang, Peiguang Wang
%%
function [ x_dot ] = firstderiv( x,dt )

num = size(x,2);

for nn = 1 : num
    if nn == 1 || nn == 2
        x_dot(:,nn) = (-25/12*x(:,nn)+4*x(:,nn+1)+-3*x(:,nn+2)+4/3*x(:,nn+3)+-1/4*x(:,nn+4))/dt; 
    elseif nn == num || nn == num-1
        x_dot(:,nn) = -(-25/12*x(:,nn)+4*x(:,nn-1)+-3*x(:,nn-2)+4/3*x(:,nn-3)+-1/4*x(:,nn-4))/dt; 
    else
        x_dot(:,nn) = (1/12*x(:,nn-2)+-2/3*x(:,nn-1)-1/12*x(:,nn+2)+2/3*x(:,nn+1))/dt;
    end
end

end

