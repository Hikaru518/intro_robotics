function [bestSolution] = findNearestSolution(prev_angles,candidates)
%find best solution 
%   Detailed explanation goes here

can_size = size(candidates);
N = can_size(1); % # of solution candidates

min_error = norm(prev_angles - candidates(1,:));
bestSolution = candidates(1,:);
for ii = 1:N
    error = norm(prev_angles- candidates(ii,:));
    if (min_error > error)
        min_error = error;
        bestSolution = candidates(ii,:);
    end
end

end

