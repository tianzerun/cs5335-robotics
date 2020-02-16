% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    distances = cspace;
    tolerance = abs((q_grid(1) - q_grid(2))) / 2;
    neighbors = [[-1 0 1 -1 1 -1 0 1]; [1 1 1 0 0 -1 -1 -1]];
    [max_row, max_col] = size(cspace);
    
    goal_x = find(abs(q_grid-q_goal(1)) < tolerance);
    goal_y = find(abs(q_grid-q_goal(2)) < tolerance);
    distances(goal_x, goal_y) = 2;
   
    froniter = [goal_x, goal_y];
    [froniter_size, ~] = size(froniter);
    while froniter_size > 0
        cur = froniter(1,:);
        cur_x = cur(1);
        cur_y = cur(2);
        cur_val = distances(cur_x, cur_y);
        froniter(1,:) = [];
        
        for n = neighbors
            n_x = cur_x + n(1);
            n_y = cur_y + n(2);
            if ((n_x >= 1) && (n_x <= max_col)...
                && (n_y >= 1) && (n_y <= max_row)...
                && distances(n_x, n_y) == 0)
                froniter = [froniter; [n_x, n_y]];
                distances(n_x, n_y) = cur_val + 1;
            end
        end
        % update 
        [froniter_size, ~] = size(froniter); 
    end
end