% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    neighbors = [[-1 0 1 -1 1 -1 0 1]; [1 1 1 0 0 -1 -1 -1]];
    goal_pos = find(distances==2);
    [~, N] = size(q_grid);
    goal_x = mod(goal_pos, N);
    goal_y = fix(goal_pos / N) + 1;
    
    tolerance = abs((q_grid(1) - q_grid(2))) / 2;
    explore_x = find(abs(q_grid-q_start(1)) < tolerance);
    explore_y = find(abs(q_grid-q_start(2)) < tolerance);
    path = [explore_x explore_y];
    
    while distances(explore_x, explore_y) ~= 2
        next_x = explore_x;
        next_y = explore_y;
        
        min_dist = distances(next_x, next_y);
        min_manhattan_dist = abs(next_x - goal_x) + abs(next_y - goal_y);
        
        for n = neighbors
            n_x = n(1) + explore_x;
            n_y = n(2) + explore_y;
            n_dist = distances(n_x, n_y);
            % Skip this cell if it is an obstacle or unreachable.
            if n_dist <= 1
                continue
            end
            
            % Use manhattan distance to break tie when points have
            % the same distance to the goal.
            cur_manhattan_dist = abs(n_x - goal_x) + abs(n_y - goal_y);
            
            % Update the next cell to be explore to this cell if its 
            % distance to the goal cell is closer.
            if n_dist < min_dist
                min_dist = n_dist;
                next_x = n_x;
                next_y = n_y;
            elseif n_dist == min_dist && cur_manhattan_dist < min_manhattan_dist
                min_dist = n_dist;
                min_manhattan_dist = cur_manhattan_dist;
                next_x = n_x;
                next_y = n_y;
            end
        end
        explore_x = next_x;
        explore_y = next_y;
        path = [path; [explore_x explore_y]];
    end
end