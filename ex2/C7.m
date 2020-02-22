% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

% To let this function generate a padded configration space where the start 
% and goal configuration are reachable, make sure the resolution of the
% configution is at least 260.
function padded_cspace = C7(cspace)
    [num_row, num_col] = size(cspace);
    valid_row_range = [1 num_row];
    valid_col_range = [1 num_col];
    neighbors = [[-1 0 1 -1 1 -1 0 1]; [1 1 1 0 0 -1 -1 -1]];
    padded_cspace = cspace;
    
    for row = 1:num_row
        for col = 1:num_col
            cell = cspace(row, col);
            
            if cell == 1
                for n = neighbors
                    n_x = row + n(1);
                    n_y = col + n(2);
                    if in_range(n_x, valid_row_range)...
                       && in_range(n_y, valid_col_range)...
                       && cspace(n_x, n_y) == 0
                        padded_cspace(n_x, n_y) = 1;
                    end
                end
            end
            
        end
    end
end

function eval = in_range(value, range)
    left = range(1);
    right = range(2);
    eval = value >= left && value <= right;
end
