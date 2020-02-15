% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
    [~, num_grid_points] = size(q_grid);
    cspace = zeros(num_grid_points);
    
    
    for row = 1:num_grid_points
        for col = 1:num_grid_points
            [poly1, poly2, ~, ~] = q2poly(robot, [q_grid(row) ; q_grid(col)]);
            % check if the two links intersect with any obstracles
            for obs = obstacles
                poly1_obs = intersect(poly1, obs);
                poly2_obs = intersect(poly2, obs);
                % when any link intersect, there is a collision
                if poly1_obs.NumRegions > 0 || poly2_obs.NumRegions > 0
                    cspace(row, col) = 1;
                    break
                end
            end
            
        end
    end
end