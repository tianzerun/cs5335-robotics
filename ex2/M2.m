% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    samples = zeros(num_samples, 4);
    samples_count = 0;
    adjacency = zeros(num_samples, num_samples);
    
    % Find num_samples collision free samples.
    while samples_count < num_samples
        tmp_samples = M1(q_min, q_max, 20);
        for i = size(tmp_samples, 1)
            q = tmp_samples(i,:);
            if validate_sample(robot, q, q_min, q_max, link_radius, sphere_centers, sphere_radii)
                samples_count = samples_count + 1;
                samples(samples_count,:) = q;
            end
        end
    end
    
    % Check if near points can be connected.
    for i = 1:num_samples
        q = samples(i,:);
        % Find nearest num_neighbors of q.
        [N_q, N_q_d] = find_nearest_neighbors(num_neighbors, q, samples);
        
        for j = 1:size(N_q, 1)
            n_q_row_num = N_q(j);
            n_q = samples(n_q_row_num,:);
            if ~check_edge(robot, q, n_q, link_radius, sphere_centers, sphere_radii)
                adjacency(i, n_q_row_num) = N_q_d(j);
                adjacency(n_q_row_num, i) = N_q_d(j);
            end
        end
    end
end


function [neighbors, distances] = find_nearest_neighbors(k, p, points)
    distances = vecnorm(points - p, 2, 2);
    [B, I] = sort(distances);
    neighbors = I(1+1:k+1,:);
    distances = B(1+1:k+1,:);
end

function is_valid = validate_sample(robot, q, q_min, q_max, link_r, sphere_c, sphere_r)
    is_valid = ~(...
        any(q < q_min)... 
        | any(q > q_max)... 
        | check_collision(robot, q, link_r, sphere_c, sphere_r)...
    );
end
