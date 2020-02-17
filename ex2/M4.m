% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    % Build tree from q_start to q_goal.
    n = 150;
    alpha = 0.2;
    beta = 0.1;
    threshold = 0.3;
    V = q_start;
    E = [];
    
    goal_found = false;
    for i = 1:n
        if goal_found
            break
        end
        
        if rand() < beta
            q_target = q_goal;
        else
            q_target = M1(q_min, q_max, 1);
        end
        q_near = closet_neighbor(q_target, V);
        q_diff = (q_target - q_near);
        q_new = q_near + alpha * q_diff / norm(q_diff);
        
        if ~check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii)...
           && ~check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii)
            % Update V and E with the new node.
            V = [V; q_new];
            E = [E; q_near; q_new];
            
            % When the new node is close enough to the goal node,
            % check if it can reach the goal node without collision.
            if norm(q_goal - q_new) < threshold
                % The program halts when the new node connects to the goal 
                % node without collision.
                if ~check_edge(robot, q_new, q_goal, link_radius, sphere_centers, sphere_radii)
                    goal_found = true;
                    V = [V; q_goal];
                    E = [E; q_new; q_goal];
                end
            end
            
        end
    end
      
    
    % Traverse the tree to recover the actual path
    path = [];
    path_found = goal_found;
    if path_found
        mapping = [];
        froniter = V(1,:);
        
        while true
            cur = froniter(1,:);
            froniter(1,:) = [];
            
            if isequal(cur, q_goal)
                path = backtrack(mapping, q_start, q_goal);
                break;
            else
                children = find_children(cur, E);
                froniter = [froniter; children];

                for i = 1:size(children,1)
                    c = children(i,:);
                    mapping = [mapping; [c cur]];
                end
            end
            
        end
    end
end


function path = backtrack(mapping, q_start, q_goal)
    path = q_goal;
    
    next = find_parent_node(mapping, q_goal);
    while true
        path = [next; path];
        if isequal(next, q_start)
            break
        else
            next = find_parent_node(mapping, next);
        end
    end
end


function children = find_children(node, edges)
    children = [];
    for i = 1:size(edges, 1)
        n = edges(i,:);
        if mod(i, 2) == 0
            continue
        end
        if isequal(node, n)
            children = [children; edges(i+1,:)]; 
        end
    end
end


function parent = find_parent_node(mapping, child)
    for i = 1:size(mapping, 1)
        path = mapping(i,:);
        cur_child = path(1:4);
        cur_parent = path(5:8);
        if isequal(cur_child, child)
           parent = cur_parent; 
           break;
        end
    end
end


function neighbor = closet_neighbor(p, points)
    [~, I] = min(vecnorm(points - p, 2, 2));
    neighbor = points(I,:);
end