% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    q_goal = path(end,:);
    num_tries = 2;
    
    attempt = 0;
    while attempt < num_tries
        smoothed_path = path(1,:);
        skip_next = false;
        for i = 1:size(path, 1) - 2
            if skip_next
                skip_next = false;
                continue
            end

            one = path(i,:);
            other = path(i+2,:);

            if ~check_edge(robot, one, other, link_radius, sphere_centers, sphere_radii)
                smoothed_path = [smoothed_path; other];
                skip_next = true;
            else
                smoothed_path = [smoothed_path; path(i+1,:)];
            end 
        end
        
        attempt = attempt + 1;
        path = smoothed_path;
    end
    
    if ~isequal(smoothed_path(end,:), q_goal)
    	smoothed_path = [smoothed_path; q_goal];
    end
end