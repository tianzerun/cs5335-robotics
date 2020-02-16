% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q -> 1x4 vector denoting the configuration to check for collision
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: in_collision -> Boolean, true if the robot at configuration q is
%                         in collision with the given spherical obstacles

function in_collision = check_collision(robot, q, link_radius, sphere_centers, sphere_radii, resolution)
    x1 = [0 0 0]';
    T2 = robot.A(1,q) * robot.A(2,q) * robot.A(3,q);
    % x2 is where the last joint is relative to the base frame
    x2 = T2.t;
    T3 = T2 * robot.A(4,q);
    % x3 is where the eff is relative to the base frame
    x3 = T3.t; 
    
    if nargin < 6
        resolution = 11;
    end
    ticks = linspace(0, 1, resolution);
    n = length(ticks);
    x12 = repmat(x1, 1, n) + repmat(x2 - x1, 1, n) .* repmat(ticks, 3, 1);
    x23 = repmat(x2, 1, n) + repmat(x3 - x2, 1, n) .* repmat(ticks, 3, 1);
    % Use discretized points to describe where the links are.
    points = [x12 x23]; % size 3 x (2*resolution) 
    
    % Check if the points are in collision with any spheres.
    in_collision = false;
    for i = 1:size(sphere_centers, 1)
        % Check if the distance between the sphere's center and any point
        % is smaller than the minimum safe distance (link_raidus + sphere_radius).
        % Note: sum up values on the columns using sum(X, 1)
        if any(sum((points - repmat(sphere_centers(i,:)', 1, size(points, 2))).^2, 1) < (link_radius + sphere_radii(i)).^2)
            in_collision = true;
            break;
        end
    end
end