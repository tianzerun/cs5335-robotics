% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)
    num_collisions = 0;
    
    for i = 1:size(q_path, 1) - 1
        config1 = q_path(i,:);
        config2 = q_path(i+1,:);
        
        [c1_l1, c1_l2, ~, ~] = q2poly(robot, config1);
        [c2_l1, c2_l2, ~, ~] = q2poly(robot, config2);
        l1_swept_vol = calculate_swept_vol(c1_l1, c2_l1);
        l2_swept_vol = calculate_swept_vol(c1_l2, c2_l2);
        combined_swept_vol = union(l1_swept_vol, l2_swept_vol);
        
        for o = obstacles
            if intersect(o, combined_swept_vol).NumRegions > 0
                num_collisions = num_collisions + 1;
                C1(robot, config1');
                C1(robot, config2');
                plot(l1_swept_vol, 'FaceColor', 'r');
                plot(l2_swept_vol, 'FaceColor', 'b');
                break;
            end
        end
    end

end

function swept_volume = calculate_swept_vol(from, to)
    points = [from.Vertices; to.Vertices];
    [k, ~] = convhull([from.Vertices; to.Vertices]);
    swept_volume = polyshape(points(k, 1), points(k, 2));
end
