% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector describing goal position
%        epsilon -> scalar denoting how close end effector must be before
%                   loop terminates
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory.

function traj = Q3(f, qInit, posGoal, epsilon, velocity)
    % calculate the dx to be moved at each time step
    v = posGoal - f.fkine(qInit).t;
    nv = v / norm(v);
    dx = velocity * nv;
    
    % initialize the matrix to store trajectory (configurations)
    numRows = round(norm(v) / velocity);
    traj = zeros(numRows, 9);
    traj(1,:) = qInit;
    
    % calculate trajectory configurations 
    q = qInit;
    rowP = 2;
    while true
        if norm(posGoal - f.fkine(q).t) < epsilon; break; end
        invJ = pinv(f.jacob0(q));
        dq = invJ * [dx;0;0;0];
        q = q + dq';
        traj(rowP,:) = q;
        rowP = rowP + 1;
    end
    
    % remove rows of zeros at the end
    traj(rowP:numRows,:) = [];
end
