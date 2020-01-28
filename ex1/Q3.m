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
    v = posGoal - f.fkine(qInit).t;
    nv = v / norm(v);
    dx = velocity * nv;
    q = qInit;
    numOfRows = round(norm(v) / velocity);
    traj = zeros(numOfRows, 9);
    traj(1,:) = qInit;
    
    rowP = 2;
    while true
        if norm(posGoal - f.fkine(q).t) < epsilon; break; end
        invJ = pinv(f.jacob0(q));

        dq = invJ * [dx;0;0;0];
        q = q + dq';
        traj(rowP,:) = q;
        rowP = rowP + 1;
    end
    traj(rowP:numOfRows,:) = [];
end