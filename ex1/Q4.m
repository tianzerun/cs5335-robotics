% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        circle -> 3xn matrix of Cartesian positions that describe the
%                  circle
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory. The end effector should trace a circle in the
%                 workspace, as specified by the input argument circle.
%                 (orientation is to be ignored)

function traj = Q4(f, qInit, circle, velocity)
    % tuning parameters
    epsilon = 0.01;
    
    % initialize a matrix to store trajectory information
    numRows = 300;
    traj = zeros(numRows, 9);
    traj(1,:) = qInit;
    
    % calculate the trajectory for each critial point on the circle
    lastRow = 1;
    for target = circle
        partialTraj = Q3(f, traj(lastRow,:), target, epsilon, velocity);
        [steps, ~] = size(partialTraj);
        prevLastRow = lastRow;
        lastRow = lastRow + steps - 1;
        traj(prevLastRow:lastRow,:) = partialTraj;
    end
    
    % remove rows of zeros at the end
    traj(lastRow + 1:numRows,:) = [];
end
