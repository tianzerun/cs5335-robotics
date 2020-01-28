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
    epsilon = 0.05;
    [~, numCols] = size(circle);
    traj = zeros(200, 9);
    traj(1,:) = qInit;
    lastRow = 1;
    
    for i = 1:(numCols - 1)
        partialTraj = Q3(f, traj(lastRow,:), circle(:,i+1), epsilon, velocity);
        [steps, ~] = size(partialTraj);
        prevLastRow = lastRow;
        lastRow = lastRow + steps - 1;
        traj(prevLastRow:lastRow,:) = partialTraj;
    end
    traj(lastRow + 1:200,:) = [];
    disp(traj);
    disp(size(traj));
end



% function traj = Q4(f, qInit, circle, velocity)
%     epsilon = 0.05;
%     [~, numCols] = size(circle);
%     traj = qInit;
%     for i = 1:(numCols - 1)
%         [numRows, ~] = size(traj);
%         partialTraj = Q3(f, traj(numRows,:), circle(:,i+1), epsilon, velocity);
%         partialTraj(1,:) = [];
%         traj = [traj; partialTraj];
%     end
%     disp(traj);
%     disp(size(traj));
% end