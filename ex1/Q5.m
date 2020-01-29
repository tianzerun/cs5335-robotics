% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First seven joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)

function q = Q5(f1, f2, qInit, f1Target, f2Target)
    % tuning parameters
    stepSize = 0.05;
    epsilon = 0.005;
    
    q = qInit;
    dx = Inf; 
    while norm(dx) > epsilon
        % construct a single dx for both f1 and f2
        f1_start = q(1:9);
        f2_start = [q(1:7) q(10:11)];
        f1_x = f1.fkine(f1_start).t;
        f2_x = f2.fkine(f2_start).t;
        f1_dx = f1Target - f1_x;
        f2_dx = f2Target - f2_x;
        dx = [f1_dx;0;0;0;f2_dx;0;0;0]; % 12 x 1
        
        % find the jacobians for both f1 and f2, and fill columns of zeros 
        % for imaginary joints
        f1_j = [f1.jacob0(f1_start) zeros(6,2)]; % 6 x 11
        f2a_j = f2.jacob0(f2_start);
        f2_j = [f2a_j(:,1:7) zeros(6, 2) f2a_j(:,8:9)]; % 6 x 11
        invJ = [f1_j ; f2_j]'; % 11 x 12
        
        dq = stepSize * invJ * dx;
        q  = q + dq';
    end
end
