% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)
function q = Q2(f, qInit, posGoal)
    stepSize = 0.5;
    q = qInit;
    dx = Inf;
    epsilon = 1.0e-06;
    while abs(dx) > epsilon
        x = f.fkine(q).t;
        dx = posGoal - x;
        invJ = pinv(f.jacob0(q));
        dq = stepSize * invJ * [dx;0;0;0];
        q  = q + dq';
    end
end



% function q = Q2(f, qInit, posGoal)
%     stepSize = 0.5;
%     q = qInit;
%     dx = Inf;
%     epsilon = 1.0e-06;
%     while abs(dx) > epsilon
%         x = f.fkine(q);
%         disp(x);
%         disp(x.t);
%         dx = tr2delta(x, SE3(posGoal));
%         invJ = pinv(f.jacob0(q));
% 
%         dq = stepSize * invJ * dx;
%         q  = q + dq';
%     end
% end