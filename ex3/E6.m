% Input: x0 -> 3x1 vector denoting the initial vehicle state
%        T -> Scalar denoting the number of time steps in the trajectory
% Output: x_truth -> 3xT matrix containing true (non-noisy) trajectory
%         odo_truth -> 2xT matrix containing true (non-noisy)
%                      odometry readings for T time steps

function [x_truth, odo_truth] = E6(x0, T)
    which_part = 'b';
    
    if which_part == 'a'
        critical_points = [
            3.575, 5.155;
            6.983, 8.68;
            7.8, 10;
            8, 0;
            3, -6;
            4, -2;
            -1.5, 8;
            -7.5, 8.3;
            -3, 1;
            -4.5, -9;
            3.9, -8.6;
            6, -7.2;
            9, -9.3;
        ];
    else
        critical_points = [
            3.8, 5;
            -4.5,0.5;
            -7,7;
            -2,8;
            3.7,6.2;
            9.2,9.3;
            5.6,9.3;
            -7,9.5;
        ];
    end
    
    % Design an odometry sequence
    odo_truth = zeros(2, T);
    veh = x0;
    cur_t = 1;
    for i = 1:numrows(critical_points)
        landmark = critical_points(i,:);
        [range, bearing] = getRangeAndBearing(veh, landmark);
        range_steps = ceil(abs(range / 0.1));
        bearing_steps = ceil(abs(bearing / 0.0546));
        % rotation
        next_t = cur_t + bearing_steps;
        for s = cur_t:next_t-1
            odo_d = 0.01;
            if bearing < 0
                angle = -0.0546;
            else
                angle = 0.0546;
            end
            cur_odo = [odo_d; angle];
            odo_truth(:,s) = cur_odo;
            veh = update(veh, cur_odo);
            range = range - odo_d;
        end
        
        cur_t = next_t;
        range_steps = ceil(abs(range / 0.1));
%         disp(range_steps);
        if range_steps > 10
            % slow run
            next_t = cur_t + 5 * 5;
            for s = cur_t:next_t
               cur_odo = [0.02; 0];
               odo_truth(:,s) = cur_odo;
               veh = update(veh, cur_odo);
            end
            cur_t = next_t;
            
            next_t = cur_t + range_steps - 5;
            % fast run
            for s = cur_t:next_t-1
                cur_odo = [0.1; 0];
                odo_truth(:,s) = cur_odo;
                veh = update(veh, cur_odo);
            end
            cur_t = next_t;
            next_t = cur_t + 5 * 2;
            % slow run
            for s = cur_t:next_t-1
                cur_odo = [0.05; 0];
                odo_truth(:,s) = cur_odo;
                veh = update(veh, cur_odo);
            end
            cur_t = next_t;
        else
            next_t = cur_t + range_steps * 2;
            % slow run
            for s = cur_t:next_t-1
                cur_odo = [0.05; 0];
                odo_truth(:,s) = cur_odo;
                veh = update(veh, cur_odo);
            end
            cur_t = next_t;
        end
        
    end
    odo_truth = odo_truth(:,1:1000);

    % Compute the x_truth based on the odo_truth
    x_truth = zeros(3, T);
    x = x0;
    for t = 1:T
        x_new = update(x, odo_truth(:,t));
        x_truth(:,t) = x_new;
        x = x_new;
    end
end

function [range, bearing] = getRangeAndBearing(veh, m)
    dx = m(1) - veh(1);
    dy = m(2) - veh(2);
    range = sqrt(dy^2 + dx^2);
    bearing = angdiff(atan2(dy, dx), veh(3));
end

function x_new = update(x, odo)
    x_new = [...
        x(1) + odo(1)*cos(x(3));...
        x(2) + odo(1)*sin(x(3));...
        x(3) + odo(2);...
    ];
end