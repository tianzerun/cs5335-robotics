% Input: x0 -> 3x1 vector denoting the initial vehicle state
%        T -> Scalar denoting the number of time steps in the trajectory
% Output: x_truth -> 3xT matrix containing true (non-noisy) trajectory
%         odo_truth -> 2xT matrix containing true (non-noisy)
%                      odometry readings for T time steps

function [x_truth, odo_truth] = E6(x0, T)
    which_part = 'b';
    
    if which_part == 'a'
        % Part a. Estimate all landmarks with greater error than E3.
        critical_points = [
            3, 5;
            7, 9;
            6, -8;
            -4, -8;
            -6, 10;
            -2, 9;
            9, -9;
        ];
    else
        % Part b. Estimate at least 10 landmarks with less error.
        critical_points = [
            6, -4;
            6, -10;
            2, -10;
            2, -5;
            8, 0;
            4, 3;
            0, 0;
            6, -4;
            10, 0;
            5, -10;
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
            odo_d = 0.05;
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
        if range_steps > 10
            % slow run
            next_t = cur_t + 5 * 1;
            for s = cur_t:next_t
               cur_odo = [0.1; 0];
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
            next_t = cur_t + 5 * 1;
            % slow run
            for s = cur_t:next_t-1
                cur_odo = [0.1; 0];
                odo_truth(:,s) = cur_odo;
                veh = update(veh, cur_odo);
            end
            cur_t = next_t;
        else
            next_t = cur_t + range_steps * 1;
            % slow run
            for s = cur_t:next_t-1
                cur_odo = [0.1; 0];
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