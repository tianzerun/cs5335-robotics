% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT cell array containing the observed landmark index for
%                T time steps; zind{t} is empty if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 1x1 matrix denoting the sensing noise in (range)
%        range -> Scalar denoting the maximum range of the sensor
%        fov -> 1x2 vector denoting the [min_bearing, max_bearing]
%               (field of view) that can be detected by the sensor
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first
%                    three rows of x_est and P_est correspond to the
%                    vehicle state, the next two correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E9(odo, zind, z, V, W, range, fov, x0, P0)
    % TODO notice here
    beta_init = (fov(1) + fov(2)) / 2;
    W = diag([W, (abs(fov(1))-abs(beta_init))^2]);
    x_est = {};
    P_est = {};
    indices = [];
    x = x0;
    P = P0;
    [~, steps] = size(odo);
    for k = 1:steps
        cur_odo = odo(:,k);
        
        % Prediction Phase
        F_x = Fx(x, cur_odo);
        F_v = Fv(x);
        x_pred = motion_model(x, cur_odo);
        P_pred = F_x*P*F_x.' + F_v*V*F_v.';
        
        % Update Phase
        % process multiple observations
        for i = 1:numrows(zind{k})
            z_i = zind{k}(i,:);
            if z_i ~= 0
                obs = [z{k}(:,i); beta_init];
                
                % Observed an existing landmark
                if ismember(z_i, indices)
                    ith = find(indices==z_i);
                    landmark = [x_pred(3+ith*2-1) x_pred(3+ith*2)];
                    H_w = Hw();
                    r = sqrt((landmark(2) - x_pred(2))^2 + (landmark(1) - x_pred(1))^2);
                    H_x = Hx(x_pred, landmark, ith, r);
                    innov = obs - h(x_pred(1), x_pred(2), x_pred(3), landmark(1), landmark(2));
                    S = H_x*P_pred*H_x.' + H_w*W*H_w.';
                    K = P_pred*H_x.'*inv(S);
                    x_pred = x_pred + K*innov;
                    P_pred = P_pred - K*H_x*P_pred;
                else % Observed a new landmark
                    indices = [indices ; z_i];
                    [n, ~] = size(P_pred);
                    Y_z = Yz(n, x_pred(3), obs(1), obs(2));
                    middle_matrix = [P_pred zeros(n, 2); zeros(2, n) W];
                    x_pred = [x_pred; g(x_pred(1), x_pred(2), x_pred(3), obs(1), obs(2))];
                    P_pred = Y_z * middle_matrix * Y_z.';
                end
            end
        end
        x = x_pred;
        P = P_pred;
        x_est{k} = x;
        P_est{k} = P;
    end
end


function x_est = motion_model(state, odo)
    x_v = state(1);
    y_v = state(2);
    theta_v = state(3);
    odo_d = odo(1);
    odo_theta = odo(2);
    x_est = [...
        x_v+odo_d*cos(theta_v);...
        y_v+odo_d*sin(theta_v);...
        theta_v+odo_theta;...
        state(4:end);...
    ];
end

function F_x = Fx(state, odo)
    [width, ~] = size(state);
    theta_v = state(3);
    odo_d = odo(1);
    F_x = eye(width);
    F_x(1:3,1:3) = [...
        1, 0, -odo_d*sin(theta_v);...
        0, 1, odo_d*cos(theta_v);...
        0, 0, 1;...
    ];
end

function F_v = Fv(state)
    [num_rows, ~] = size(state);
    theta_v = state(3);
    F_v = zeros(num_rows, 2);
    F_v(1:3,1:2) = [...
        cos(theta_v) 0;...
        sin(theta_v) 0;...
        0 1;...
    ];
end

function z_est = h(x_v, y_v, theta_v, x_i, y_i)
    d_x = x_i - x_v;
    d_y = y_i - y_v;
    z_est = [...
        (d_y^2 + d_x^2)^(1/2);...
        angdiff(atan2(d_y, d_x), theta_v);...
    ];
end

function H_x = Hx(state, landmark, landmark_index, r)
    ith = landmark_index;
    [num_cols, ~] = size(state);
    x_i = landmark(1);
    y_i = landmark(2);
    x_v = state(1);
    y_v = state(2);
    H_x = zeros(2, num_cols);
    H_x(1:2,1:3) = [...
        -(x_i-x_v)/r, -(y_i-y_v)/r, 0;...
        (y_i-y_v)/(r^2), -(x_i-x_v)/(r^2), -1;...
    ];
    H_x(1:2,3+ith*2-1:3+ith*2) = Hpi(x_v, y_v, x_i, y_i, r);
end

function H_pi = Hpi(x_v, y_v, x_i, y_i, r)
    d_x = x_i - x_v;
    d_y = y_i - y_v;
    H_pi = [...
        d_x/r       d_y/r;...
        -d_y/(r^2)  d_x/(r^2);...
    ];
end

function H_w = Hw()
    H_w = [1 0; 0 1];
end

function Y_z = Yz(n, theta_v, r, beta)
    Y_z = [...
        eye(n) zeros(n,2);... 
        Gx(theta_v, r, beta) zeros(2,n-3) Gz(theta_v, r, beta);...    
    ];
end

function pos = g(x_v, y_v, theta_v, range, bearing)
    pos = [...
        x_v + range * cos(theta_v + bearing);...
        y_v + range * sin(theta_v + bearing);...
    ];
end

function G_x = Gx(theta_v, range, bearing)
    angle = theta_v + bearing;
    G_x = [...
        1 0 -range*sin(angle);...
        0 1 range*cos(angle);...
    ];
end

function G_z = Gz(theta_v, range, bearing)
    angle = theta_v + bearing;
    G_z = [...
        cos(angle) -range*sin(angle);...
        sin(angle) range*cos(angle);...
    ];
end