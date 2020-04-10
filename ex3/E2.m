% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the known initial vehicle state
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (2M)x(2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first two
%                    rows of x_est and P_est correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E2(odo, zind, z, W, x0)
    x_est = {};
    P_est = {};
    v_pose = x0;
    M = 0;
    x = [];
    P = [];
    indices = [];
    [~, steps] = size(odo);
    for k = 1:steps
        cur_odo = odo(:,k);
        v_pose = getVehiclePos(v_pose, cur_odo);

        % Predication Phase
        % Assume landmarks are stationary. Nothing happens.

        % Update Phase
        z_i = zind(k);
        if z_i ~= 0
            obs = z{k};
            % Observed an existing landmark
            if ismember(z_i, indices)
                ith = find(indices==z_i);
                landmark = [x(ith*2-1) x(ith*2)];
                H_w = Hw();
                H_x = zeros(2, 2*M);
                H_x(1:2, ith*2-1:ith*2) = Hpi(v_pose(1), v_pose(2), landmark(1), landmark(2), obs(1));
                innov = obs - h(v_pose(1), v_pose(2), v_pose(3), landmark(1), landmark(2));
                S = H_x*P*H_x.' + H_w*W*H_w.';
                K = P*H_x.'*inv(S);
                x = x + K*innov;
                P = P - K*H_x*P;
            else % Observed a new landmark
                indices = [indices ; z_i];
                M = M + 1;
                x = [x; g(v_pose(1), v_pose(2), v_pose(3), obs(1), obs(2))];
                Y_z = eye(2*M);
                Y_z(2*M-1:2*M, 2*M-1:2*M) = Gz(v_pose(3), obs(1), obs(2));
                middle_matrix = zeros(2*M);
                middle_matrix(1:(M-1)*2, 1:(M-1)*2) = P;
                middle_matrix(2*M-1:2*M, 2*M-1:2*M) = W;
                P = Y_z * middle_matrix * Y_z';
            end
        end
        x_est{k} = x;
        P_est{k} = P;
    end
     
end

function H_w = Hw()
    H_w = [1 0; 0 1];
end

function z_est = h(x_v, y_v, theta_v, x_i, y_i)
    z_est = [...
        ((y_i-y_v)^2 + (x_i-x_v)^2)^(1/2);...
        angdiff(atan2(y_i-y_v, x_i-x_v), theta_v);...
    ];
end

function H_pi = Hpi(x_v, y_v, x_i, y_i, range)
    r = range;
    d_x = x_i - x_v;
    d_y = y_i - y_v;
    H_pi = [...
        (d_x)/r     (d_y)/r;...
        -(d_y)/r^2  (d_x)/r^2;...
    ];
end


function G_z = Gz(theta_v, range, bearing)
    angle = theta_v + bearing;
    G_z = [...
        cos(angle) -range*sin(angle);...
        sin(angle) range*cos(angle);...
    ];
end

function pos = g(x_v, y_v, theta_v, range, bearing)
    pos = [...
        x_v + range * cos(theta_v + bearing);...
        y_v + range * sin(theta_v + bearing);...
    ];
end

function new_pose = getVehiclePos(pose, odo)
    new_pose = [...
        pose(1)+odo(1)*cos(pose(3)+odo(2));...
        pose(2)+odo(1)*sin(pose(3)+odo(2));...
        pose(3)+odo(2);...
    ];
end