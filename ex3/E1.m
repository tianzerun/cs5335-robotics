% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks) for localization
% Output: x_est -> 1xT cell array containing the vehicle state mean
%                  for T time steps (i.e., x_est{t} is a 3x1 vector)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a 3x3 matrix)

function [x_est, P_est] = E1(odo, zind, z, V, W, x0, P0, map)
    [~, steps] = size(odo);    
    x_est = cell(1, steps);
    P_est = cell(1, steps);    
    x = x0;
    P = P0;
    for k = 1:steps
        cur_odo = odo(:,k);
        
        % Perdiction Phase
        % The estimated state and the estimated covariance at time k+1 
        % based on information from time k.
        F_x = Fx(cur_odo(1), cur_odo(2), x(3));
        F_v = Fv(x(3), cur_odo(2));
        x_pred = dynamics(x(1), x(2), x(3), cur_odo(1), cur_odo(2), 0, 0);
        P_pred = F_x*P*F_x.' + F_v*V*F_v.';

        % Update Phase
        z_i = zind(k);
        if z_i ~= 0
            landmark = map.landmark(z_i);
            h = obs_model(x_pred(1), x_pred(2), x_pred(3), landmark(1), landmark(2), 0, 0);
            innov = z{k} - h;

            H_x = Hx(x_pred(1), x_pred(2), landmark(1), landmark(2), z{k}(1));
            H_w = Hw();
            S = H_x*P_pred*H_x.' + H_w*W*H_w.';
            K = P_pred*H_x.'*inv(S);

            x = x_pred + K*innov;
            P = P_pred - K*H_x*P_pred;
        else
            x = x_pred;
            P = P_pred;
        end
        x_est{k} = x;
        P_est{k} = P; 
    end
        
end


function x_est = dynamics(x_k, y_k, theta_k, odo_d, odo_theta, v_d, v_theta)
    x_est = [...
        x_k+(odo_d+v_d)*cos(theta_k+odo_theta);...
        y_k+(odo_d+v_d)*sin(theta_k+odo_theta);...
        theta_k+odo_theta+v_theta;...
    ];
end

function F_x = Fx(odo_d, odo_theta, theta_v)
    F_x = [...
        1, 0, -odo_d*sin(theta_v+odo_theta);...
        0, 1, odo_d*cos(theta_v+odo_theta);...
        0, 0, 1;...
    ];
end

function F_v = Fv(theta_v, odo_theta)
    F_v = [...
        cos(theta_v+odo_theta), 0;...
        sin(theta_v+odo_theta), 0;...
        0, 1;...
    ];
end

function H_x = Hx(x_v, y_v, x_i, y_i, r)
    H_x = [...
        -(x_i-x_v)/r, -(y_i-y_v)/r, 0;...
        (y_i-y_v)/r^2, -(x_i-x_v)/r^2, -1;...
    ];
end

function H_w = Hw()
    H_w = [1 0; 0 1];
end

function z_est = obs_model(x_v, y_v, theta_v, x_i, y_i, w_r, w_beta)
    z_est = [...
        ((y_i-y_v)^2 + (x_i-x_v)^2)^(1/2) + w_r;...
        angdiff(atan2(y_i-y_v, x_i-x_v), theta_v) + w_beta;...
    ];
end
