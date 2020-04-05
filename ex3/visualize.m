% Visualizes the estimated trajectory and landmarks, ground truth,
% and optional toolbox EKF estimate (computed only if sol passed in).
% Mode: 'l' for localization, 'm' for mapping, 's' for SLAM.
% Displays estimation errors

function visualize(x_est, P_est, indices, map, veh, mode, sol)
    % Plot ground truth landmarks (black hexagrams) and trajectory (blue line)
    figure;
    scatter(map.map(1,:), map.map(2,:), 'kh');
    hold on;
    plot(veh.x_hist(:,1), veh.x_hist(:,2), 'b');
    sol_exists = (nargin > 6);

    % Visualize the estimated trajectory
    if mode == 'l' || mode == 's'
        if sol_exists
            [err_xt, err_et, err_xe, ang_xt, ang_et, ang_xe] = visualize_path(x_est, P_est, veh, sol);
        else
            [err_xt, err_et, err_xe, ang_xt, ang_et, ang_xe] = visualize_path(x_est, P_est, veh);
        end

        T = length(x_est);
        fprintf('Trajectory errors (average Euclidean and angular distances over %d timesteps):\n', T);
        fprintf('-- between estimate and ground truth: \n   Pos %f \tOri %f\n', err_xt, ang_xt);
        if sol_exists
            fprintf('-- between toolbox EKF estimate and ground truth: \n   Pos %f \tOri %f\n', err_et, ang_et);
            fprintf('-- between estimate and toolbox EKF estimate: \n   Pos %f \tOri %f\n', err_xe, ang_xe);
        end
    end
    
    % Visualize the estimated landmarks
    if mode == 'm' || mode == 's'
        if sol_exists
            [err_xt, err_et, err_xe, num_xt, num_et, num_xe] = visualize_map(x_est, P_est, indices, map, mode == 's', sol);
        else
            [err_xt, err_et, err_xe, num_xt, num_et, num_xe] = visualize_map(x_est, P_est, indices, map, mode == 's');
        end
        fprintf('Landmark errors (average Euclidean distance for landmarks estimated by both):\n');
        fprintf('-- between estimate and ground truth (%d landmarks): \n   %f\n', num_xt, err_xt);
        if sol_exists
            fprintf('-- between toolbox EKF estimate and ground truth (%d landmarks): \n   %f\n', num_et, err_et);
            fprintf('-- between estimate and toolbox EKF estimate (%d landmarks): \n   %f\n', num_xe, err_xe);
        end
    end 
end