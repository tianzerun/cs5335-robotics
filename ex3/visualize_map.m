% Visualizes the estimated landmarks, ground truth,
% and optional toolbox EKF estimate (computed only if sol passed in).
% Returns errors between each pair

function [err_xt, err_et, err_xe, num_xt, num_et, num_xe] = visualize_map(x_est, P_est, indices, map, is_slam, sol)
    % If doing SLAM, need to offset indices by 3
    % (first 3 elements of state vector are for vehicle position estimate)
    if is_slam
        offset = 3;
    else
        offset = 0;
    end

    % Collect the estimated landmark positions
    T = length(x_est);
    M = (length(x_est{T}) - offset) / 2;
    m_est = reshape(x_est{T}((1+offset):end), [2 M]);
    % Plot the estimated landmarks as red dots
    scatter(m_est(1,:), m_est(2,:), 'r.');
    % Plot 95%-confidence ellipses around landmarks
    for i = 1:M
        plot_ellipse(P_est{T}((2*i-1+offset):(2*i+offset),(2*i-1+offset):(2*i+offset)) * 5.991, m_est(:,i), 'r');
    end
    
    % If toolbox EKF object is passed in, visualize the estimate too
    ekf_exists = (nargin > 5);
    if ekf_exists
        % Collect and plot the estimated landmarks in green
        e_est = reshape(sol.history(T).x_est((1+offset):end), [2 M]);
        scatter(e_est(1,:), e_est(2,:), 'g.');
        % Plot 95%-confidence ellipses around landmarks
        for i = 1:M
            plot_ellipse(sol.history(T).P((2*i-1+offset):(2*i+offset),(2*i-1+offset):(2*i+offset)) * 5.991, e_est(:,i), 'g');
        end
    end
    
    % Compute errors between estimate, ground truth, and toolbox EKF estimate
    % For each estimate, construct 2*M_truth matrix of landmark positions,
    % where M_truth is the true number of landmarks (in argument map);
    % undetected landmarks will have NaN values in their estimates
    map_t = map.map;
    map_x = NaN(size(map_t));
    for i = 1:length(indices)
        map_x(:,indices(i)) = x_est{T}((2*i-1+offset):(2*i+offset));
    end
    if ekf_exists
        map_e = NaN(size(map_t));
        for i = 1:size(map_t, 2)
            if ~isnan(sol.landmarks(1,i))
                map_e(:,i) = sol.history(T).x_est((sol.landmarks(1,i)+offset):(sol.landmarks(1,i)+1+offset));
            end
        end
    end
    
    % Only compute errors between landmarks that are estimated by both
    % Return number of landmarks that are estimated by both,
    % as well as average error in the overlapping landmarks
    mask_xt = ~isnan(map_x) & ~isnan(map_t);
    num_xt = sum(sum(mask_xt)) / 2;  
    err_xt = average_error(reshape(map_x(mask_xt), [2 num_xt]), reshape(map_t(mask_xt), [2 num_xt]));
    if ekf_exists
        mask_et = ~isnan(map_e) & ~isnan(map_t);
        num_et = sum(sum(mask_et)) / 2;
        err_et = average_error(reshape(map_e(mask_et), [2 num_et]), reshape(map_t(mask_et), [2 num_et]));
        mask_xe = ~isnan(map_x) & ~isnan(map_e);
        num_xe = sum(sum(mask_xe)) / 2;
        err_xe = average_error(reshape(map_x(mask_xe), [2 num_xe]), reshape(map_e(mask_xe), [2 num_xe]));
    else
        num_et = NaN;
        err_et = NaN;
        num_xe = NaN;
        err_xe = NaN;
    end
end