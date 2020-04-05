% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% Input: questionNum -> Integer between 0 and 6 that denotes question
%                       number to run.
%        efk_l, ekf_m, ekf_s -> Optional pre-computed Robotics toolbox EKF
%                               objects computed in question E0
% Output: efk_l, ekf_m, ekf_s -> If EKF objects were computed, you can save
%                               it and pass it in on later calls to ex3_ekf
%                               to avoid re-computing it

function [ekf_l, ekf_m, ekf_s] = ex3_ekf(questionNum, ekf_l, ekf_m, ekf_s)

    close all;
    
    if nargin < 1
        error('Error: Please enter a question number as a parameter');
    end
    
    % Process noise
    V = diag([0.02 0.5*pi/180].^2);
    % Sensing noise
    W = diag([0.1 1*pi/180].^2);
    % Initial mean
    x0 = [0 0 0]';
    % Initial covariance
    P0 = diag([.01 .01, 0.005].^2);
    
    % ========== Question E0 ==========
    if questionNum == 0
        [ekf_l, ekf_m, ekf_s] = E0(V, W, x0, P0);
    end
    
    % ========== Question E1 ==========
    if questionNum == 1
        load('e1.mat');
        [x_est, P_est] = E1(odo_l, zind_l, z_l, V, W, x0, P0, map_l);
        if nargin > 1
            visualize(x_est, P_est, [], map_l, veh_l, 'l', ekf_l);
        else
            visualize(x_est, P_est, [], map_l, veh_l, 'l');
        end
    end

    % ========== Question E2 ==========
    if questionNum == 2
        load('e2.mat');
        [x_est, P_est, indices] = E2(odo_m, zind_m, z_m, W, x0);
        if nargin > 2
            visualize(x_est, P_est, indices, map_m, veh_m, 'm', ekf_m);
        else
            visualize(x_est, P_est, indices, map_m, veh_m, 'm');
        end
    end
    
    % ========== Question E3 ==========
    if questionNum == 3
        load('e3.mat');
        [x_est, P_est, indices] = E3(odo_s, zind_s, z_s, V, W, x0, P0);
        if nargin > 3
            visualize(x_est, P_est, indices, map_s, veh_s, 's', ekf_s);
        else
            visualize(x_est, P_est, indices, map_s, veh_s, 's');
        end
    end
    
    % ========== Question E4 ==========
    if questionNum == 4
        load('e3_r8.mat');
        ekf_s_R8 = E4(V, W, x0, P0);
        [x_est, P_est, indices] = E3(odo_s, zind_s, z_s, V, W, x0, P0);
        visualize(x_est, P_est, indices, map_s, veh_s, 's', ekf_s_R8);
    end
end