% Input: V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
% Output: ekf_l -> Robotics toolbox EKF object
%                  after localizing against a known map
%         ekf_m -> Robotics toolbox EKF object
%                  after estimating a map with known vehicle poses
%         ekf_s -> Robotics toolbox EKF object after performing SLAM

function [ekf_l, ekf_m, ekf_s] = E0(V, W, x0, P0)
    % Localization
    rng(0);
    map = LandmarkMap(20);
    veh = Bicycle('covar', V);
    veh.x0 = x0;
    veh.add_driver(RandomPath(map.dim));
    sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', [-pi/2 pi/2], 'range', 4, 'animate');
    ekf_l = EKF(veh, V, P0, sensor, W, map);
    rng(0);
    ekf_l.run(1000);

    % After creating a map and vehicle, and running the Robotics toolbox
    % EKF, running the following line should produce Figure 1.
    % If it does not, you may have forgotten to reset the random seed to 0.
    visualize({}, {}, [], map, veh, 'n');
    
    % Mapping
    rng(0);
    map = LandmarkMap(20);
    veh = Bicycle();
    veh.x0 = x0;
    veh.add_driver(RandomPath(map.dim));
    sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', [-pi/2 pi/2], 'range', 4, 'animate');
    ekf_m = EKF(veh, [], [], sensor, W, []);
    rng(0);
    ekf_m.run(1000);
    
    % SLAM
    rng(0);
    map = LandmarkMap(20);
    veh = Bicycle('covar', V);
    veh.x0 = x0;
    veh.add_driver(RandomPath(map.dim));
    sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', [-pi/2 pi/2], 'range', 4, 'animate');
    ekf_s = EKF(veh, V, P0, sensor, W, []);
    rng(0);
    ekf_s.run(1000);
end