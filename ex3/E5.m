% Input: odo_truth -> 2xT matrix containing true (non-noisy)
%                     odometry readings for T time steps
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks)
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        range -> Scalar denoting the maximum range of the sensor
%        fov -> 1x2 vector denoting the [min_bearing, max_bearing]
%               (field of view) that can be detected by the sensor
%        mode -> Character denoting sensing mode
%                'o': One visible landmark (if any) detected per time step
%                'a': All visible landmarks (if any) detected per time step
%                'f': False positives detections are added to observations
% Output: odo -> 2xT matrix containing noisy odometry readings for T time steps
%         zind -> 1xT cell array containing the observed landmark indices
%                 for T time steps; zind{t} is empty if no landmarks observed
%         z -> 1xT cell array containing the (range, bearing) observation
%              for T time steps; z{t} is empty if no observation at time t

function [odo, zind, z] = E5(odo_truth, map, V, W, x0, range, fov, mode)
    [~, T] = size(odo_truth);
    odo = zeros(2, T);
    zind = cell(1, T);
    z = cell(1, T);
    x_t = x0;
    
    for t = 1:T
        % Generate Noisy Odometry
        odo_t = odo_truth(:,t);
        % add noise to the true odometry reading
        odo(:,t) = odo_t + sqrtm(V) * randn(2,1);
        x_t = getVehiclePos(x_t, odo_t);
        
        % Generate Noisy Observation
        % get nosiy range and bearing to all landmarks
        rbs = h(x_t, map, W);
        in_range = find(rbs(:,1) >= 0 & rbs(:,1) <= range... 
                        & rbs(:,2) >= fov(1) & rbs(:,2) <= fov(2));
        rbs = rbs(in_range,:);
        observed_count = length(in_range);
        
        % Add observation record to z and zind based on the mode.
        if mode == 'a'
            % 'a' mode where all landmarks within range and fov are added.
            if observed_count == 0
                continue;
            else
                zind{t} = in_range;
                z{t} = rbs;
            end
        elseif mode == 'o'
            % 'o' mode where only one observed landmark is added at random.
            i = 1;
            if observed_count == 0
                continue;
            elseif observed_count >= 1
                i = randi(observed_count);
            end
            zind{t} = in_range(i);
            z{t} = rbs(i,:)';
        else
            disp('The following mode is not supported:');
            disp(mode);
        end    
    end
end

function z = h(veh, map, W)
    landmarks = map.map';
    dx = landmarks(:,1) - veh(1);
    dy = landmarks(:,2) - veh(2);
    z = [sqrt(dx.^2 + dy.^2) angdiff(atan2(dy, dx), veh(3))];
    z = z + randn(size(z)) * sqrtm(W);
end


function new_pose = getVehiclePos(pose, odo)
    new_pose = [...
        pose(1)+odo(1)*cos(pose(3));...
        pose(2)+odo(1)*sin(pose(3));...
        pose(3)+odo(2);...
    ];
end