% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
    qs = zeros(num_samples, 4);
    
    for i = 1:4
        q_i_min = q_min(i);
        q_i_max = q_max(i);
        % To generate a number from interval r = a + (b-a).*rand(N,1)
        % where a is the lower bound and b is the upper bound.
        qs(:,i) = q_i_min + (q_i_max - q_i_min) * rand(num_samples, 1);
    end
end