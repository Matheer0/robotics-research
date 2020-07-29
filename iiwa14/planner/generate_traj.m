function [taskWaypoints, timeInterval] = generate_traj(hand_pos_init, hand_pos_final, time_step, toolSpeed)
    % Generate Task-space Trajectory via interpolation (linear trajectory)
    
    distance = norm(tform2trvec(hand_pos_init) - tform2trvec(hand_pos_final)); % compute tool traveling distance.
    initTime = 0;
    finalTime = (distance/toolSpeed) + initTime; % calculate trajectory times
    dim_steps = ceil( ( finalTime - initTime ) / time_step ); % number of movements
    data_size = dim_steps + 1; % we need to store each movement, plus initTime
    
    % create intermediate time array of size: dim_steps x 1
    trajTimes = zeros(1, data_size);
    trajTimes(1) = initTime;
    for i = 2:data_size
        trajTimes(i) = time_step * (i-1);
    end 
    
    
    timeInterval = [trajTimes(1); trajTimes(end)];  % time interval from beginning to the end

    % Generate intermediate task-space trajectory waypoints trajectory between hand_pos_init and hand_pos_final
    [taskWaypoints, taskVelocities] = transformtraj(hand_pos_init,hand_pos_final,timeInterval,trajTimes);
    % taskWaypoints: 4x4xM matrix of transformations
    
end