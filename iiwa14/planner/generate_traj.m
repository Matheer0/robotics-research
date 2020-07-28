function [taskWaypoints, timeInterval] = generate_traj(hand_pos_init, hand_pos_final, timeStep, toolSpeed)
    % Generate Task-space Trajectory via interpolation (linear trajectory)
    
    distance = norm(tform2trvec(hand_pos_init) - tform2trvec(hand_pos_final)); % compute tool traveling distance.
    initTime = 0;
    finalTime = (distance/toolSpeed) - initTime; % calculate trajectory times
    
    trajTimes = initTime:timeStep:finalTime;   % create intermediate time array
    timeInterval = [trajTimes(1); trajTimes(end)];  % time interval from beginning to the end

    
    % Generates intermediate task-space trajectory waypoints trajectory between hand_pos_init and hand_pos_final
    [taskWaypoints, taskVelocities] = transformtraj(hand_pos_init,hand_pos_final,timeInterval,trajTimes);
    % taskWaypoints: 4x4xM matrix of transformations
    
end