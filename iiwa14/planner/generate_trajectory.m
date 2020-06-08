function [taskWaypoints, timeInterval] = generate_trajectory (hand_pos_init, hand_pos_final, timeStep, toolSpeed) 

    %% Generate Task-space Trajectory
    % Compute task-space trajectory waypoints via interpolation.
    % First, compute tool traveling distance.
    distance = norm(tform2trvec(hand_pos_init) - tform2trvec(hand_pos_final));
    

    % Next, define trajectory times based on traveling distance and desired tool speed.
    initTime = 0;
    finalTime = (distance/toolSpeed) - initTime;
    
    trajTimes = initTime:timeStep:finalTime;   % create intermediate time array
    timeInterval = [trajTimes(1); trajTimes(end)];  % time interval from beginning to the end


    % Generates trajectory between hand_pos_init and hand_pos_final to compute intermediate task-space waypoints.
    [taskWaypoints, taskVelocities] = transformtraj(hand_pos_init,hand_pos_final,timeInterval,trajTimes);
    
    
end