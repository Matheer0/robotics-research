function [trajectory, trajectory_opt] = linear_traj(robot, joint_pos_init, hand_pos_final, time_step, hand_speed, max_joint_speed)
    % Linear trajectory planning with Non-optimized & Optimized Control 
    
    dim_joint   = numel(joint_pos_init); % Get number of joints 
    endEffector = robot.BodyNames{end};
    
    hand_pos_init  = getTransform(robot, joint_pos_init, endEffector);  % hand position, 4x4 matrix 
    

    %% Generate task-space trajectory
    [taskWaypoints, timeInterval] = generate_traj(hand_pos_init, hand_pos_final, time_step, hand_speed);
    [~,~,time_size] = size(taskWaypoints); % length of time, including initial time
    dim_steps = time_size - 1; % number of moves
    dim_state = dim_joint * 2;


    %% Non-optimized Control Task-Space Motion
    q_param = joint_pos_init;           % 7x1 matrix 

    trajectory.manipulability_index        = zeros(1, dim_steps);
    trajectory.time             = zeros(1, dim_steps);
    trajectory.state            = zeros(dim_state, dim_steps);
    trajectory.hand_pos_desired = zeros(6, dim_steps);
    trajectory.hand_pos_current = zeros(6, dim_steps);


    %% Optimized Control Task-Space Motion
    q_opt_param = joint_pos_init;               % 7x1 matrix   


    trajectory_opt.manipulability_index_opt        = zeros(1, dim_steps);
    trajectory_opt.time             = zeros(1, dim_steps);
    trajectory_opt.state            = zeros(dim_state, dim_steps);
    trajectory_opt.hand_pos_desired = zeros(6, dim_steps);
    trajectory_opt.hand_pos_current = zeros(6, dim_steps);


    %%

    for i = 1 : dim_steps
        t_current = (i-1) * time_step;    % current time
        next_index = i + 1;
        pose_desired = taskWaypoints(:,:,next_index); % next desired hand position
        
        %% Non-opt
        pose_current = getTransform(robot, q_param, endEffector);   % current hand position 
        desired_velocity = transformation_diff(pose_current, pose_desired); % calculate the velocity needed for the desired hand position
        [dq, measure] = inverse_kinematics(robot, q_param, desired_velocity);     % use inverse kinematics to control the robot
        state = [q_param; dq];
    
        % save data for visualization
        trajectory.manipulability_index(i) = measure;
        trajectory.time(i) = t_current;
        trajectory.hand_pos_desired(:,i) = transformation_diff(pose_desired);
        trajectory.hand_pos_current(:,i) = transformation_diff(pose_current);    
        trajectory.state(:,i) = state;
    
        % get next state
        q_param  = q_param + time_step * dq; 
    
    
        %% Optimized Control               
        pose_current_opt = getTransform(robot, q_opt_param, endEffector);  
        desired_velocity_opt = transformation_diff(pose_current_opt, pose_desired); 
        [dq_opt, measure_opt] = inverse_kinematics_opt(robot, q_opt_param, desired_velocity_opt, max_joint_speed);   
        state_opt = [q_opt_param; dq_opt];        
        
        % save data for visualization
        trajectory_opt.manipulability_index_opt(i) = measure_opt;
        trajectory_opt.time(i) = t_current;    
        trajectory_opt.hand_pos_desired(:,i) = transformation_diff(pose_desired) ;
        trajectory_opt.hand_pos_current(:,i) = transformation_diff(pose_current_opt) ;    
        trajectory_opt.state(:,i) = state_opt ; 
    
        % get next state
        q_opt_param  = q_opt_param + time_step * dq_opt; 
    
        %%   
           
    end



end