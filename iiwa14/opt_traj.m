function [optTrajectory, optTrajectory_opt] = opt_traj(robot, joint_pos_init,joint_pos_final, uMax, alpha,  max_joint_speed)
    % Optimized trajectory planning with Non-optimized & Optimized Control 
    
    %% Generate optimized trajectory 
    traj = generate_optTraj(robot, joint_pos_init,joint_pos_final, uMax, alpha);
    
    times    = traj.grid.time;     % 1xnGrid array
    states   = traj.grid.state;   
    controls = traj.grid.control;
    
    dim_steps = numel(times) - 1;
    dim_state = numel(states(:,1));
    dim_joint = numel(joint_pos_init);
    
    %% Non-optimized Control Task-Space Motion
    q_param = joint_pos_init;           % 7x1 matrix 

    optTrajectory.manipulability_index   = zeros(1, dim_steps);
    optTrajectory.time             = zeros(1, dim_steps);
    optTrajectory.state            = zeros(dim_state, dim_steps);
    optTrajectory.hand_pos_desired = zeros(6, dim_steps);
    optTrajectory.hand_pos_current = zeros(6, dim_steps);


    %% Optimized Control Task-Space Motion
    q_opt_param = joint_pos_init;               % 7x1 matrix   

    optTrajectory_opt.manipulability_index_opt        = zeros(1, dim_steps);
    optTrajectory_opt.time             = zeros(1, dim_steps);
    optTrajectory_opt.state            = zeros(dim_state, dim_steps);
    optTrajectory_opt.hand_pos_desired = zeros(6, dim_steps);
    optTrajectory_opt.hand_pos_current = zeros(6, dim_steps);


    %% Robot Control
    
    endEffector = robot.BodyNames{end};
    t_current = 0;
    
    for i = 2 : numel(times)
        
        
        t_next = times(i);
        time_step = t_next - t_current;
        
        state_desired = states(:,i); 
        config_desired = state_desired(1:dim_joint); % desired hand position
        pose_desired = getTransform(robot, config_desired, endEffector); 
        
        current_index = i-1;
        
        %% Non-opt
        pose_current = getTransform(robot, q_param, endEffector);   % current hand position 
        desired_velocity = transformation_diff(pose_current, pose_desired); % calculate the velocity needed for the desired hand position
        [dq, measure] = inverse_kinematics(robot, q_param, desired_velocity);     % use inverse kinematics to control the robot
        state = [q_param; dq];
    
        % save data for visualization
        optTrajectory.manipulability_index(current_index) = measure;
        optTrajectory.time(current_index) = t_next;    % save data for visualization  
        optTrajectory.hand_pos_desired(:,current_index) = transformation_diff(pose_desired);
        optTrajectory.hand_pos_current(:,current_index) = transformation_diff(pose_current);    
        optTrajectory.state(:,current_index) = state;
    
        % get next state
        q_param  = q_param + time_step * dq; 

    
        %% Opt
                       
        pose_current_opt = getTransform(robot, q_opt_param, endEffector);  
        desired_velocity_opt = transformation_diff(pose_current_opt, pose_desired); 
        [dq_opt, measure_opt] = inverse_kinematics_opt(robot, q_opt_param, desired_velocity_opt, max_joint_speed);   
        state_opt = [q_opt_param; dq_opt];        
    
        optTrajectory_opt.manipulability_index_opt(current_index) = measure_opt;
        optTrajectory_opt.time(current_index) = t_next;    
        optTrajectory_opt.hand_pos_desired(:,current_index) = transformation_diff(pose_desired) ;
        optTrajectory_opt.hand_pos_current(:,current_index) = transformation_diff(pose_current_opt) ;    
        optTrajectory_opt.state(:,current_index) = state_opt ; 
    
        % get next state
        q_opt_param  = q_opt_param + time_step * dq_opt; 
    
        %%   
        t_current = t_next;
           
    end
    
    
    
end