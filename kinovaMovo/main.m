clear all ; close all ;  

%% I. Robot Setup
time_step = 0.1 ; % time step, in seconds
hand_speed = 0.2; % m/s

robot       = loadrobot("kinovaMovo");
dim_joint   = numel(robot.homeConfiguration);  % Kinova Movo has 29 non-fixed joints 
joint_pos_init = robot.randomConfiguration ; 




%% II. Final Hand Positions
% left_hand_pos_final = trvec2tform([0.4, 0, 0.6]) * axang2tform([0 1 0 pi]);         % target left hand position

cartesian_position = [0.6, 0.5, 0.5];
left_hand_pos_final = trvec2tform(cartesian_position);       % target left hand position
right_hand_pos_final = left_hand_pos_final;                  % target right hand position


%% III. First work on left hand 

left_endEffector = robot.BodyNames{12}; % left hand end-effector frame name: 'left_wrist_3_link'
left_hand_pos_init  = getTransform(robot, joint_pos_init, left_endEffector);        % current left hand position 

% Generate task-space trajectory
dim_state = dim_joint * 2 ;
[left_taskWaypoints, left_timeInterval]   = generate_trajectory(left_hand_pos_init, left_hand_pos_final, time_step, hand_speed) ; 
left_dim_steps = ceil( ( left_timeInterval(2) - left_timeInterval(1) ) / time_step ) ;


% Non-optimized Control Task-Space Motion
left_q       = joint_pos_init ;
left_q_param = transpose([left_q(1:dim_joint). JointPosition]);    % 29x1 matrix  
left_dq      = zeros(size(left_q_param)) ;              		   % 29x1 matrix 
left_state   = [left_q_param.', left_dq.'] ;                   	   % 1x58 matrix

left_manipulability_index        = zeros(1, left_dim_steps) ;
left_trajectory.time             = zeros(1, left_dim_steps) ;
left_trajectory.state            = zeros(left_dim_steps, dim_state) ;
left_trajectory.hand_pos_desired = zeros(left_dim_steps, 6) ;
left_trajectory.hand_pos_current = zeros(left_dim_steps, 6) ;


% Optimized Control Task-Space Motion
left_q_opt       = joint_pos_init ;
left_q_opt_param = transpose([left_q_opt(1:dim_joint). JointPosition]);     % 29x1 matrix   
left_dq_opt      = zeros(size(left_q_opt_param)) ;                          % 29x1 matrix 
left_state_opt   = [left_q_opt_param.', left_dq_opt.'] ;                    % 1x58 matrix

left_manipulability_index_opt        = zeros(1, left_dim_steps) ;
left_trajectory_opt.time             = zeros(1, left_dim_steps) ;
left_trajectory_opt.state            = zeros(left_dim_steps, dim_state) ;
left_trajectory_opt.hand_pos_desired = zeros(left_dim_steps, 6) ;
left_trajectory_opt.hand_pos_current = zeros(left_dim_steps, 6) ;


left_timeInterval(1)
left_timeInterval(2)

% Iterate to update left hand positions
n = 1 ;
for t = left_timeInterval(1) : time_step : left_timeInterval(2)
    
    if mod(n,5) == 0
        n
    end
    
    % Non-opt
    left_pose_desired = left_taskWaypoints(:,:,n) ;                                                            % desired left hand position
    left_pose_current = getTransform(robot, left_q, left_endEffector) ;                                        % current left hand position 
    left_desired_velocity = transformation_diff(left_pose_current, left_pose_desired) ;                        % calculate the velocity needed for the desired hand position
    [left_dq, left_measure] = inverse_kinematics(robot, left_q, left_endEffector, left_desired_velocity) ;     % use inverse kinematics to control the robot
    left_state = [left_q_param.', left_dq.'] ;
    
    % save data for visualization
    left_manipulability_index(n) = left_measure ;
    left_trajectory.time(n) = t ;      
    left_trajectory.hand_pos_desired(n,:) = transformation_diff(left_pose_desired) ;
    left_trajectory.hand_pos_current(n,:) = transformation_diff(left_pose_current) ;    
    left_trajectory.state(n,:) = left_state ; 

    % get next state
    left_q_param  = left_q_param + time_step * left_dq; 
    for i = 1:dim_joint
        left_q(i).JointPosition = left_q_param(i);
    end    
    
  
    % Opt
    left_pose_desired_opt = left_taskWaypoints(:,:,n) ;                  
    left_pose_current_opt = getTransform(robot, left_q_opt, left_endEffector) ;  
    left_desired_velocity_opt = transformation_diff(left_pose_current_opt, left_pose_desired_opt) ; 
    [left_dq_opt, left_measure_opt] = inverse_kinematics_opt(robot, left_q_opt, left_endEffector, left_desired_velocity_opt) ;   
    left_state_opt = [left_q_opt_param', left_dq_opt'] ;        
    
    left_manipulability_index_opt(n) = left_measure_opt ;
    left_trajectory_opt.time(n) = t ;    
    left_trajectory_opt.hand_pos_desired(n,:) = transformation_diff(left_pose_desired_opt) ;
    left_trajectory_opt.hand_pos_current(n,:) = transformation_diff(left_pose_current_opt) ;    
    left_trajectory_opt.state(n,:) = left_state_opt ; 
    
    % get next state
    left_q_opt_param  = left_q_opt_param + time_step * left_dq_opt ; 
    for i = 1:dim_joint
        left_q_opt(i).JointPosition = left_q_opt_param(i);
    end
    
    
    
    % Move to next step  
    n = n + 1 ;   
end




%% IV. Work on Non-optimized right hand 

% Non-optimized Control Task-Space Motion
right_q       = left_q ;
right_endEffector = robot.BodyNames{39};           % Get right hand end-effector frame name: 'right_wrist_3_link'
right_hand_pos_init  = getTransform(robot, right_q, right_endEffector);      % current right hand position 

[right_taskWaypoints, right_timeInterval]   = generate_trajectory(right_hand_pos_init, right_hand_pos_final, time_step, hand_speed) ; 
right_dim_steps = ceil( ( right_timeInterval(2) - right_timeInterval(1) ) / time_step ) ;


right_q_param = transpose([right_q(1:dim_joint). JointPosition]);   % 29x1 matrix  
right_dq      = zeros(size(right_q_param)) ;                        % 29x1 matrix 
right_state   = [right_q_param.', right_dq.'] ;                     % 1x58 matrix

right_manipulability_index        = zeros(1, right_dim_steps) ;
right_trajectory.time             = zeros(1, right_dim_steps) ;
right_trajectory.state            = zeros(right_dim_steps, dim_state) ;
right_trajectory.hand_pos_desired = zeros(right_dim_steps, 6) ;
right_trajectory.hand_pos_current = zeros(right_dim_steps, 6) ;


n = 1 ;
for t = right_timeInterval(1) : time_step : right_timeInterval(2)
    
    right_pose_desired = right_taskWaypoints(:,:,n) ;                  % desired right hand position
    right_pose_current = getTransform(robot, right_q, right_endEffector) ;   % current right hand position 
    right_desired_velocity = transformation_diff(right_pose_current, right_pose_desired) ; % calculate velocity needed 
    [right_dq, right_measure] = inverse_kinematics(robot, right_q, right_endEffector, right_desired_velocity) ;     % use inverse kinematics to control the robot
    right_state = [right_q_param.', right_dq.'] ;
    
    % save data for visualization
    right_manipulability_index(n) = right_measure ;
    right_trajectory.time(n) = t ;      
    right_trajectory.hand_pos_desired(n,:) = transformation_diff(right_pose_desired) ;
    right_trajectory.hand_pos_current(n,:) = transformation_diff(right_pose_current) ;    
    right_trajectory.state(n,:) = right_state ; 
    % get next state
    right_q_param  = right_q_param + time_step * right_dq; 
    for i = 1:dim_joint
        right_q(i).JointPosition = right_q_param(i);
    end    
   
    n = n + 1 ;   
end


%% IV. Work on Optimized right hand 

% Optimized Control Task-Space Motion
right_q_opt       = left_q_opt ;
right_hand_pos_opt_init  = getTransform(robot, right_q_opt, right_endEffector);      % current right hand position 

[right_taskWaypoints_opt, right_timeInterval_opt]   = generate_trajectory(right_hand_pos_opt_init, right_hand_pos_final, time_step, hand_speed) ; 
right_dim_steps_opt = ceil( ( right_timeInterval_opt(2) - right_timeInterval_opt(1) ) / time_step ) ;


right_q_opt_param = transpose([right_q_opt(1:dim_joint). JointPosition]);   % 29x1 matrix  
right_dq_opt      = zeros(size(right_q_opt_param)) ;                        % 29x1 matrix 
right_state_opt   = [right_q_opt_param.', right_dq_opt.'] ;                 % 1x58 matrix


right_manipulability_index_opt        = zeros(1, right_dim_steps_opt) ;
right_trajectory_opt.time             = zeros(1, right_dim_steps_opt) ;
right_trajectory_opt.state            = zeros(right_dim_steps_opt, dim_state) ;
right_trajectory_opt.hand_pos_desired = zeros(right_dim_steps_opt, 6) ;
right_trajectory_opt.hand_pos_current = zeros(right_dim_steps_opt, 6) ;


right_timeInterval_opt(1)
right_timeInterval_opt(2)


n = 1 ;
for t = right_timeInterval_opt(1) : time_step : right_timeInterval_opt(2)
    
    if mod(n,5) == 0
        n
    end
    
    
    right_pose_desired_opt = right_taskWaypoints_opt(:,:,n) ;                  
    right_pose_current_opt = getTransform(robot, right_q_opt, right_endEffector) ;  
    right_desired_velocity_opt = transformation_diff(right_pose_current_opt, right_pose_desired_opt) ; 
    [right_dq_opt, right_measure_opt] = inverse_kinematics_opt(robot, right_q_opt, right_endEffector, right_desired_velocity_opt) ;   
    right_state_opt = [right_q_opt_param', right_dq_opt'] ;        

    
    right_manipulability_index_opt(n) = right_measure_opt ;
    right_trajectory_opt.time(n) = t ;    
    right_trajectory_opt.hand_pos_desired(n,:) = transformation_diff(right_pose_desired_opt) ;
    right_trajectory_opt.hand_pos_current(n,:) = transformation_diff(right_pose_current_opt) ;    
    right_trajectory_opt.state(n,:) = right_state_opt ; 
     

    % get next state
    right_q_opt_param  = right_q_opt_param + time_step * right_dq_opt ; 
    for i = 1:dim_joint
        right_q_opt(i).JointPosition = right_q_opt_param(i);
    end   
 
    n = n + 1 ;   
end




%% VI. Show Results
% Non-opt Left Hand
left_ave = mean(left_manipulability_index);
left_minimum = min(left_manipulability_index);
left_maximum = max(left_manipulability_index);
fprintf('Left Hand Non-Opt: Ave %d, Min %d, Max %d \n', left_ave, left_minimum, left_maximum);

% Opt Left Hand
left_ave_opt = mean(left_manipulability_index_opt);
left_minimum_opt = min(left_manipulability_index_opt);
left_maximum_opt = max(left_manipulability_index_opt);
fprintf('Left Hand Opt: Ave %d, Min %d, Max %d \n', left_ave_opt, left_minimum_opt, left_maximum_opt);



% Non-opt Right Hand
right_ave = mean(right_manipulability_index);
right_minimum = min(right_manipulability_index);
right_maximum = max(right_manipulability_index);
fprintf('Right Hand Non-Opt: Ave %d, Min %d, Max %d \n', right_ave, right_minimum, right_maximum);

% Opt Right Hand
right_ave_opt = mean(right_manipulability_index_opt);
right_minimum_opt = min(right_manipulability_index_opt);
right_maximum_opt = max(right_manipulability_index_opt);
fprintf('Right Hand Opt: Ave %d, Min %d, Max %d \n', right_ave_opt, right_minimum_opt, right_maximum_opt);



%% VII. Show Visualization
visualize_trajectory(robot, left_trajectory, cartesian_position) ;
visualize_trajectory(robot, right_trajectory, cartesian_position) ;

visualize_trajectory(robot, left_trajectory_opt, cartesian_position) ;
visualize_trajectory(robot, right_trajectory_opt, cartesian_position) ;


