clear all ; close all ;  

robot       = importrobot('iiwa14.urdf');
dim_joint   = numel(robot.homeConfiguration) ; % Get number of joints 
endEffector = robot.BodyNames{end} ; % Get end-effector frame name


%% Specify the some parameters 
time_step = 0.1 ; % time step, in seconds
hand_speed = 0.1; % m/s

joint_pos_init = robot.randomConfiguration ; 
hand_pos_init  = getTransform(robot, joint_pos_init, endEffector);  % current hand position given the random joint angles
hand_pos_final = trvec2tform([0.4, 0, 0.6]) * axang2tform([0 1 0 pi]); % target hand position


%% Generate task-space trajectory
[taskWaypoints, timeInterval]   = generate_trajectory(hand_pos_init, hand_pos_final, time_step, hand_speed) ; 
dim_state = dim_joint * 2 ;
dim_steps = ceil( ( timeInterval(2) - timeInterval(1) ) / time_step ) ;


%% Non-optimized Control Task-Space Motion
q       = joint_pos_init ;
q_param = [q(1:dim_joint). JointPosition]';   % 7x1 matrix  
dq      = zeros(size(q_param)) ;              % 7x1 matrix 
state   = [q_param', dq'] ;                   % 1x14 matrix

manipulability_index        = zeros(1, dim_steps) ;
trajectory.time             = zeros(1, dim_steps) ;
trajectory.state            = zeros(dim_steps, dim_state) ;
trajectory.hand_pos_desired = zeros(dim_steps, 6) ;
trajectory.hand_pos_current = zeros(dim_steps, 6) ;


%% Optimized Control Task-Space Motion
q_opt       = joint_pos_init ;
q_opt_param = [q_opt(1:dim_joint). JointPosition]';   % 7x1 matrix   
dq_opt      = zeros(size(q_opt_param)) ;              % 7x1 matrix 
state_opt   = [q_opt_param', dq_opt'] ;               % 1x14 matrix

manipulability_index_opt        = zeros(1, dim_steps) ;
trajectory_opt.time             = zeros(1, dim_steps) ;
trajectory_opt.state            = zeros(dim_steps, dim_state) ;
trajectory_opt.hand_pos_desired = zeros(dim_steps, 6) ;
trajectory_opt.hand_pos_current = zeros(dim_steps, 6) ;


%%
n = 1 ;
for t = timeInterval(1) : time_step : timeInterval(2)
    
    %% Non-opt
    
    pose_desired = taskWaypoints(:,:,n) ;                  % desired hand position
    pose_current = getTransform(robot, q, endEffector) ;   % current hand position 
    desired_velocity = transformation_diff(pose_current, pose_desired) ; % calculate the velocity needed for the desired hand position
    [dq, measure] = inverse_kinematics(robot, q, desired_velocity ) ;     % use inverse kinematics to control the robot
    state = [q_param', dq'] ;
    
    % save data for visualization
    manipulability_index(n) = measure ;
    trajectory.time(n) = t ;    % save data for visualization  
    trajectory.hand_pos_desired(n,:) = transformation_diff(pose_desired) ;
    trajectory.hand_pos_current(n,:) = transformation_diff(pose_current) ;    
    trajectory.state(n,:) = state ; 
    % get next state
    q_param  = q_param + time_step * dq; 
    for i = 1:dim_joint
        q(i).JointPosition = q_param(i);
    end    
    
    %% Opt
   
    pose_desired_opt = taskWaypoints(:,:,n) ;                  
    pose_current_opt = getTransform(robot, q_opt, endEffector) ;  
    desired_velocity_opt = transformation_diff(pose_current_opt, pose_desired_opt) ; 
    [dq_opt, measure_opt] = inverse_kinematics_opt(robot, q_opt, desired_velocity_opt ) ;   
    state_opt = [q_opt_param', dq_opt'] ;        
    
    manipulability_index_opt(n) = measure_opt ;
    trajectory_opt.time(n) = t ;    
    trajectory_opt.hand_pos_desired(n,:) = transformation_diff(pose_desired_opt) ;
    trajectory_opt.hand_pos_current(n,:) = transformation_diff(pose_current_opt) ;    
    trajectory_opt.state(n,:) = state_opt ; 
    
    % get next state
    q_opt_param  = q_opt_param + time_step * dq_opt ; 
    for i = 1:dim_joint
        q_opt(i).JointPosition = q_opt_param(i);
    end 
    
    %%   
    n = n + 1 ;   
end

%% Compare two methods
% Non-opt  Version
ave = mean(manipulability_index);
minimum = min(manipulability_index);
maximum = max(manipulability_index);
fprintf('Non-Opt: Ave %d, Min %d, Max %d \n', ave, minimum, maximum);

% Opt  Version
ave_opt = mean(manipulability_index_opt);
minimum_opt = min(manipulability_index_opt);
maximum_opt = max(manipulability_index_opt);
fprintf('Opt: Ave %d, Min %d, Max %d \n', ave_opt, minimum_opt, maximum_opt);

visualize_trajectory(robot, trajectory) ;
visualize_trajectory(robot, trajectory_opt) ;


