clear all ; close all ;  


%% make a robot
robot       = importrobot('iiwa14.urdf');
dim_joint   = numel(robot.homeConfiguration) ; % Get number of joints 
endEffector = robot.BodyNames{end} ; % Get end-effector frame name


%% Specify the some parameters 
time_step = 0.1 ; % time step, in seconds
hand_speed = 0.1; % m/s

joint_pos_init = robot.homeConfiguration ; 
hand_pos_init  = getTransform(robot, joint_pos_init, endEffector);  % current hand position given the random joint angles
hand_pos_final = trvec2tform([0.4, 0, 0.6]) * axang2tform([0 1 0 pi]); % target hand position


%% generate task-space trajectory
[taskWaypoints, timeInterval]   = generate_trajectory(hand_pos_init, hand_pos_final, time_step, hand_speed) ; 


%% Control Task-Space Motion
q       = joint_pos_init ;
q_param = [q(1:dim_joint). JointPosition];    % Here!!!!!!!
dq      = zeros(1,dim_joint) ;
state   = [q_param, dq] ;


dim_state = dim_joint * 2 ;
dim_steps = ceil( ( timeInterval(2) - timeInterval(1) ) / time_step ) ; % Here !!!!!!!
% dim_steps = ( timeInterval(2) - timeInterval(1) ) / time_step  ;
trajectory.time = zeros(1, dim_steps) ;
trajectory.state = zeros(dim_steps, dim_state) ;
trajectory.hand_pos = zeros(dim_steps, 6) ;
n = 1 ;

for t = timeInterval(1) : time_step : timeInterval(2)
 
    pose_desired = taskWaypoints(:,:,n) ;                           % desired hand position
    pose_current = getTransform(robot, q, endEffector) ;   % current hand position 
    desired_velocity = transformation_diff(pose_current, pose_desired) ; % calculate the velocity needed for the desired hand position
    
    % use inverse kinematics to control the robot
    dq = inverse_kinematics (robot, q, desired_velocity ) ;   
    
    state = [q_param, dq ] ;        
    % get next state
    q_param  = q_param + dq * time_step ;    
    
    % save data for visualization
    trajectory.time(n) = t ;    
    trajectory.hand_pos_desired(n,:) = transformation_diff(pose_desired) ;
    trajectory.hand_pos_current(n,:) = transformation_diff(pose_current) ;    
    trajectory.state(n,:) = state ; 
    n = n + 1 ;   
end

visualize_trajectory(robot, trajectory) ;

