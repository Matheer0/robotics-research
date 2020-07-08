clear all ; close all ; 

%% robot setup


robot       = importrobot('iiwa14.urdf');
dim_joint   = numel(robot.homeConfiguration) ; % Get number of joints 
endEffector = robot.BodyNames{end} ; % Get end-effector frame name

% random control constraints
time_step = 0.1 ; % time step, in seconds
hand_speed = 0.1; % m/s
uMax = pi * ones(dim_joint,1);

joint_pos_init = robot.homeConfiguration; 
joint_pos_init_param = [joint_pos_init(1:dim_joint).JointPosition]';   % 7x1 matrix
hand_pos_init  = getTransform(robot, joint_pos_init, endEffector);  % current hand position given the random joint angles


joint_pos_final = robot.randomConfiguration;
joint_pos_final_param = [joint_pos_final(1:dim_joint).JointPosition]';   % 7x1 matrix
hand_pos_final  = getTransform(robot, joint_pos_final, endEffector); 


%% Previous trajectory planning

% robot moves in a straight line from start configuration to goal configuration
[taskWaypoints, timeInterval] = generate_trajectory(hand_pos_init, hand_pos_final, time_step, hand_speed) ;
dim_steps = ceil( ( timeInterval(2) - timeInterval(1) ) / time_step );  %Nt

times    = zeros(1, dim_steps);
states   = zeros(7, dim_steps);
controls = zeros(7, dim_steps);


q = joint_pos_init;
q_param = joint_pos_init_param;

for t = 1 : dim_steps
    
    
    % update time
    times(t) = t * time_step;
    
    % update state
    states(:,t) = q_param;
    
    % get next state
    desired_transformation = taskWaypoints(:,:,t); 
    current_transformation = getTransform(robot, q, endEffector) ;   % current hand position 
    desired_velocity = transformation_diff(current_transformation, desired_transformation) ; % calculate the velocity needed for the desired hand position
    [dq, measure] = inverse_kinematics(robot, q, desired_velocity) ;  
    
    % update state
    controls(:,t) = dq;
    
    q_param  = q_param + time_step * dq; 
    for i = 1:dim_joint
        q(i).JointPosition = q_param(i);
    end

    
end





%% trajectory optimization



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% For all dynamics calculations, the data format must be either 'row' or 'column'.
robot.DataFormat = 'column';

problem.func.dynamics = @(t,x,u)( dynamics(robot,x, hand_speed * ones(dim_joint,1),u) );
problem.func.pathObj = @(t,x,u)( pathObj(x,u) );	% accel-squared cost function


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;

problem.bounds.finalTime.low = 0.1;
problem.bounds.finalTime.upp = 100;

problem.bounds.initialState.low = joint_pos_init_param;
problem.bounds.initialState.upp = joint_pos_init_param;

problem.bounds.finalState.low = joint_pos_final_param;
problem.bounds.finalState.upp = joint_pos_final_param;

problem.bounds.control.low = -uMax;
problem.bounds.control.upp = uMax;





%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% robot moves in a straight line from start configuration to goal configuration



problem.guess.time = times;
problem.guess.state = states;
problem.guess.control = controls;



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%problem.options.method = 'trapezoid'; 
%problem.options.trapezoid.nGrid = 15;



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve the problem                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);







