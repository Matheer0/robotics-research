clear all ; close all ;  

%% robot setup
robot       = importrobot('iiwa14.urdf');
robot.DataFormat = 'column';
dim_joint   = numel(robot.homeConfiguration); % Get number of joints 
endEffector = robot.BodyNames{end}; % Get end-effector frame name


%% specify parameters for robot object

% linear path planning parameters
time_step = 0.1 ; % time step, in seconds
hand_speed = 0.1; % m/s
                                                                                                                        
% optimized path planning parameters
uMax = 50 * ones(dim_joint,1); % random control constraints for path planning
alpha = 0.9;       % trade-off parameter


% control optimization parameters
max_joint_speed = [0; 1.48; 1.48; 1.75; 1.31; 2.27; 2.36]; % random joint speed constraints, used for 
                                                           % fmincon in inverse_kinematics_opt.m

                                                     

%% robot positions setup

%joint_pos_init = robot.randomConfiguration;   % 7x1 matrix
%joint_pos_final = robot.randomConfiguration;

%cartesian_pos_final = [0.4, 0, 0.6];
%hand_pos_final = trvec2tform(cartesian_pos_final) * axang2tform([0 1 0 pi]); % target hand position

joint_pos_init = [1.366810854767469; -0.047714618953960; 0.465977092464026;
                  -1.100463967687917; -0.244195969366768;1.939780737310899;
                  0.285919865265083];
joint_pos_final = [-1.34651803292669; 1.35260787526178;  1.11890660499873;
                  0.435225571215641; -0.670851645972758; -1.81988132007208;
                  3.04573368893129];

              
hand_pos_final  = getTransform(robot, joint_pos_final, endEffector);  % final hand position, 4x4 matrix 
cartesian_pos_final = tform2trvec(hand_pos_final);  % cartesian final postion




%% linear trajectory generation
[trajectory, trajectory_opt] = linear_traj(robot, joint_pos_init,... 
                                           hand_pos_final, time_step,... 
                                           hand_speed, max_joint_speed);

% non-optimized control
[ave, minimum, maximum] = checkout_result(trajectory.manipulability_index);

% optimized control
[ave_opt, minimum_opt, maximum_opt]...
    = checkout_result(trajectory_opt.manipulability_index_opt);



%% optimized trajectory generation
[optTrajectory, optTrajectory_opt] = opt_traj(robot, joint_pos_init,... 
                                        joint_pos_final, uMax,... 
                                        alpha, max_joint_speed);

% non-optimized control
[nonLinear_ave, nonLinear_minimum, nonLinear_maximum]...
     = checkout_result(optTrajectory.manipulability_index);

% optimized control
[nonLinear_ave_opt, nonLinear_minimum_opt,nonLinear_maximum_opt] ... 
    = checkout_result(optTrajectory_opt.manipulability_index_opt);


% alpha = 0.1:
%       Non-Opt: Ave 3.820968e-02, Min 3.796091e-03, Max 9.864981e-02  
%       Opt Version: Opt: Ave 7.924077e-02, Min 3.293399e-02, Max 1.319185e-01


% alpha = 0.5:
%       Non-Opt: Ave 4.197011e-02, Min 1.151876e-03, Max 1.015073e-01
%       Opt Version: Ave 8.751646e-02, Min 4.002992e-02, Max 1.464803e-01



% alpha = 0.8:
%       Non-Opt: Ave 3.367299e-02, Min 2.006861e-03, Max 6.607888e-02
%       Opt Version: Ave 8.209630e-02, Min 2.209068e-02, Max 1.474709e-01 


% alpha = 0.9:
%       Non-Opt: Ave 3.382521e-02, Min 2.215611e-03, Max 6.713863e-02
%       Opt Version: Ave 8.509146e-02, Min 2.389138e-02, Max 1.509780e-01 



%% show results

% linear trajectory generation without optimized inverse kinematics
fprintf('Linear Traj, Non-Opt Control: Ave %d, Min %d, Max %d \n',...
    ave, minimum, maximum);
% linear trajectory generation with optimized inverse kinematics
fprintf('Linear Traj, Opt Control: Ave %d, Min %d, Max %d \n',...
    ave_opt, minimum_opt, maximum_opt);



% Linear (non-optimized) trajectory:
%       Linear Traj, Non-Opt Control: Ave 8.223715e-02, Min 2.263554e-02, Max 1.058529e-01 
%       Linear Traj, Opt Control: Ave 1.160642e-01, Min 6.026822e-02, Max 1.595046e-01 



    
% optimized trajectory generation without optimized inverse kinematics
fprintf('Opt Traj, Non-Opt Control: Ave %d, Min %d, Max %d \n',...
    nonLinear_ave, nonLinear_minimum, nonLinear_maximum);
% optimized trajectory generation with optimized inverse kinematics
fprintf('Opt Traj, Opt Control: Ave %d, Min %d, Max %d \n',...
    nonLinear_ave_opt, nonLinear_minimum_opt, nonLinear_maximum_opt);




%% visualize trajectories

% linear path planning
visualize_trajectory(robot, trajectory, cartesian_pos_final);
visualize_trajectory(robot, trajectory_opt, cartesian_pos_final);

% optimized path planning
visualize_trajectory(robot, optTrajectory, cartesian_pos_final);
visualize_trajectory(robot, optTrajectory_opt, cartesian_pos_final);


