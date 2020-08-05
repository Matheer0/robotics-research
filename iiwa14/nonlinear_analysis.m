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
alpha_list = [0,0.1,0.5,0.9,2,5,10,30,40,50,60,80,100,200,500,1000];  % trade-off parameters to test


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



%% optimized trajectory generation

parameters = numel(alpha_list);
result_list = zeros(4, parameters);
result_opt_list = zeros(4, parameters);

for i = 1 : parameters
    
    alpha = alpha_list(i);
    [optTrajectory, optTrajectory_opt] = opt_traj(robot, joint_pos_init,... 
                                        joint_pos_final, uMax,... 
                                        alpha, max_joint_speed);

    % non-optimized control
    nonLinear_result = checkout_result(optTrajectory);
    % optimized control
    nonLinear_result_opt = checkout_result(optTrajectory_opt);
    
    result_list(:,i) = nonLinear_result;
    result_opt_list(:,i) = nonLinear_result_opt;

end





%% visualize relationships regarding alpha
result_opt_list

figure(1)
for i = 1 : parameters
    hold on
    nonLinear_result_opt = result_opt_list(:,i);
    ave = nonLinear_result_opt(1);
    alpha = alpha_list(i);
    plot(alpha, ave, 'r.', 'MarkerSize',20)
    drawnow;
end


figure(2)
for i = 1 : parameters
    hold on
    nonLinear_result_opt = result_opt_list(:,i);
    torque = nonLinear_result_opt(4);
    alpha = alpha_list(i);
    plot(alpha, torque, 'b.', 'MarkerSize',20)
    drawnow;
end











%{
%% show results

for i = 1 : parameters
    
    nonLinear_result = result_list(:,i);
    nonLinear_result_opt = result_opt_list(:,i);
    
    % optimized trajectory generation without optimized inverse kinematics
    fprintf('Opt Traj, Non-Opt Control: Ave %d, Min %d, Max %d \n',...
        nonLinear_result(1), nonLinear_result(2), nonLinear_result(3));
    % optimized trajectory generation with optimized inverse kinematics
    fprintf('Opt Traj, Opt Control: Ave %d, Min %d, Max %d \n',...
        nonLinear_result_opt(1), nonLinear_result_opt(2), nonLinear_result_opt(3));

end

%}