function [dq, index] = inverse_kinematics(robot, q_current, desired_velocity)
    % Non-optimization version of Inverse Kinematics
    
    joints = numel(q_current);
    jacobian = robot.geometricJacobian(q_current, robot.BodyNames{end});   % calculate the task jacobian matrix
    jacobian = jacobian(4:6,:);      % use 4:6 because I only want to control the position (and leave the orientation free), 3x7 matrix                                                              
    pinv_jacobian = pinv(jacobian);  % pseudo-inverse of the jacobian, 7x3 matrix
    
    home_config_param = robot.homeConfiguration;
    v_0     = home_config_param - q_current;    % desired joint velocity in the null-space
                
    null_space_projection = eye(joints) - pinv_jacobian * jacobian ;     % nullspace projection, 7x7 matrix 
    dq      = pinv_jacobian * desired_velocity(4:6) + null_space_projection * v_0 ;   % 7x1 matrix 
    index   = manipulability(robot, q_current);
    
end