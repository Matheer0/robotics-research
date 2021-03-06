function [dq, index] = inverse_kinematics(robot, q_current, endEffector, desired_velocity)
    % Non-optimization Version of Inverse Kinematics
    joints = numel(q_current);
    jacobian = robot.geometricJacobian(q_current, endEffector);   % calculate the task jacobian matrix
    jacobian = jacobian(4:6,:);                                    % use 4:6 because I only want to control the position (and leave the orientation free), 3x29 matrix                                                              
    pinv_jacobian = pinv(jacobian);                                % pseudo-inverse of the jacobian, 29x3 matrix
   
    q_param = [q_current(1: joints). JointPosition]';   
    homeConfiguration = robot.homeConfiguration;
    home_config_param = [homeConfiguration(1: joints). JointPosition]';
    v_0     = home_config_param - q_param;    % desired joint velocity in the null-space
                
    null_space_projection = eye(joints) - pinv_jacobian * jacobian ;     % calculate the nullspace projection 
    dq      = pinv_jacobian * desired_velocity(4:6) + null_space_projection * v_0 ;   % 29x1 matrix 
    index   = manipulability(robot, q_param, endEffector);
    
end