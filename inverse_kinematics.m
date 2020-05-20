function dq = inverse_kinematics (robot, q_input, desired_velocity)
    
    joints = numel(q_input);
    jacobian = robot.geometricJacobian(q_input, robot.BodyNames{end});   % calculate the task jacobian matrix
    jacobian = jacobian(4:6,:);                                    % use 4:6 because I only want to control the position (and leave the orientation free), 3x7 matrix                                                              
    pinv_jacobian = pinv(jacobian);                                % pseudo-inverse of the jacobian, 7x3 matrix
   
    
    %% optimization via measure of distance from a singular configuration
  
    
    
    %%
    
    q_input_param = [q_input(1: joints). JointPosition];   
    configuration = robot.homeConfiguration;
    home_config_param = [configuration(1: joints). JointPosition];
    v_0     = home_config_param - q_input_param;    % desired joint velocity in the null-space
                
    null_space_projection = eye(joints) - pinv_jacobian * jacobian ;     % calculate the nullspace projection 
    dq      = pinv_jacobian * desired_velocity(4:6) + null_space_projection * v_0' ; 
    dq      = dq' ;    
    
end