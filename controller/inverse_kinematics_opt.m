function [dq, index] = inverse_kinematics_opt (robot, q_current, desired_velocity)
    
    joints = numel(q_current);
    jacobian = robot.geometricJacobian(q_current, robot.BodyNames{end});   % calculate the task jacobian matrix
    jacobian = jacobian(4:6,:);                                    % use 4:6 because I only want to control the position (and leave the orientation free), 3x7 matrix                                                              
    pinv_jacobian = pinv(jacobian);                                % pseudo-inverse of the jacobian, 7x3 matrix
    q_current_param = [q_current(1: joints). JointPosition];
    
    %% Find optimal q via optimization
    lb = -1 * pi * ones(joints , 1);  % joint types are revolute
    ub = -1 * lb;
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    q0 = q_current_param;  % start at current configuration
    fun = @(q)(-1 * manipulability(robot,q));
    q_opt = fmincon(fun, q0);
   
    
    %%   
    v_0     = q_opt - q_current_param;    % OPTIMIZED desired joint velocity in the null-space            
    null_space_projection = eye(joints) - pinv_jacobian * jacobian ;     % calculate the nullspace projection 
    dq      = pinv_jacobian * desired_velocity(4:6) + null_space_projection * v_0' ; 
    dq      = dq' ; 
    index   = manipulability(robot, q_current_param);
    
end