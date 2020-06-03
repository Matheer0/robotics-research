function [dq, index] = inverse_kinematics_opt (robot, q_current, desired_velocity)
    
    joints = numel(q_current);
    jacobian = robot.geometricJacobian(q_current, robot.BodyNames{end});   % calculate the task jacobian matrix
    jacobian = jacobian(4:6,:);                                    % use 4:6 because I only want to control the position (and leave the orientation free), 3x7 matrix                                                              
    pinv_jacobian = pinv(jacobian);                                % pseudo-inverse of the jacobian, 7x3 matrix
    q_current_param = [q_current(1: joints). JointPosition]';      % 7x1 matrix 
    
    
    %% Find optimal q via optimization
    ub = pi * ones(size(q_current_param));  % joint types are revolute
    lb = -1 * ub;
    A = [];
    b = [];
    Aeq = jacobian;                         % equality constraint: J * q_opt = J * q_current_param
    beq = jacobian * q_current_param;
    q0 = q_current_param;             % start at current configuration
    fun = @(q)(-1 * manipulability(robot,q));
    q_opt = fmincon(fun,q0,A,b,Aeq,beq,lb,ub);
   
    
    %%   
    v_0     = q_opt - q_current_param;          % OPTIMIZED desired joint velocity in the null-space            
    null_space_projection = eye(joints) - pinv_jacobian * jacobian ;                 % calculate the nullspace projection 
    dq      = pinv_jacobian * desired_velocity(4:6) + null_space_projection * v_0 ;  % 7x1 matrix 
    index   = manipulability(robot, q_current_param);
    
end