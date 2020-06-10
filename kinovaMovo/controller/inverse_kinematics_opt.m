function [dq, index] = inverse_kinematics_opt(robot, q_current, desired_velocity)
    
    joints = numel(q_current);
    jacobian = robot.geometricJacobian(q_current, robot.BodyNames{end});   % calculate the task jacobian matrix
    jacobian = jacobian(4:6,:);                                    % use 4:6 because I only want to control the position (and leave the orientation free), 3x29 matrix                                                              
    pinv_jacobian = pinv(jacobian);                                % pseudo-inverse of the jacobian, 29x3 matrix
    q_current_param = [q_current(1: joints). JointPosition]';      % 29x1 matrix 
    
    null_space_projection = eye(joints) - pinv_jacobian * jacobian ;      % calculate the nullspace projection
    projection_inv = null_space_projection \ eye(joints) ;
    pseudo_solution = pinv_jacobian * desired_velocity(4:6) ;             % pseudo-inverse solution 
        
    
    
    %% Find optimal q via optimization
    max_speed = [3; 3; 3; 3; 3;
                 3; 3; 3; 3; 3;
                 3; 3; 3; 3; 3;
                 3; 3; 3; 3; 3;
                 3; 3; 3; 3; 3;
                 3; 3; 3; 3] ;
    min_speed = -1 * max_speed ;
    
    A_1 = null_space_projection;
    b_1 = max_speed - pseudo_solution + null_space_projection * q_current_param ; 
    A_2 = -1 * null_space_projection;
    b_2 = -1 * (min_speed - pseudo_solution) - null_space_projection * q_current_param ; 
    A = [A_1; A_2];
    b = [b_1; b_2];
    
    ub = pi * ones(size(q_current_param));  % joint types are revolute
    lb = -1 * ub;
    Aeq = jacobian;                         % equality constraint: J * q_opt = J * q_current_param
    beq = jacobian * q_current_param;
    q0 = q_current_param;             % start at current configuration
    fun = @(q)(-1 * manipulability(robot,q));
    q_opt = fmincon(fun,q0,A,b,Aeq,beq,lb,ub);
   
    
    %%   
    v_0     = q_opt - q_current_param;          % OPTIMIZED desired joint velocity in the null-space            
    dq      = pseudo_solution + null_space_projection * v_0 ;  % 29x1 matrix 
    
    index   = manipulability(robot, q_current_param);
    
end