function dx = dynamics(x,u,robot_obj)
    
    % returns [dimStates, dimTimes] array,
    % where size of x is dimStates * dimTimes
    
    [dimStates,dimTimes] = size(x);
    [dimJoints, ~] = size(u);
    
    % initialize output vector
    dx = zeros(dimStates,dimTimes);
    
    % update dx matrix
    for i = 1:dimTimes
        control = u(:,i);
        state = x(:,i);
        q_param = state( 1:dimJoints, :);           % configuration fileds
        dq = state( (dimJoints+1) : dimStates, :);  % joint velocity
        
        % calculate joint accelerations, given configuration with applied external forces
        ddq = forwardDynamics(robot_obj, q_param, dq, control);
        
        dx(:,i) = [dq; ddq];
    end
    
    
end