function dx = dynamics(x,u,robot_obj)
    %config = robot_obj.homeConfiguration;
    
    % returns (dimStates * dimTimes) array, where
    % where size(x) = dimStates * dimTimes
    
    robot_obj.DataFormat = 'column';
    
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
        
        
        
        %{
        % assign parameters to configuration
        for j = 1:dimJoints
            config(j).JointPosition = q_param(j);
        end
        %}
        
        
        % calculate joint accelerations, given configuration with applied external forces
        ddq = forwardDynamics(robot_obj, q_param, dq, control);
     
        dx(:,i) = [dq; ddq];
    end
    
    
end