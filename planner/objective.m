function integrands =  objective(x,u,robot_obj,alpha)
    
	% function computes the cost function integrand array
    % of size 1 x dimTimes, where dimTimes = # of grid points
    
    [dimJoints, dimTimes] = size(u);
    integrands = zeros(1,dimTimes); 
    
    % iterate to update every scalar integrand
    for i = 1:dimTimes
        state = x(:,i);
        q_param = state( 1:dimJoints, :);  % configuration fileds
        measure = manipulability(robot_obj, q_param); % manipulability measure
        
        % combine u'* u and manipulability
        control = u(:,i); 
        integrands(i) = transpose(control) * control - alpha *  measure;
    
    end
    
    
end