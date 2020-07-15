function integrand =  objective(x,u)
    
	% function computes the cost function integrand array
    % size: 1 x dimTimes, where dimTimes = # of grid points
    
    [dimControl, dimTimes] = size(u);
    integrand = zeros(1,dimTimes);
    
    for i = 1:dimTimes
        control = u(:,i);
        integrand(i) = transpose(control) * control;
    end
    
    
end