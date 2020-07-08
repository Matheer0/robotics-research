function integrand = pathObj(x,u)
	% function computes the cost function integrand for the trajectory
	u
	product = u' * u;
    integrand = sum(product(:));
    size(integrand)
    
end