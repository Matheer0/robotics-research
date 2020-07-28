function measure = manipulability(robot_obj, config_parameter)
    % function calculating manipulability   
    
    endEffector = robot_obj.BodyNames{end} ;    % Get end-effector frame name
    jacobian = robot_obj.geometricJacobian(config_parameter, endEffector);
    
    % jacobian = jacobian(4:6,:);           
    determinant = det( jacobian * transpose(jacobian) );
    
    measure = real( sqrt(determinant) );
    % Note: real() is used because sqrt(determinant) may become very small 
    % during optimization process, and may have imaginary parts due to
    % numerical error, thus we just keep the real part
    
end