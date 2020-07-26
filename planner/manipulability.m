function measure = manipulability(robot_obj, config_parameter)
    % function calculating manipulability   
    
    endEffector = robot_obj.BodyNames{end} ;           % Get end-effector frame name
    jacobian = robot_obj.geometricJacobian(config_parameter, endEffector);
    
    % jacobian = jacobian(4:6,:);           
    determinant = det( jacobian * transpose(jacobian) );
    measure = sqrt(determinant);
    
end