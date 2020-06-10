%% Negative Manipulability Function we would like to Minimize
function measure = manipulability(robot_obj, config_parameter, endEffector)
    config = robot_obj.homeConfiguration;
    for i = 1:numel(config_parameter)
        config(i).JointPosition = config_parameter(i);
    end
    
    % jacobian = jacobian(4:6,:); 
    jacobian = robot_obj.geometricJacobian(config, endEffector);          
    determinant = det(jacobian * transpose(jacobian));
    measure = sqrt(determinant);
end