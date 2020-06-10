%% Negative Manipulability Function we would like to Minimize
function measure = manipulability(robot_obj, config_parameter)
    config = robot_obj.homeConfiguration;
    for i = 1:numel(config_parameter)
        config(i).JointPosition = config_parameter(i);
    end
    
    endEffector = robot_obj.BodyNames{end};           % Get end-effector frame name
    jacobian = robot_obj.geometricJacobian(config, endEffector);
    
    % jacobian = jacobian(4:6,:);           
    determinant = det(jacobian * transpose(jacobian));
    measure = sqrt(determinant);
end