%% Negative Manipulability Function we would like to Minimize
function measure = neg_manipulability(robot_obj, parameter)
    config = robot_obj.homeConfiguration;
    for i = 1:numel(parameter)
        config(i).JointPosition = parameter(i);
    end
    
    endEffector = robot_obj.BodyNames{end} ;           % Get end-effector frame name
    jacobian = robot_obj.geometricJacobian(config, endEffector);
    
    % jacobian = jacobian(4:6,:);           
    determinant = det( jacobian * transpose(jacobian) );
    measure = -1 * sqrt(determinant);
end