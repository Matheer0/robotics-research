function ddq = dynamics(robot_object, config_parameter, dq, u)

    % Calculate joint accelerations for given configuration with applied external forces
    robot_object.DataFormat = 'column';
    
    config = robot_obj.homeConfiguration;
    for i = 1:numel(config_parameter)
        config(i).JointPosition = config_parameter(i);
    end
    
	ddq = forwardDynamics(robot_object, config, dq, u);
    
end