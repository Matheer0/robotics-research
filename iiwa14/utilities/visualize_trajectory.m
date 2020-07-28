function visualize_trajectory (robot, trajectory, destination)

    dim_joint = numel(robot.homeConfiguration) ;
    dim_steps = numel(trajectory.time) ;
    
    
    % Show the initial configuration of the robot.   
    config = trajectory.state(1:dim_joint, 1);
    show(robot, config, 'PreservePlot', false, 'Frames', 'off');
    hold on
    axis([-1 1 -1 1 -0.1 1.5]);

    
    % Visualize the task-space trajectory, by  
    % iterating through the stateTask states and interpolate 
    for i = 1 : dim_steps

        config = trajectory.state(1:dim_joint, i);    
        show(robot, config, 'PreservePlot', false, 'Frames','off');
        
        % draw a line to show the end-effector trajectory
        pose_desired = trajectory.hand_pos_desired(:, i);
        pose_current = trajectory.hand_pos_current(:, i);
        plot3(pose_desired(4), pose_desired(5), pose_desired(6), 'r.', 'MarkerSize',20)
        plot3(pose_current(4), pose_current(5), pose_current(6), 'b.', 'MarkerSize',20)
        % draw destination at every step
        plot3(destination(1), destination(2), destination(3), 'g.', 'MarkerSize',20)
        drawnow;
        
    end
    
    
end