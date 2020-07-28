function visualize_optTrajectory(robot, times, states, controls)
% visialize optimized trajectory

    dim_joint = numel(robot.homeConfiguration);
    endEffector = robot.BodyNames{end};
    
    % generalize each step
    for i = 1 : numel(times)
        
        % state  x = [q;dq]
        %       dx = [dq;ddq]
        
        x = states(:,i);
        q_param = x(1:dim_joint);
        
        transformation = getTransform(robot, q_param, endEffector);
        cartesian = tform2trvec(transformation);
          
        show(robot, q_param, 'PreservePlot', false, 'Frames', 'off');
        hold on
        axis([-1 1 -1 1 -0.1 1.5]);   
        plot3(cartesian(1), cartesian(2), cartesian(3), 'r.', 'MarkerSize',20)     
         
        drawnow;
        
    end
end
