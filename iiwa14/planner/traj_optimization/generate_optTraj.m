function soln = generate_optTraj(robot, joint_pos_init,joint_pos_final, uMax, alpha)
    % trajectory optimization
    

    % state  x = [q;dq]
    %       dx = [dq;ddq]
    
    
    dim_joint = numel(joint_pos_init); % get number of joints
    boundary_speed = zeros(dim_joint, 1);

    initial_state = [joint_pos_init ; boundary_speed];
    final_state   = [joint_pos_final ; boundary_speed];


    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                     Set up function handles                         %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    % Note: for all dynamics calculations, the data format must be either
    %       'row' or 'column'
    %  (We have set to 'column' in main function)

    problem.func.dynamics = @(t,x,u)( dynamics(x,u,robot) );
    problem.func.pathObj = @(t,x,u)( objective(x,u,robot,alpha) );


    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                     Set up problem bounds                           %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

    problem.bounds.initialTime.low = 0;
    problem.bounds.initialTime.upp = 0;

    problem.bounds.finalTime.low = 0.1;
    problem.bounds.finalTime.upp = 100;

    problem.bounds.initialState.low = initial_state;
    problem.bounds.initialState.upp = initial_state;

    problem.bounds.finalState.low = final_state;
    problem.bounds.finalState.upp = final_state;

    problem.bounds.control.low = -uMax;
    problem.bounds.control.upp = uMax;


    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                    Initial guess at trajectory                      %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

    problem.guess.time = [0,10];
    problem.guess.state = [initial_state, final_state];

    boundary_control = zeros(dim_joint, 1);
    problem.guess.control = [boundary_control, boundary_control];


    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                     Solver options                                  %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


    problem.options.nlpOpt = optimset('display','iter',...
                                      'MaxFunEvals',3.5e5,...
                                      'TolCon',2e-2,...
                                      'TolFun',5e-1);

    problem.options.defaultAccuracy = 'high';
    problem.options.method = 'trapezoid';  % trapezoid direct collocation
    problem.options.trapezoid.nGrid = 15;


    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
    %                    Solve the problem                                %
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

    soln = optimTraj(problem);


end
