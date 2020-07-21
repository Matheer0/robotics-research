clear all ; close all ; 

%% robot setup 
robot     = importrobot('iiwa14.urdf');
robot.DataFormat = 'column'; 
robot.Gravity = [0; 0; -9.81];
dim_joint = numel(robot.homeConfiguration) ; % Get number of joints 


% random (torque) control constraints
uMax = 50 * ones(dim_joint,1);


% initial & final configurations setup
joint_pos_init = robot.homeConfiguration;   % 7x1 matrix
%joint_pos_final = robot.randomConfiguration;
joint_pos_final = [-1.34651803292669;1.35260787526178;
                   1.11890660499873;0.435225571215641;
                   -0.670851645972758;-1.81988132007208;
                   3.04573368893129];
               
torq_init = gravityTorque(robot, joint_pos_init);  % torque control, 7x1 matrix
torq_final = gravityTorque(robot, joint_pos_final);



%% trajectory optimization

% state  x = [q;dq]
%       dx = [dq;ddq]


boundary_speed = zeros(dim_joint, 1);

initial_state = [joint_pos_init ; boundary_speed];
final_state   = [joint_pos_final ; boundary_speed];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% For all dynamics calculations, the data format must be either 'row' or 'column'.

problem.func.dynamics = @(t,x,u)( dynamics(x,u,robot) );
problem.func.pathObj = @(t,x,u)( objective(x,u) );	% accel-squared cost function


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

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


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0,10];
problem.guess.state = [initial_state, final_state];
problem.guess.control = [torq_init, torq_final];



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% TolCon: Termination tolerance on the constraint violation.
% TolFun: Termination tolerance on the function value.

problem.options.nlpOpt = optimset(...
    'display','iter',...
    'MaxFunEvals',3.5e3,...
    'TolCon',5e-3,...
    'TolFun',5e-2);

problem.options.defaultAccuracy = 'high';
problem.options.method = 'trapezoid';  % trapezoid direct collocation
problem.options.trapezoid.nGrid = 15;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve the problem                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display the solution                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

times    = soln.grid.time;     % 1xnGrid array
states   = soln.grid.state;   
controls = soln.grid.control;  

visualize_optTrajectory(robot, times, states, controls);


