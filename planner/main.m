clear all ; close all ; 

%% robot setup

robot     = importrobot('iiwa14.urdf');
dim_joint = numel(robot.homeConfiguration) ; % Get number of joints 


% random control constraints
uMax = pi * ones(dim_joint,1);

joint_pos_init = robot.homeConfiguration; 
joint_pos_init_param = [joint_pos_init(1:dim_joint).JointPosition]';   % 7x1 matrix


joint_pos_final = robot.randomConfiguration;
joint_pos_final_param = [joint_pos_final(1:dim_joint).JointPosition]';   % 7x1 matrix


%% trajectory optimization

% state  x = [q;dq]
%       dx = [dq;ddq]

boundary_speed = zeros(dim_joint, 1);

initial_state = [joint_pos_init_param ; boundary_speed];
final_state   = [joint_pos_final_param ; boundary_speed];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% For all dynamics calculations, the data format must be either 'row' or 'column'.

robot.DataFormat = 'column';

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

boundary_control = zeros(dim_joint, 1);
problem.guess.control = [boundary_control, boundary_control];



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


problem.options.nlpOpt = optimset(...
    'display','iter',...
    'TolFun',1,...
    'TolCon',1e-1);

problem.options.defaultAccuracy = 'low';
problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 5;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve the problem                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display the solution                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%




