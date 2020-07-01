clear all ; close all ; 

%% This section is copied from main.m
robot       = importrobot('iiwa14.urdf');
dim_joint   = numel(robot.homeConfiguration) ; % Get number of joints 
endEffector = robot.BodyNames{end} ; % Get end-effector frame name


time_step = 0.1 ; % time step, in seconds
hand_speed = 0.1; % m/s

%joint_pos_init = robot.randomConfiguration ; 
%hand_pos_init  = getTransform(robot, joint_pos_init, endEffector);  % current hand position given the random joint angles
%initial_position = tform2trvec(hand_pos_init);
initial_position = [0; 0; 0];
destination = [0.4; 0; 0.6];
%hand_pos_final = trvec2tform(destination) * axang2tform([0 1 0 pi]); % target hand position



%% trajectory optimization

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% x = state
% dx = rate
% ddx = acceleration
% 
% cost function: integral( ddx^2 )
% dynamics: ddx = f(x,dx,u)
% subject to:
%   		x(0)  = x0;
%   		x(f)  = xf;
%   		dx(0) = dx0;
%   		dx(f) = dxf;
%
%
% z = [x;v1;v2]
% u = [u1;u2]
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Problem set up 		                              %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
duration = 1;

x0 = initial_position;
dx0 = zeros(3,1);
z0 = [x0; dx0; dx0]; 


xf = destination;
dxf = zeros(3,1);
zf = [xf; dxf; dxf]; 
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
w = 1./[1,1,1];  % weighting vector for path objective
problem.func.dynamics = @(t,z,u)( dynamics(z,u) );
problem.func.pathObj = @(t,z,u)( pathObj(u,w) );	% accel-squared cost function


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = duration;
problem.bounds.finalTime.upp = duration;

problem.bounds.initialState.low = z0;
problem.bounds.initialState.upp = z0;
problem.bounds.finalState.low = zf;
problem.bounds.finalState.upp = zf;



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0,duration];
problem.guess.state = [z0, zf];
problem.guess.control = [ones(2,2); zeros(3,2)];



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.options.method = 'trapezoid'; 
problem.options.trapezoid.nGrid = 40;



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve the problem                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);















%{

%% Original task-space trajectory generation (linear)
[taskWaypoints, timeInterval]   = generate_trajectory(hand_pos_init, hand_pos_final, time_step, hand_speed) ; 
duration = timeInterval(2) - timeInterval(1);
dim_steps = ceil( duration/ time_step ) ;


%% Visualization

show(robot, joint_pos_init, 'PreservePlot', false, 'Frames', 'off');
hold on
axis([-1 1 -1 1 -0.1 1.5]);


for n = 1:dim_steps

    pose_desired = tform2trvec(taskWaypoints(:,:,n)) ;    
    plot3(pose_desired(1), pose_desired(2), pose_desired(3), 'r.', 'MarkerSize',20)
    
    % draw destination 
    plot3(destination(1), destination(2), destination(3), 'g.', 'MarkerSize',20)
    drawnow;
end

%}
