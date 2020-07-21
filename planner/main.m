clear all ; close all ; 

%% robot setup

robot     = importrobot('iiwa14.urdf');
robot.DataFormat = 'column';
dim_joint = numel(robot.homeConfiguration) ; % Get number of joints 


% random control constraints
uMax = 50 * ones(dim_joint,1);


% initial & final configurations setup
joint_pos_init = robot.homeConfiguration;   % 7x1 matrix
%joint_pos_final = robot.randomConfiguration;
joint_pos_final = [-1.34651803292669;1.35260787526178;
                   1.11890660499873;0.435225571215641;
                   -0.670851645972758;-1.81988132007208;
                   3.04573368893129];


%% trajectory optimization

% state  x = [q;dq]
%       dx = [dq;ddq]


%quadTol = 1e-1; %Compute quadrature to this tolerance

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

boundary_control = zeros(dim_joint, 1);
problem.guess.control = [boundary_control, boundary_control];



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


problem.options.nlpOpt = optimset(...
    'display','iter',...
    'MaxFunEvals',3.5e3,...
    'TolCon',1.4e-3,...
    'TolFun',3e-2);

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





%{

First 60 iterations
                                            First-order      Norm of
 Iter F-count            f(x)  Feasibility   optimality         step
    0     318    0.000000e+00    1.646e-01    6.570e-10
    1     636    3.956086e-01    1.363e-01    3.438e-01    1.809e+00
    2     954    4.066279e-01    2.452e-02    1.157e-01    4.137e-01
    3    1272    3.302768e-01    3.951e-03    9.802e-02    3.393e-01
    4    1590    2.250568e-01    3.238e-02    8.249e-02    8.229e-01
    5    1908    1.811828e-01    1.451e-02    8.546e-02    6.571e-01
    6    2226    1.538697e-01    1.288e-02    5.137e-02    5.643e-01
    7    2544    1.334225e-01    1.168e-02    5.374e-02    4.818e-01
    8    2862    1.143134e-01    1.766e-02    5.895e-02    5.039e-01
    9    3180    1.019665e-01    1.155e-02    6.006e-02    3.718e-01
   10    3498    9.258061e-02    1.025e-02    4.045e-02    3.084e-01
   11    3817    8.170480e-02    2.138e-03    4.435e-02    4.537e-01
   12    4136    7.183993e-02    1.836e-03    4.509e-02    4.847e-01
   13    4455    6.656103e-02    8.262e-04    3.276e-02    2.845e-01
   14    4774    6.302021e-02    3.075e-04    2.521e-02    2.039e-01
   15    5093    5.966545e-02    3.888e-04    3.047e-02    2.647e-01
   16    5412    5.668382e-02    1.245e-03    2.327e-02    3.169e-01
   17    5731    5.422762e-02    1.796e-03    2.232e-02    3.403e-01
   18    6050    5.221180e-02    8.066e-04    2.124e-02    3.106e-01
   19    6369    5.041909e-02    3.152e-04    1.707e-02    2.990e-01
   20    6687    4.858833e-02    2.559e-03    1.653e-02    2.090e-01
   21    7005    4.639780e-02    4.003e-03    2.263e-02    2.821e-01
   22    7323    4.385036e-02    5.814e-03    3.180e-02    3.726e-01
   23    7641    3.973495e-02    2.001e-02    4.072e-02    6.212e-01
   24    7959    3.384125e-02    4.653e-02    4.535e-02    9.641e-01
   25    8277    2.786952e-02    7.386e-02    5.887e-02    1.188e+00
   26    8595    2.504340e-02    2.961e-02    4.009e-02    9.108e-01
   27    8913    2.333850e-02    1.934e-02    3.317e-02    7.904e-01
   28    9231    2.210499e-02    5.791e-03    1.963e-02    5.267e-01
   29    9549    2.111841e-02    5.678e-03    1.257e-02    4.506e-01
   30    9867    2.061783e-02    1.992e-03    1.495e-02    3.414e-01

                                            First-order      Norm of
 Iter F-count            f(x)  Feasibility   optimality         step
   31   10185    2.024570e-02    5.695e-04    8.398e-03    2.321e-01
   32   10503    1.973049e-02    1.703e-03    1.146e-02    3.299e-01
   33   10821    1.913830e-02    4.528e-03    1.445e-02    4.238e-01
   34   11139    1.861084e-02    3.269e-03    1.638e-02    4.775e-01
   35   11457    1.829545e-02    3.069e-03    1.417e-02    4.403e-01
   36   11775    1.804821e-02    9.874e-04    1.379e-02    3.044e-01
   37   12093    1.770311e-02    1.747e-03    1.190e-02    1.960e-01
   38   12411    1.725625e-02    4.360e-03    1.192e-02    3.709e-01
   39   12729    1.695729e-02    5.246e-03    1.043e-02    4.629e-01
   40   13047    1.648145e-02    4.890e-03    1.054e-02    5.238e-01
   41   13366    1.572519e-02    2.242e-03    1.939e-02    1.282e+00
   42   13685    1.526519e-02    2.377e-03    2.294e-02    1.055e+00
   43   14004    1.486020e-02    2.393e-03    1.578e-02    8.769e-01
   44   14324    1.454768e-02    3.457e-03    1.596e-02    6.856e-01
   45   14643    1.404998e-02    3.168e-03    2.357e-02    1.399e+00
   46   14962    1.368464e-02    3.110e-03    2.104e-02    8.811e-01
   47   15280    1.322459e-02    6.619e-03    2.061e-02    4.214e-01
   48   15598    1.275295e-02    7.079e-03    1.782e-02    5.293e-01
   49   15916    1.224418e-02    8.045e-03    2.123e-02    4.479e-01
   50   16234    1.147554e-02    1.084e-02    1.681e-02    4.656e-01
   51   16553    1.077206e-02    5.107e-03    2.163e-02    5.816e-01
   52   16872    1.045605e-02    7.024e-03    2.856e-02    8.866e-01
   53   17190    9.645515e-03    3.791e-03    2.143e-02    4.167e-01
   54   17508    9.372556e-03    5.239e-03    2.094e-02    1.370e-01
   55   17826    9.057946e-03    1.476e-03    1.515e-02    7.220e-02
   56   18144    8.740016e-03    2.375e-03    1.675e-02    2.215e-01
   57   18462    8.525155e-03    2.633e-03    1.057e-02    6.855e-02
   58   18781    8.317398e-03    5.655e-04    1.445e-02    1.132e-01
   59   19100    8.063082e-03    2.894e-04    7.881e-03    7.508e-02
   60   19418    7.847431e-03    2.024e-03    9.646e-03    1.377e-01

                                            First-order      Norm of
 Iter F-count            f(x)  Feasibility   optimality         step
   61   19737    7.654262e-03    9.073e-05    1.398e-02    3.071e-01
   62   20056    7.439856e-03    1.080e-04    1.198e-02    2.758e-01

%}
