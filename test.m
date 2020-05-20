clear all ; close all ; 

%% Robot Setup
robot       = importrobot('iiwa14.urdf');
joint_pos_init = robot.randomConfiguration;
q_input       = joint_pos_init;

%%
joints = numel(q_input);
jacobian = robot.geometricJacobian(q_input, robot.BodyNames{end});
jacobian = jacobian(4:6,:); 
pinv_jacobian = pinv(jacobian);


%% Find optimal q via optimization
%lb = -1 * pi * ones(joints , 1);  % joint types are revolute
%ub = -1 * lb;
lb = [];
ub = [];
A = [];
b = [];
Aeq = [];
beq = [];
q0 = [];
%q_opt = fmincon( @(q)neg_manipulability(robot, q), q0, A, b, Aeq, beq, lb, ub)
fun =  @(q) ( neg_manipulability(robot, q)  )?;
q_opt = fmincon (fun )?;


%% Output
q_input_param = [q_input(1: joints). JointPosition]; 
v_0     = q_opt - q_input_param;    % desired joint velocity in the null-space
                
null_space_projection = eye(joints) - pinv_jacobian * jacobian ;     % calculate the nullspace projection 
dq      = pinv_jacobian * desired_velocity(4:6) + null_space_projection * v_0' ; 
dq      = dq' 













% show(robot, q)

