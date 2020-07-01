function dz = dynAcc(z, u)	
	% function computes dynamics
	% inputs: z = [x;v1;v2]
	% 		  u = [u1;u2]

	% Unpack the inputs
	x = z(1:3,:);   % position
	v1 = z(4:6,:);  % rates (dynamics)
	% v2 = z(7:9,:);  % rates (integrator)    %unused
	u1 = u(1:2,:);  % actuators
	u2 = u(3:5,:);  % kinematics

	

	% Chain integrators
	dx = v1;
	%dv1 = dynQuadRotor(x, u1, p);	% Call to the actual dynamics function
	dv1 = u2;
	dv2 = u2;

	% Pack up the outputs
	dz = [dx; dv1; dv2];

end