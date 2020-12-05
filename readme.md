# Matlab Code of Robot Arm Trajectory Planning

## Summary
I optimizaed the multi-joint robot arm's trajectory planning by balancing manipulability maximization and control cost minimization, as my 2020 summer robotics research project at McGill University. 

The optimized planned was tested in simulation on KUKA iiwa and Kinova Gen3 to verify the effectiveness, and the robot arms indeed generated much natural configurations than the original linear planner.

## Future Works
Because of the unfortunate situation this year, I was not able to test the algorithm using the real robot, and the Kinova Movo robot in our lab also needs some software support (i.e. migaraion to Gazebo 9.x). Furthermore, parameter tuning can be done using more advanced techniques.