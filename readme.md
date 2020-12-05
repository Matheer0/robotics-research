# Matlab Code for Robot Arm Trajectory Planning

## Summary
I optimizaed the multi-joint robot arm's trajectory planning by balancing manipulability maximization and control cost minimization, as my 2020 summer robotics research project at McGill University. 

This optimized planner was tested in simulation on KUKA iiwa and Kinova Gen3 to verify the effectiveness, and the robot arms indeed generated much more natural configurations than using the original linear planner.

## Future Works
Because of the unfortunate situation this year, I was not able to test the algorithm using the real robot, and the Kinova Movo robot in our lab also needs some software support (i.e. migaraion to Gazebo 9.x). Furthermore, parameter tuning can be done using more advanced techniques.