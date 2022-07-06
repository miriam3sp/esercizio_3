# Exercise3

1. Implement a service server that computes the forward kinematics of a robot and a service client that uses this service and prints the solution to stdout.
2. Verify the result by comparing it with the service /compute_fk of the move_group node
3. Implement an action server that computes all inverse kinematic solutions of a robot (one by one) and an action client that uses this action and prints the solutions in stdout (one by one, as they are received) . The action server does not send the same solution twice and stops when all solutions have been found. At that moment, they are returned all together.
4. Repeat the experiment at point 3 by neglecting the joint limits
5. Visualize the IK solutions in RViz


## Start demo

catkin build

source devel/setup.bash

roslaunch kinematics_service server_fk.launch

source devel/setup.bash

roslaunch kinematics_service client_fk.launch

source devel/setup.bash

roslaunch kinematics_service compute_fk.launch





