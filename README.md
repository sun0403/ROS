# ROS Cube Stacking Project

This project demonstrates a robotic arm performing pick-and-place operations to stack cubes using ROS, Gazebo, and MoveIt. The process involves launching a simulation environment, spawning cubes, and running a control script.

## **Setup and Execution**

1. Launch Gazebo and RViz
Start the simulation environment using the following command:
```bash
roslaunch franka_zed_gazebo moveit_gazebo_panda.launch
2. Spawn Cubes in Gazebo
Generate the cubes in the simulation using:
rosrun franka_zed_gazebo spawn_cubes.py
3. Run the Stacking Operation
Execute the following command to control the robotic arm for stacking:

bash
roslaunch cube_stack stack.launch

