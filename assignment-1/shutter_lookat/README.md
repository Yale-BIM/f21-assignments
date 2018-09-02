# Shutter Look At Package

This ROS package has starting code for the [Assignment 1](../README.md) of CPSC-659 Building Interactive Machines.

## Python Nodes (whitin the scripts directory)

- **look_forward.py:** Node that makes shutter orient its zed camera forward. It exists when the camera
is looking forward.

- **generate_target.py:** Node that publishes the location of a simulated target in front of the robot.

- **look_at_target.py:** Base code for Part IV of the assignment.


## Launch files

- **generate_target.launch:** Launch file that brings up the robot, generates a simulated target, 
opens up rviz, and makes the robot look forward.