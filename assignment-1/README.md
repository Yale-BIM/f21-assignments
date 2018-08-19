# Assignment 1

This is the second assignment for Yale's CPSC-659 Building Interactive Machines course.

## Table of Contents

## Introduction 
This assignment will provide you practical experience with the [tf](ros.org/wiki/tf) ROS package and
the [MoveIt! Motion Planning Framework](https://moveit.ros.org/). You will also practice
a bit of math and geometry, e.g., to transform points across coordinate frames.

#### System Requirements
As for the first assignment, you should have access to a computer with `Ubuntu 16.04` and `ROS Kinetic` to complete the homework. 

> NOTE: If you have Ubuntu 18.04, you can also complete this homework 
using ROS Melodic. 

You should also have `git` installed in the machine that you are using to work on your assignment.
You will use git to save your work to your [GitLab](http://www.gitlab.com) repository.

#### Background Knowledge

This assignment assumes that you have already completed the first assignment ([assignment-0](../../assignment-0/README.md)) and,
thus, have set up your catkin workspace. You are also expected to have experience with Linux shells 
(e.g., [bash](https://www.gnu.org/software/bash/)), [git](https://git-scm.com/), and
the [Robot Operating System (ROS)](http://www.ros.org/). This includes being familiar with
the `roscore`, `rosrun`, `roslaunch`, `rostopic`, `rosmsg`, `rosnode`, `rqt_graph`, and `rviz` tools. You
should also know how to bring up a simulation of the Shutter robot in ROS, and
control the position of its joints one at a time. If
you are unfamiliar with any of these tools, programs, or procedures, please revisit the 
[assignment-0](../../assignment-0/README.md).

#### Deliverables

- **Report:** You are expected to submit a pdf with answers to the questions/tasks at 
the end of each part of the assignment. This report should also have any information needed 
to understand and/or run your code, as well as the specific commit SHA of the version of the code
that you would like to be evaluated on. Though not mandatory, it is recommended that you generate this pdf 
with [Overleaf](https://www.overleaf.com/edu/yale#!overview) and this 
[simple assignment template](https://www.overleaf.com/latex/templates/simple-assignment-template/mzkqqqjypzvd) 
in [LaTeX](https://www.latex-project.org/).

- **Code:** You are also expected to push code for this assignment to your 
[GitLab](http://www.gitlab.com) repository as indicated in the [general instructions](../README.md) 
document for CPSC-659 assignments. 

#### Evaluation

You assignment will be evaluated based on the content of your report and your code:

`todo: complete`

<!-- #### Further Reading and Tutorials -->

## Preliminaries

### Notation
We refer to `vectors` or column matrices with bold lower-case letters (e.g., $`\bold{x}`$).
Other `matrices`, such as linear transformations, are indicated with capital letters (e.g., $`R`$). 

### Conventions

In this course, all reasoning in space is done in a 
[right hand system](http://mathworld.wolfram.com/Right-HandRule.html). The orientation
of the cross product between $`\bold{i} = [1, 0, 0]^T`$ (in the direction of the $`x`$ axis) and 
$`\bold{j} = [0, 1, 0]^T`$ ($`y`$ axis) is determined by
placing $`\bold{i}`$ and $`\bold{j}`$ tail-to-tail, flattening the right hand, extending it in the direction
of $`\bold{i}`$, and then curling the fingers towards $`\bold{j}`$. The thumb then points in the direction
of $`\bold{k} = \bold{i} \times \bold{j} = [0,0,1]^T`$ (corresponding to the $`z`$ axis). 

<span style="display:block;text-align:center">
<img src="https://upload.wikimedia.org/wikipedia/commons/d/d2/Right_hand_rule_cross_product.svg"
width="100" alt="Right hand system from Wikipedia.org"/><br>
Right Hand System (image from Wikipedia.org)
</span>

We also use a [right hand rule](https://en.wikipedia.org/wiki/Right-hand_rule#Rotations) 
for rotations: right fingers are curled in the direction of rotation and the right thumb 
points in the positive direction of the axis.

<span style="display:block;text-align:center">
<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/3/34/Right-hand_grip_rule.svg/220px-Right-hand_grip_rule.svg.png"
width="100" alt="Right hand rule from Wikipedia.org"/><br>
Right Hand Rule (image from Wikipedia.org)
</span>


## Part I. Understanding 3D Geometric Transformations

3D spatial transformations are linear mappings between two vector spaces. 




## Part I. Introduction to tf
This part of the assignment will help you understand how [tf](http://wiki.ros.org/tf) works.

> tf is a package that lets the user keep track of multiple coordinate frames over time. tf 
maintains the relationship between coordinate frames in a tree structure buffered in time, and 
lets the user transform points, vectors, etc. between any two coordinate frames at any desired 
point in time. 

1. Complete the [Introduction to tf2](http://wiki.ros.org/tf2/Tutorials/Introduction%20to%20tf2)
tutorial from ROS. You should familiarize yourself with the `view_frames` and `tf_echo` tools. 
You should also learn to visualize the transforms in /tf and /tf_static with [rviz](http://wiki.ros.org/rviz).

### Questions / Tasks
Now that you know how to use basic tf tools, bring up a simulation of the robot Shutter. 
You will inspect its tf tree with tf tools. 

> NOTE: In [assignment-0](../../assignment-0/README.md), you ran `roscore` before bringing up the robot to enable ROS nodes to communicate. 
But you can also launch `shutter.launch` directly. If roscore isn't already running, roslaunch 
will automatically start it. Try it!

- **I-1.** Generate an image of the tf tree of Shutter with [view_frames](http://wiki.ros.org/tf/Debugging%20tools#Viewing_TF_trees). 
Include this image in your report.

    *Tip:* You can also generate the image with the 
    [rqt_tf_tree](http://wiki.ros.org/rqt_tf_tree) interface if you prefer.

- **I-2.** Based on the tf tree, which links are between the robot's *base_footprint* 
and *zed_camera_link*?

- **I-3.** How are the transformations in the /tf and /tf_static topics generated?

    *Tip:* You should inspect what nodes and topics are being published in your ROS system,
    e.g., with the [rqt_graph](http://wiki.ros.org/rqt_graph) tool. You can also read the shutter.launch script and any
    subsequent script that it launches to understand how the robot's tf tree is being generated.


## Part II. Understanding tf messages
The /tf and /tf_static topics transmit 
[tf2_msgs/TFMessage](http://docs.ros.org/jade/api/tf2_msgs/html/msg/TFMessage.html) messages.
These messages are a list of transformations, encoded as 
[geometry_msgs/TransformStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/TransformStamped.html) messages.

Each [geometry_msgs/TransformStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/TransformStamped.html) 
message has:

- a `header`, with a `stamp` of when the transform was published 
and the `frame_id` of the reference frame for the transformation;
- a `child_frame_id`, corresponding to the name of the child frame; and
- a `transform`, of type [geometry_msgs/Transform](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Transform.html),
with the translation and rotation of the child_frame_id in the reference frame_id.

These type of transformations are visualized with tools like [view_frames](http://wiki.ros.org/tf/Debugging%20tools#Viewing_TF_trees)
as:



### Questions / Tasks

- **I-3.** What is the 3D translation and rotation of the *zed_camera_link* from 
the *wrist_1_link* in the robot? 

    *Tip:* You can use the [tf_echo](http://wiki.ros.org/tf#tf_echo) tool.
    
## Part II. Computing the position of Shutter's camera relative to its base




## Part III. Solving the Inverse Kinematics problem with MoveIt!

## Part IV. Orienting Shutter's camera towards a target

## Part V. 


