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

#### Further Reading and Tutorials

- [tf: The Transform Library](http://wiki.ros.org/Papers/TePRA2013_Foote?action=AttachFile&do=get&target=TePRA2013_Foote.pdf)

## Preliminaries

### Notation
We refer to `vectors` or column matrices with bold lower-case letters (e.g., $`\bold{x}`$).
Other `matrices`, such as linear transformations, and `scalars` are written with regular
font weight. 

### Conventions

In this course, all reasoning in space is done in a 
[right hand system](http://mathworld.wolfram.com/Right-HandRule.html). The orientation
of the cross product between $`\bold{i} = [1, 0, 0]^T`$ (in the direction of the $`x`$ axis) and 
$`\bold{j} = [0, 1, 0]^T`$ ($`y`$ axis) is determined by
placing $`\bold{i}`$ and $`\bold{j}`$ tail-to-tail, flattening the right hand, extending it in the direction
of $`\bold{i}`$, and then curling the fingers towards $`\bold{j}`$. The thumb then points in the direction
of $`\bold{k} = \bold{i} \times \bold{j} = [0,0,1]^T`$ (corresponding to the $`z`$ axis). 

<p align="center">
<img src="https://upload.wikimedia.org/wikipedia/commons/d/d2/Right_hand_rule_cross_product.svg"
width="100" alt="Right hand system from Wikipedia.org"/><br>
Right Hand System (image from Wikipedia.org)
</p>

We also use a [right hand rule](https://en.wikipedia.org/wiki/Right-hand_rule#Rotations) 
for rotations: right fingers are curled in the direction of rotation and the right thumb 
points in the positive direction of the axis.

<p align="center">
<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/3/34/Right-hand_grip_rule.svg/220px-Right-hand_grip_rule.svg.png"
width="75" alt="Right hand rule from Wikipedia.org"/><br>
Right Hand Rule (image from Wikipedia.org)
</p>

### Kinematic Chains

A kinematic chain is an assembly of rigid bodies or `links` connected by `joints`.
The joints allow the links to move relative to one another
and are typically instrumented with sensors, e.g., to measure the relative position of neihboring links.

There are different types of joints but, in this assignment, we will focus on
working with `revolute joints` because Shutter has 4 of them. Revolute joints
have a single axis of rotation and, thus, exibit just one Degree of Freedom. The 
`joint angle` of these revolute joints controls the displacement between the pair
of links that are connected to it.

In general, we like to think about Degrees of Freedom (DoF) as the number of independent position 
variables that would have to be specified in order to locate all parts of a robot.
Shutter, in particular, has 4 motors in its arm, each of which implements a revolute joint.
Thus, Shutter has 4 DoF. 


### 3D Transformations

3D spatial transformations map 3D points from one `coordinate system` (or `frame`) to another.
They are particularly relevant for robotics and 3D vision applications, where the 
elements of interest are in different locations in the world. For example, transformations
are useful to know the position of the camera in the Robot shutter relative to one of its 
links or its base. Similarly, 3D transformations can help infer the location of an object 
with respect to a camera that observes it.

Following [ROS conventions](http://wiki.ros.org/tf/Overview/Transformations), 
we refer to a point $`\mathbf{p}`$ within a frame $`B`$ as $`^{B}\mathbf{p}`$. 
We also refer to the relationship between any two frames $`A`$ and $`B`$ as 
a 6 Degrees of Freedom (DoF) transformation: 
a translation followed by a rotation. Specifically,
the pose of $`A`$ in $`B`$ is given by the translation from $`B`$'s origin to $`A`$'s origin, 
and the rotation of $`A`$'s coordinate axes in $`B`$. 

>- **Translations:** A 3D translation can be represented by a vector $`\bold{t} = [t_1, t_2, t_3]`$
    or by a $`4 \times 4`$ matrix in homogeneous coordinates:<br>
    $`t =
    \begin{bmatrix}
    1 & 0 & 0 & t_1\\
    0 & 1 & 0 & t_2\\
    0 & 0 & 1 & t_3\\
    0 & 0 & 0 & 1
    \end{bmatrix}
    `$<br>
    The scalars $`t_1`$, $`t_2`$, and $`t_3`$ correspond to the displacements in $`x`$,
    $`y`$, and $`z`$, respectively. Thus, a translation has 3 DoF. <br> <br>
    
>- **Rotations:** A 3D rotation has 3 DoF as well. Each DoF corresponds to a rotation around one of the axes of the 
    coordinate frame. We can represent rotations in homogeneous coordinates as:<br>
    $`R = 
    \begin{bmatrix}
    r_{11} & r_{12} & r_{13} & 0\\
    r_{12} & r_{22} & r_{23} & 0\\
    r_{13} & r_{32} & r_{33} & 0\\
    0 & 0 & 0 & 1
    \end{bmatrix}
    `$<br>
    Note that the $`3 \times 3`$ submatrix of $`R`$ with the elements $`r_11`$ ... $`r_33`$
    is an [orthogonal matrix](https://en.wikipedia.org/wiki/Orthogonal_matrix).
    
Be aware that [ROS uses quaternions](http://wiki.ros.org/tf2/Tutorials/Quaternions) 
to represent rotations, and there are many other rotation representations (e.g., 
see [Euler angles](https://en.wikipedia.org/wiki/Euler_angles)).

### Changing the Frame of a Point
Let $`^{A}\mathbf{p}`$ be a 3D point in the $`A`$ frame. Its position in 
$`B`$ can be expressed as $`^{B}\mathbf{p} = ^{B}_{A}T\ ^{A}\mathbf{p} = ^{B}_{A}(R \times t) ^{A}\mathbf{p}`$,
where:
 
- $`^{B}_{A}t`$ is the transformation in homogeneous coordinates that encodes the
 translation between the frames $`A`$ and $`B`$. In particular, the values $`t_1, t_2, t_3`$ of
the translation $`^{B}_{A}t`$ are the origin of the frame $`A`$ in $`B`$.
- $`^{B}_{A}R`$ is the rotation corresponding to the orientation of $`A`$'s coordinate axes in 
$`B`$. 

Note that the 3D vector with elements $`r_{11}, r_{21}, r_{31}`$ 
from the first column of the rotation matrix $`^{B}_{A}R`$ has the same direction as the $`x`$ axis of $`A`$ 
in the $`B`$ frame. Similarly, the elements $`r_{12}, r_{22}, r_{32}`$
and $`r_{13}, r_{23}, r_{33}`$ have the same direction of the $y$ and $z$ axes of 
$`A`$ in $`B`$, respectively.

### Transforms in ROS

The [tf](http://wiki.ros.org/tf) library in ROS represents transforms and coordinate frames 
in a `tree structure` buffered in time. The tree is a directed graph, where any two 
vertices are connected by one path. The nodes of this graph corresponds to coordinate frames,
each associated with a link, and the edges correspond to transforms between pairs of frames. 

Any directed edge in the tf tree has a `parent` frame (source node), and a `child` frame 
(target node). Let the parent frame be $`P`$ and the child be $`C`$. Then, the transform
stored in the edge parent -> child corresponds to $`^{P}_{C}T`$.

<!-- todo: add image of nodes and edge with transform -->

The tf library quickly computes the net transform between two nodes (frames) 
by multiplying the edges connecting them. To traverse up a directed edge from a child to a parent node, 
tf uses the inverse of the transformation that is stored in the edge.

<!-- todo: say something about querying transforms over time here? -->

## Part I. Introduction to tf
This part of the assignment will help you understand how [tf](http://wiki.ros.org/tf) lets 
users keep track of multiple coordinate frames over time in ROS. 

1. Complete the [Introduction to tf2](http://wiki.ros.org/tf2/Tutorials/Introduction%20to%20tf2)
tutorial from ROS. You should familiarize yourself with the `view_frames` and `tf_echo` tools. 
You should also learn to visualize the transforms in /tf and /tf_static with [rviz](http://wiki.ros.org/rviz).

### Questions / Tasks
Now that you know how to use basic tf tools, bring up a simulation of the robot Shutter. 
You will inspect its tf tree with tf tools. 

> NOTE: In [assignment-0](../../assignment-0/README.md), you ran `roscore` before bringing up the robot to enable ROS nodes to communicate. 
But you can also launch `shutter.launch` directly, as you did in the tutorial. 
If roscore isn't already running, roslaunch  will automatically start it. Try it!

- **I-1.** Generate an image of the tf tree of Shutter with [view_frames](http://wiki.ros.org/tf/Debugging%20tools#Viewing_TF_trees). 
Include this image in your report.

    *Tip:* You can also generate the image with the 
    [rqt_tf_tree](http://wiki.ros.org/rqt_tf_tree) interface if you prefer.

- **I-2.** Based on the tf tree from I-1, which frames are between the robot's *base_footprint* 
frame and the *zed_camera_link* frame?

- **I-3.** Based on the tf tree, what is the 3D transformation $`^{W}_{Z}T`$
between the *wrist_1_link* frame ($`W`$) and the *zed_camera_link* frame ($`Z`$)? Please
provide the transformation in homogeneous coordinates.

    *Tip:* You can use the [tf_echo](http://wiki.ros.org/tf#tf_echo) tool to query
    transformations. You will then need to assemble the $`4 \times 4`$ homogenous transformation matrix 
    from these values. We recommend [this primer](wiki.ogre3d.org/Quaternion+and+Rotation+Primer) from Ogre
    if you are confused about different rotation representations.

- **I-4.** Similar to the previous question, what is the 3D transformation $`^{B}_{F}T`$
between the *biceps_link* frame ($`B`$) and the *forearm_link* frame ($`F`$)? Again, please
provide the transformation in homogeneous coordinates.

- **I-5.** How are the transformations in the /tf and /tf_static topics generated after you 
bring up the robot? Please explain which node(s) contribute to generating the tf tree.

    *Tip:* You should inspect what nodes and topics are being published in your ROS system,
    e.g., with the [rqt_graph](http://wiki.ros.org/rqt_graph) tool. You can also read the shutter.launch script
    in the shutter_bringup package (and any subsequent script that it launches) 
    to understand how the robot's tf tree is being generated.


## Part II. Publishing tf messages
As mentioned earlier, the [tf](http://wiki.ros.org/tf) library
uses a tree structure to represent frames and transformations in ROS. These frames and transformations
are created based on the messages streamed through the /tf and /tf_static topics. By convention,
these topics transmit [tf2_msgs/TFMessage](http://docs.ros.org/jade/api/tf2_msgs/html/msg/TFMessage.html) messages,
which contain a list of transformations encoded as 
[geometry_msgs/TransformStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/TransformStamped.html) messages.

Each [geometry_msgs/TransformStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/TransformStamped.html) 
message has:

- a `header`, with a `stamp` of when the transform was published 
and the `frame_id` of the reference frame for the transformation;
- a `child_frame_id`, corresponding to the name of the child frame; and
- a `transform`, of type [geometry_msgs/Transform](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Transform.html),
with the translation and rotation of the transform $`{parent}_{child}T`$.

Read [this tutorial](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29) 
to understand how to publish tf transformations programmatically to the /tf topic.

### Questions / Tasks
Let's now create a simulated moving object in ROS.

- **II-1.** Modify the `generate_target.py` Python script 
in the `shutter_lookat` package of this assignment
to publish the location of a simulated object as a tf frame. The object is already
created in the script and its position is automatically updated, such that it moves on
a circular path relative to the "base_footprint" frame of Shutter. 

    For this part of the assignment, you should publish a new frame in /tf called "target" which contains the position
    of the simulated object in the "base_footprint" frame. For the orientation of the "target" 
    frame, you should set 0 deg. rotation with respect to "base_footprint".

    *Tip:* You can check that your code is working as expected by visualizing
    the "target" frame relative to the robot Shutter in [rviz](http://wiki.ros.org/rviz).
    To this end, bring up the simulated robot, run your new 
    generate_target.py script, open rviz, and add a RobotModel and TF displays. You should
    then be able to see the target moving in a circular path in front of the robot, as
    in the gif below.
    <!-- todo: add gif -->
    
    Once the "target" tf frame is published properly by your modified generate_target.py script, 
    please commit your code to your repository.
    
- **II-2.** Stop any ROS processes that you are running, including roscore. Then, 
    bring up the robot, run your new generate_target.py script, and 
    generate a new image of the tf tree in ROS, e.g., using [view_frames](http://wiki.ros.org/tf/Debugging%20tools#Viewing_TF_trees). 
    Add the image of the tf tree to your report.
    
### Part III. Querying TF
One of the key features of the [tf](http://wiki.ros.org/tf) library is the ability
to query transformations from the tf tree that is assembled


Read the [writing a tf listener](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29) 
and [learning about tf and time](http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28Python%29) tutorials.

## Part IV. Solving the Inverse Kinematics problem with MoveIt!

1. Find ray from camera center to target
2. Compute orientation of ray in world footprint frame
3. Change the orientation of the camera to point towards the target



    *Tip:* A tutorial in C++ on publishing rviz Markers can be found 
    [here](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes). You
    need to follow the same steps (but in Python) to publish a marker from your node.
    First, define the publisher that will stream Markers through a topic in your node.
    Then, create the Marker messages and publish them continuously as the position
    of the target changes over time. 

## Part IV. Orienting Shutter's camera towards a moving target




