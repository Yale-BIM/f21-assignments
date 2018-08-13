# Assignment 0

This assignment will provide you practical experience using the [Robot Operating 
System (ROS)](http://www.ros.org/) and [git](https://git-scm.com/).

#### System Requirements
You should have access to a computer with `Ubuntu 16.04` and `ROS Kinetic` to complete the homework. 
The instructions below assume that you are using a [bash shell](https://www.gnu.org/software/bash/) 
to do the assignment, and that have installed the *desktop-full* Kinetic version of ROS 
using `apt-get` as in this guide: 
[http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu).

> NOTE: If you have Ubuntu 18.04, you can also complete this homework 
using ROS Melodic. Make sure to follow the Melodic versions of the ROS tutorials linked below.

You should also have `git` installed in the machine that you are using to work on your assignment.
You will use git to save your work to your [GitLab](http://www.gitlab.com) repository.

#### Background Knowledge

If you do not have much experience working with a Linux shell, please read this 
[introduction to bash](http://cs.lmu.edu/~ray/notes/bash/) by R. Toal and 
[What Is The Bashrc File Used For? ](https://www.lifewire.com/bashrc-file-4101947) by G. Newell. 
At the bare minimum, you should be familiar with the commands `cd`, `ls`, `rm`, `mkdir`, `echo`, 
`nano` before starting Part I of the assignment.

#### Deliverables

- **Report:** You are expected to submit a pdf with answers to the assignment questions,
information needed to understand your code, and the specific commit SHA of the code accompanying
the report. Though not mandatory, you can generate this pdf 
with [Overleaf](https://www.overleaf.com/edu/yale#!overview) and the [LaTeX template]()
provided as part of this assignment.

- **Code:** You are also expected to push code to your [GitLab](http://www.gitlab.com) repository
as indicated in the [general instructions](../README.md) document for CPSC-659 assignments. 

## Part I - Setting up your workspace to work with Shutter

*Catkin* is the build system for ROS. To understand what it is for and why it exists, 
read sections 1, 2 and 4 of Catkin's conceptual overview document: 
[http://wiki.ros.org/catkin/conceptual_overview](http://wiki.ros.org/catkin/conceptual_overview).

Set up your Catkin workspace to work with the Shutter robot:

1. Create a [workspace](http://wiki.ros.org/catkin/workspaces) called *catkin_ws* 
in your home directory. Follow the steps in this tutorial: 
[http://wiki.ros.org/catkin/Tutorials/create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

    > The [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
    page has tabs for switching between ROS distributions. Follow the tutorial for the distribution of
    ROS that you have installed in your system, e.g., Kinetic.

2. Download Shutter's codebase into your workspace's `src` directory.

    ```bash
    # Go to the src folder in your workspace
    $ cd ~/catkin_ws/src

    # Clone the Shutter packages from GitLab
    $ git clone https://gitlab.com/interactive-machines/shutter/shutter-ros.git
 
    # Load git submodules with ROS dependencies
    $ cd shutter-ros
    $ git submodule init
    ```
    
    > [Git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) are other,
    external projects (Git repositories) that have been included in 
    the shutter-ros repository. These projects are needed to run the robot's base code.
    
    You should now have a number of directories in ~/catkin_ws/src/shutter-ros, including:
    
    ```bash
    $ cd ~/catkin_ws/src
    $ ls -C1 shutter-ros
    arbotix_ros
    moveit
    shutter_bringup
    shutter_description
    (...)
    ```
    
    Some of these directories are standard folders, other are ROS catkin packages. 
    A ROS catkin package contains:
    
    * A [catkin compliant package.xml](http://wiki.ros.org/catkin/package.xml) file
    that contains basic information about the package, e.g., package name, description,
    license, author, dependencies, etc.
    
    * A [CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt) file that is 
    used by catkin to build the software package.
    
    For example, the shutter_bringup package has the following files:
    
    ```bash
    # Example
    $ ls -C1 ~/catkin_ws/src/shutter-ros/shutter_bringup
    CMakeLists.txt
    config
    launch
    package.xml
    README.md
    ```
    
    > Each ROS package must have its own folder. This means that there cannot be
    nester packages. Multiple packages cannot share the same directory.
    
    Read the README.md file in the root level of the 
    [shutter-ros](https://gitlab.com/interactive-machines/shutter/shutter-ros.git) repository
    to understand its content and general organization.
        
3. Install other third-party dependencies with [rosdep](http://docs.ros.org/independent/api/rosdep/html/).
 
    ```bash
    # update rosdep 
    $ rosdep update

    # install dependencies for Shutter
    $ cd ~/catkin_ws
    $ rosdep install -y -r --ignore-src --rosdistro=kinetic --from-paths src
    ```
   
    > If rosdep is not found in your system, first install it and initialize it as
    indicated [here](http://docs.ros.org/independent/api/rosdep/html/overview.html). 
    You will need sudo access to complete this step. 
        
4. Build the packages in the src directory of your workspace with `catkin_make`. 

    ```bash
    # Build your workspace
    $ cd ~/catkin_ws
    $ catkin_make -DCMAKE_BUILD_TYPE=Release
    ```

    > You might want to select a different CMake build type other than Release (e.g. RelWithDebInfo or Debug).
    More options can be found in [cmake.org](http://cmake.org/cmake/help/v2.8.12/cmake.html#variable:CMAKE_BUILD_TYPE). 

    Now you should have a devel space in `~/catkin_ws/devel`, which contains its own setup.*sh files.
    Sourcing one of these setup.*sh files will `overlay` the install space onto your environment. 
    
    > Overlaying refers to building and using a ROS package from source on top of an existing version
    of that same package (e.g., installed to the system in /opt/ros/kinetic). For more information
    on overlaying, read [this tutorial](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying).
    
    > Add ```source ~/catkin_ws/devel/setup.bash``` at the end of your `.bashrc` file
     to automatically set up your environment with your workspace every time you open a new shell.
     Otherwise, make sure to source ~/catkin_ws/devel/setup.bash on every new shell that you want
     to use to work with ROS. Sourcing setup.bash from your devel space will ensure that ROS 
     can work properly with the code that you've added to and built in ~/catkin_ws.
                  

### Questions
Read more about catkin workspaces [here](http://wiki.ros.org/catkin/workspaces), 
and answer the following questions in your assignment report:

**PI-Q1.** What other directory was automatically created in ~/catkin_ws when you executed 
`catkin_make` and what is this directory for?

**PI-Q2.** The command `catkin_make` should have generated 3 types of setup files (setup.bash,
setup.sh, setup.zsh). What is the difference between these setup files?


## Part II - Bringing up Shutter

Now that you have setup Shutter's code in your catkin workspace, you will simulate
the Shutter robot and learn to use basic ROS tools to gather information about [ROS nodes](http://wiki.ros.org/Nodes)
-- processes that perform computation in your ROS system -- and 
[ROS messages](http://wiki.ros.org/Messages) -- data being sent from one node to another.
     
1. Open a new bash shell and start [roscore](http://wiki.ros.org/roscore). Roscore is a collection 
of nodes and programs that are pre-requisites of a ROS-based system.

    ```bash
    $ roscore
    ```
       
    > You must have roscore running in order for ROS nodes to communicate.
         
2. Open another terminal, and *bring up* a simulated robot 
with [roslaunch](http://wiki.ros.org/roslaunch).

    ```bash
    $ roslaunch shutter_bringup shutter.launch simulation:=true 
    ```
    
    > The term *bring up* is often used in ROS to denote the process of starting 
    the base functionality of a robot.
    
    > Roslaunch is a tool for easily launching multiple ROS nodes locally (or remotely
    via [SSH](https://en.wikipedia.org/wiki/Secure_Shell)) and setting up parameters 
    in ROS' [Parameter Server](http://wiki.ros.org/Parameter%20Server). You will make your
    own launch file later in this assignment.
    
    The shutter.launch file will then publish a 3D model of the robot (in [URDF format](http://wiki.ros.org/urdf))
    to the [ROS Parameter Server](http://wiki.ros.org/roscpp/Overview/Parameter%20Server). The model
    has information about the the joints of the robot and its sensors, including their placement
    relative to each other and properties. In the event that you ever need to, you can access the 
    URDF string describing the robot's model with the `rosparam` command:
    
    ```bash
    # Example
    $ rosparam get /robot_description
    "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<!-- ===================================================================================\
    \ -->\n<!-- |    This document was autogenerated by xacro from /home/shutter/shutter_ws/src/shutter-ros/shutter_description/robots/shutter.v.0.1.urdf.xacro\
    \ | -->\n<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                \
    \                 | -->\n<!-- ===================================================================================\
    \ -->\n<robot name=\"widowx_arm\" xmlns:xacro=\"http://ros.org/wiki/xacro\">\n \
    \ <transmission name=\"tran1\">\n    <type>transmission_interface/SimpleTransmission</type>\n\
    \    <joint name=\"joint_1\"/>\n    <actuator name=\"motor1\">\n      <hardwareInterface>EffortJointInterface</hardwareInterface>\n\
    \      <mechanicalReduction>1</mechanicalReduction>\n    </actuator>\n  </transmission>\n\
    \  <transmission name=\"tran2\">\n    <type>transmission_interface/SimpleTransmission</type>\n\
    \    <joint name=\"joint_2\"/>\n    <actuator name=\"motor2\">\n      <hardwareInterface>EffortJointInterface</hardwareInterface>\n\
    \      <mechanicalReduction>1</mechanicalReduction>\n    </actuator>\n  </transmission>\n\
    \  <transmission name=\"tran3\">\n    <type>transmission_interface/SimpleTransmission</type>\n\ 
    (...)
    ```
    
    or you can get it programmatically with the roscpp or rospy API as explained 
    [here](http://wiki.ros.org/roscpp/Overview/Parameter%20Server#Getting_Parameters) and 
    [here](http://wiki.ros.org/rospy/Overview/Parameter%20Server#Getting_parameters), respectively.
    
    > The ROS Parameter Server is a shared, multi-variate dictionary that is accessible via network APIs. 
    Nodes use this server to store and retrieve parameters at runtime. As it is not designed for 
    high-performance, it is best used for static, non-binary data such as configuration parameters. 
    It is meant to be globally viewable so that tools can easily inspect the configuration state of 
    the system and modify if necessary. 
    
    The shutter.launch script also runs two nodes in two different processes:
    
    - */arbotix*: robot driver from the [arbotix_python ROS package](https://github.com/marynelv/arbotix_ros).
    The driver provides a basic ROS interface to the robot's 
    [Arbotix RoboController](https://www.trossenrobotics.com/p/arbotix-robot-controller.aspx) 
    in its base, or equivalent simulation functionality.
    
    - */robot_state_publisher:* [state_publisher](http://wiki.ros.org/robot_state_publisher) node which 
    publishes the position of the joints of the robot to [tf](ros.org/wiki/tf).
    

### Questions
    
Complete the following tasks and answer the questions based on the status of your ROS system after
bringing up Shutter.
  
**PII-Q1.** ROS uses [topics](http://wiki.ros.org/Topics) to communicate information between nodes. 

> ROS topics are named buses over which data messages are exchanged. 
In general, nodes are not aware of who they are communicating with. 
Instead, nodes that are interested in data *subscribe* to the relevant topic; 
nodes that generate data *publish* to the relevant topic. 
There can be multiple publishers and subscribers to a topic. 

Using the `rqt_graph` node in the [rqt_graph package](http://wiki.ros.org/rqt_graph), generate an 
image of all of the nodes that are running in ROS after bringing up the robot and the topics that are used 
to exchange information.

```bash
$ rosrun rqt_graph rqt_graph
```

> Rosrun allows you to run an executable in a given package from a shell. The general usage is
as `$ rosrun <packge_name> <executable_name> [parameters]`.

> rqt_graph is part of the [rqt framework](http://wiki.ros.org/rqt) for GUI development in ROS. 
If rqt_graph is not already installed in your system, install it with 
```$ sudo apt-get install ros-kinetic-rqt ros-kinetic-rqt-common-plugins```.

Include the graph generated with rqt_graph in your report.


**PII-Q2.** Based on the graph that you generated in PII-QI, through which topic are the /arbotix and 
/robot_state_publisher node communicating in ROS?

**PII-Q3.** Nodes communicate with each other by publishing [messages](http://wiki.ros.org/Messages) 
to topics. Using the [rostopic tool](http://wiki.ros.org/rostopic), investigate what type 
of [ROS message]() is being sent/received through the topic that /arbotix and /robot_state_publisher 
are using to communicate with one another?

**Tip:** Query information about a topic with:
```bash
$ rostopic info <topic_name>
```


**PII-Q4.** What are the fields of the message type that is used to transmit information from 
/arbotix to /robot_state_publisher?

**Tip:** Query information about a message type with the [rosmsg tool](http://wiki.ros.org/rosmsg):
```bash
$ rosmsg show <message_type>
```


**PII-Q5.** Through which other topics is the /arbotix node publishing messages? 

**Tip:** Request information about a node with the [rosnode tool](wiki.ros.org/rosnode):
```bash
$ rosnode info <node_name>
```

Provide the list of topics and their respective message types in your report.


**PII-Q6.** Using the [rostopic echo](http://wiki.ros.org/rostopic#rostopic_echo) tool 
find out what is the current position and velocity of the 4 joints (motors) of the robot
as published by the /arbotix node.

**Tip:** Passing the parameter -n 1 to ```rostopic echo``` will only print a single message.


**PII-Q7.** Find out at what rate the /arbotix node is publishing the position of the joints
of the robot with the [rostopic hz](http://wiki.ros.org/rostopic#rostopic_hz) tool.

    
## Part III. Moving the robot one joint at a time

First bring up Shutter as in Part I. Then, follow the steps below to move the joints of the
robot.
     
1. Start [RViz](http://wiki.ros.org/rviz) with [rosrun](http://wiki.ros.org/rosbash#rosrun) to 
visualize the current state of the simulated robot.

    ```bash
    $ roscd shutter_description/config
    $ rosrun rviz rviz -d robotiew.rviz
    ```
    
    > The -d parameter of rviz tells the program to load a given configuration file. This
    file indicates what displays, tool properties and cameras to show in the interface. 
    See this [RViz user guide](http://wiki.ros.org/rviz/UserGuide) for more information. 
    
2. Control the position of the robot's joints with the `arbotix_gui` interface.

    ```bash
    $ rosrun arbotix_python arbotix_gui
    ```
    
    By moving the sliders in the GUI, you will be sending requests for new joint positions 
    to the robot. The requests are sent from the arbotix_gui ROS node (called /controllerGUI)
    to the arbotix driver node (/arbotix) through the /joint_*/command topic.
    