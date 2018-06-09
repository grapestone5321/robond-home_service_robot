# robond-home_service_robot
Udacity robond home service robot project

## Project Submission

### Summary of Tasks

Let’s summarize what should be done in this project to simulate a home service robot

1. Design a simple environment with the Building Editor in Gazebo.
2. Teleoperate your robot and manually test SLAM.
3. Create a wall_follower node that autonomously drives your robot to map your environment.
4. Use the ROS navigation stack and manually commands your robot using the 2D Nav Goal arrow in rviz to move to 2 different desired positions and orientations.
5. Write a pick_objects node that commands your robot to move to the desired pickup and drop off zones. 
6. Write an add_markers node that subscribes to your robot odometry, keeps track of your robot pose, and publishes markers to rviz. 

### Evaluation

Once you get a working home service robot, check the Project Rubric to see if it meets the specifications listed. If you meet the specifications, then you are ready to submit the project!

### Here are the files that you need to include in your project submission

- Your catkin_ws/src directory which includes: 
  - The official ROS packages that you downloaded
  - C++ packages and nodes 
  - World files: gazebo world, pgm, and yaml files
  - Shell scripts


## Home Service Robot Project

Here's a detailed list of the steps in this project, check them off as you complete them:
- Write a Shell Script
- Design your environment with Building Editor
- Test SLAM
- Wall Follower
- Test Navigation
- Reach Multiple Goals
- Model Virtual Objects
- Put it all Together


### 1. Write a Shell Script

Now it's your turn to experiment with shell scripts and customize them by following these steps:
- Install xterm with sudo apt-get install xterm
- Create a launch.sh file
- Write one or multiple commands to be executed by the terminals
- Turn your script into an executable one with chmod +x launch.sh
- Launch the shell script file with ./launch.sh


### 2. Catkin Workspace Setup

Follow these instructions to prepare your catkin_ws/src directory for the project. Check off each step as you complete it!
- Update your system with sudo apt-get update
- Install the ROS navigation system with sudo apt-get install ros-kinetic-navigation
- Create a catkin_ws if you haven’t done so already
- Grab all the official ROS packages from GitHub
- Install all the packages dependencies
- Build your catkin workspace

### 3. Building Editor

Follow these steps to successfully design your environment with the Building Editor in Gazebo:
- Open a terminal and launch Gazebo
- Click edit and launch the Building Editor
- Design a simple environment
- Apply textures or colors
- Save the Building Editor environment and go back to Gazebo
- Save the Gazebo environment to the World directory of your catkin_ws/src

### 4. Testing SLAM

To manually test SLAM, create a test_slam.sh shell script that launches these files:
- The turtlebot_world.launch file to deploy a turtlebot in your environment
- The gmapping_demo.launch or run slam_gmapping to perform SLAM
- The view_navigation.launch to observe the map in rviz
- The keyboard_teleop.launch to manually control the robot with keyboard commands

### 5. Wall Follower

Follow these instructions to autonomously map your environment:
- Create a wall_follower package
- Create a wall_follower C++ node
- Edit the wall_follower C++ node name
- Edit the wall_follower C++ subscriber and publisher topics name
- Write a wall_follower.sh shell script that launch the turtlebot_world.launch, gmapping_demo.launch, view_navigation.launch, and the wall_follower node
- Edit the CMakeLists.txt file and add directories, executable, and target link libraries
- Build your catkin_ws
- Run your wall_follower.sh shell script to autonomously map the environment
- Once you are satisfied with the map, kill the wall_follower terminal and save your map in both pgm and yaml formats in the World directory of your catkin_ws/src.


### 6. Testing Navigation

Write a test_navigation.sh shell script that launches these files:
- Add turtlebot_world.launch to deploy a turtlebot in your environment
- Add amcl_demo.launch to localize the turtlebot
- Add view_navigation.launch to observe the map in rviz


### 7. Reaching Multiple Goals

Follow these instructions to autonomously command the robot to travel to both desired pickup and drop off zones:
Task List
- Create a pick_objects package with move_base_msgs, actionlib, and roscpp dependencies
- Create a pick_objects C++ node
- Edit the C++ node and modify it's node name and frame_id
- Modify the C++ node and publish a second goal for the robot to reach
- Display messages to track if robot successfully reached both zones
- Pause 5 seconds after reaching the pickup zone
- Edit the CMakeLists.txt file and add directories, executable, and target link libraries
- Build your catkin_ws
- Create a pick_objects.sh script file that launches the turtlebot, AMCL, rviz and your pick_objects node.


### 8. Modeling Virtual Objects

Follow these steps to create a virtual object in rviz:
- Create an add_markers package with roscpp and visualization_msgs dependencies
- Create an add_markers C++ node
- Copy the C++ code and edit the node name to add_markers
- Edit the frame_id
- Modify the C++ code to publish a single shape as describer earlier
- Edit the CMakeLists.txt file and add the executable, and libraries
- Build the catkin_ws
- Create an add_marker.sh shell script that launches the turtlebot, AMCL, rviz, and your add_markers node.
- Launch your shell script and manually add a Marker in rviz


### 9. Putting it all Together

Follow these steps to successfully simulate a home service robot:
- Edit the add_markers node and subscribe to odometry values
- Modify the C++ node as described earlier
- Build your catkin_ws
- Add markers to the view_navigation.launch file and save it as a new rviz configuration
- Create a home_service.sh file that launches the turtlebot, AMCL, rviz config file, pick_objects and add_markers nodes

## Working Environment

The home service robot project has been tested in three different working environments:
- Udacity Workspace: LUbuntu
- Virtual Machine: LUbuntu 
- Jetson Tx2: Ubuntu

Therefore, for this project, you have the choice of working on the Udacity Workspace which is highly recommended. Optionally you can chose to work on your Virtual Machine, or Jetson TX2. Keep in mind that the location of your catkin workspace is different in each. Here’s the directory of the catkin_ws for each of the provided working environments: 
- Lubuntu Udacity Workspace: /home/workspace
- LUbuntu VM: /home/robond
- Jetson Tx2: /home/nvidia

Working in the provided Udacity Workspace is highly recommended, because it's has a dedicated GPU and we can provide support for it. If you choose to work in the Udacity Workspace, refer to these instructions, move to the next concept, enable GPU, and GO TO DESKTOP!

## Shell Scripts

### Launching Nodes in ROS

For this project, you’ll rely heavily on shell scripts to launch and run nodes. When working with ROS, there are three common ways to launch/run nodes included in different packages:

1. Opening multiple terminals and launching/running a single node in each terminal.
- Advantage: You’ll be able to monitor the output of each node and easily identify bugs and errors. 
- Disadvantage: You’ll have many terminals open and you’ll have to repeat this procedure everytime you want to run or debug your project which is time consuming especially for large projects.

2. Writing a single launch file that groups all the nodes in all the packages and launching it within a single terminal. 
- Advantage: You’ll save a lot of time by running a single command. 
- Disadvantage: You won’t be able to easily track errors and bugs generated from different nodes. 

3. Writing a shell script file and programming it to launch one or many nodes each in a separate terminal. 
- Advantage: This technique is highly recommended since you’ll have the power to easily track the output of different nodes and you’ll save time by running a single command to launch all the nodes in the project. 
- Disadvantage: None.

### Launch File

Let’s start by taking a look at the launch.sh script where it’s launching both gazebo and rviz in separate instances of xterm terminals:

``` bash
#!/bin/sh
xterm  -e  " gazebo " &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " rosrun rviz rviz"
``` 

The launch.sh shell script launches three terminals and issues one or multiple commands in each terminal. Let’s break down this script to understand the meaning of each line.

1- #!/bin/sh
- This statement is called a shebang. It must be included in every shell script you write since it specifies the full path of the UNIX interpreter to execute it. 

2- xterm -e " gazebo " &
- With the xterm -e statement, we are launching a new instance of an xterminal. Inside this terminal, we are launching gazebo. Following, we are adding an ampersand & to indicate that another instance of an xterm terminal will be created in a separate statement. 

3 - sleep 5
- We are pausing this script for 5 seconds. 

4- xterm -e " source /opt/ros/kinetic/setup.bash; roscore" &
- We are launching a second instance of the xterm terminal. Inside this terminal, we are sourcing the ROS workspace and launching the ROS master. 

5 - sleep 5
- We are pausing this script again for 5 seconds.

6- xterm -e " rosrun rviz rviz"
- We are launching a third instance of the xterm terminal, and running rviz. 

After launching this script, we’ll have three open xterm terminals, and we will be able to track any errors or bugs that occur. To recap, this script will open the first terminal and launch gazebo. Then it will pause for 5 seconds and open a second terminal to launch the ROS master. It will pause for another 5 seconds and, finally, open a third terminal to launch RVIZ. 


## Catkin Workspace

To program your home service robot, you will need to interface it with different ROS packages. Some of these packages are official ROS packages which offer great tools and others are packages that you’ll create. The goal of this section is to prepare and build your catkin workspace. 

Here’s the list of the official ROS packages that you will need to grab, and other packages and directories that you’ll need to create at a later stage as you go through the project. Your catkin_ws/src directory should look as follows: 

### Official ROS packages

Import these packages now and install them in the src directory of your catkin workspace. Be sure to clone the full GitHub directory and not just the package itself. 

1. gmapping: With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.

2. turtlebot_teleop: With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.

3. turtlebot_rviz_launchers: With the view_navigation.launch file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you. 

4. turtlebot_gazebo: With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it. 

### Your Packages and Directories

You’ll install these packages and create the directories as you go through the project.

1. World: Inside this directory, you will store your gazebo world file and the map generated from SLAM.

2. ShellScripts: Inside this directory, you’ll store your shell scripts. 

3. RvizConfig: Inside this directory, you’ll store your customized rviz configuration files.

4. wall_follower: You will store a wall_follower node that will autonomously drive your robot around to perform SLAM.

5. pick_objects: You will write a node that commands your robot to drive to the pickup and drop off zones. 

6. add_markers: You will write a node that model the object with a marker in rviz. 

## Package Tree

Here's a high level overview of how you catkin_ws/src directory should look like:

``` bash
    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file                   
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file      
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file 
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├──                                # Your packages and direcotries
    |
    ├── World                          # world files
    │   ├── ...
    ├── ShellScripts                   # shell scripts files
    │   ├── ...
    ├──RvizConfig                      # rviz configuration files
    │   ├── ...
    ├──wall_follower                   # wall_follower C++ node
    │   ├── src/wall_follower.cpp
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──
```

### Troubleshooting

Q: What if my catkin_ws failed to build? 

A: There may be many reasons as to why your catkin_ws failed to build! Here are some common solutions to this problem:

Make sure you installed all the package dependencies with rosdep -i install <package name>.

Make sure you only have a single version of each package.

You can save your old catkin_ws and start over with a new one.

Look at the messages being generated on your terminal to identify the error.

## Testing SLAM

The next task of this project is to autonomously map the environment you designed earlier with the Building Editor in Gazebo. But before you tackle autonomous mapping, it’s important to test if you are able to manually perform SLAM by teleoperating your robot. The goal of this step is to manually test SLAM. 

Write a shell script test_slam.sh that will deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in rviz. We will be using turtlebot for this project but feel free to use your personalized robot to make your project stand out! 

### Launch it

Launch your test_slam.sh file, search for the xterminal running the keyboard_teleop node, and start controlling your robot. You are not required to fully map your environment but just make sure everything is working fine. You might notice that the map is low quality, but don’t worry about that for now. If everything seems to be working fine, move on to the next concept!

