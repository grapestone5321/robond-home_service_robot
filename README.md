# Robond Home Service Robot
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


## Wall Follower

Now that you’ve manually performed SLAM, it’s time to automate the process and let your robot follow the walls and autonomously map the environment while avoiding obstacles. To do so, we’ll get rid of the keyboard teleop node and instead interface the robot with a wall_follower node.

### Wall Follower Algorithm

You might be wondering what’s a wall follower algorithm? A wall follower algorithm is a common algorithm that solves mazes. This algorithm is also known as the left-hand rule algorithm or the right-hand rule algorithm depending on which is your priority. The wall follower can only solve mazes with connected walls, where the robot is guaranteed to reach the exit of the maze after traversing close to walls. We will implement this basic algorithm in our environment to travel close to the walls and autonomously map it. 

Here’s the wall follower algorithm(the left-hand one) at a high level:

``` bash
If left is free:
    Turn Left
Else if left is occupied and straight is free:
    Go Straight
Else if left and straight are occupied:
    Turn Right 
Else if left/right/straight are occupied or you crashed:
    Turn 180 degrees
```


This algorithm has a lot of disadvantages because of the restricted space it can operate in. In other words, this algorithm will fail in open or infinitely large environments. Usually, the best algorithms for autonomous mapping are the ones that go in pursuit of undiscovered areas or unknown grid cells. 

### Wall Follower node

For this project, you will be given the C++ wall_follower node. However, if you are up to the challenge, you can code it yourself. To do so, you will need to subscribe to the laser measurements of the robot and process them. To do this, place the robot in front of different blocks, then read back the measurements. This way you can identify the robot’s left side, right side, and forward side as well as distinguish between objects and free spaces. After processing the measurements, you must send your robot to the corresponding direction by publishing driving commands to actuate its wheels. 

Here’s the given version of the wall_follower node that you can grab from the GitHub repo. Go through this code and the comments to understand how the measurements are processed and how the wheels are actuated. This code implements the left-hand wall follower algorithm described earlier, but it’s a bit more optimized to avoid obstacles.

### gmapping Parameters

Notice that map shown here is not 100% accurate, but still resembles the environment. That’s because the gmapping parameters values used were the default values. In general, it’s essential to tune them in order to get a 100% accurate map. These parameters are all listed under the gmapping documentation, where you can look at them yourself. If you experiment with some of these parameter values, you should be able to get better maps. For example, you might try,
- reducing the angularUpdate and linearUpdate values so the map gets updated for smaller ranges of movements,
- reducing the x and y limits, which represent the initial map size, 
- increasing the number of particles.

You can try tweaking these parameters and/or any other parameter you think should be changed. You can also leave them as default if you wish, as long as you think your robot will be able to travel to two different positions that you will choose at a later time. 

## Testing Navigation

The next task of this project is to pick two different goals and test your robot's ability to reach them and orient itself with respect to them. We'll refer to these goals as the pickup and drop off zones. This section is only for testing purposes to make sure our robot is able to reach these positions before autonomously commanding it to travel towards them. 

We will be using the ROS Navigation stack, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm, to plan our robot trajectory from start to goal position. The ROS navigation stack permits your robot to avoid any obstacle on its path by re-planning a new trajectory once your robot encounters them. You are familiar with this navigation stack from the localization project where you interfaced with it and sent a specific goal for your robot to reach while localizing itself with AMCL. If you are planning to modify the ROS navigation algorithm or you are curious to know how it's done, take a look at this official tutorial which teaches you how to write a global path planner as a plugin in ROS. 

### Test it

Once you launch all the nodes, you will initially see the particles around your robot, which means that AMCL recognizes the initial robot pose. Now, manually point out to two different goals, one at a time, and direct your robot to reach them and orient itself with respect to them. 

## Reaching Multiple Goals

Earlier, you tested your robot capabilities in reaching multiple goals by manually commanding it to travel with the 2D NAV Goal arrow in rviz. Now, you will write a node that will communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach. As mentioned earlier, the ROS navigation stack creates a path for your robot based on Dijkstra's algorithm, a variant of the Uniform Cost Search algorithm, while avoiding obstacles on its path.

There is an official ROS tutorial that teaches you how to send a single goal position and orientation to the navigation stack. You are already familiar with this code from the Localization project where you used it to send your robot to a pre-defined goal. Check out the tutorial and go through its documentation. 

Here’s the C++ code of this node which sends a single goal for the robot to reach. I included some extra comments to help you understand it:

``` bash
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
```


## Customize the code

You will need to modify this code and edit its node name to pick_objects. Then, edit the frame_id to map, since your fixed frame is the map and not base_link. After that, you will need to modify the code and include an extra goal position and orientation for your robot to reach. 

The first goal should be your desired pickup goal and the second goal should be your desired drop off goal. The robot has to travel to the desired pickup zone, display a message that it reached its destination, wait 5 seconds, travel to the desired drop off zone, and display a message that it reached the drop off zone. 


## Modeling Virtual Objects

The final task of this project is to model a virtual object with markers in rviz. The virtual object is the one being picked and delivered by the robot, thus it should first appear in its pickup zone, and then in its drop off zone once the robot reaches it. 

First, let’s see how markers can be drawn in rviz. Luckily, there’s an official ROS tutorial that teaches you how to do it. The tutorial is an excellent reference and includes a C++ node capable of drawing basic shapes like arrows, cubes, cylinders, and spheres in rviz. You will learn how to define a marker, scale it, define its position and orientation, and finally publish it to rviz. The node included in the tutorial will publish a different shape each second at the same position and orientation. Check out the tutorial and go through the documentation to get started. 

You will need to first run this node and visualize the markers in rviz. Then you’ll need to modify the code and publish a single shape example: a cube. Your code should follow this algorithm:
- Publish the marker at the pickup zone 
- Pause 5 seconds
- Hide the marker 
- Pause 5 seconds 
- Publish the marker at the drop off zone 

Later you will be able to combine this node with the pick_objects node coded earlier to simulate the full home service robot. 

## Putting it all Together

Now it’s time to simulate a full home service robot capable of navigating to pick up and deliver virtual objects. To do so, the add_markers and pick_objects node should be communicating. Or, more precisely, the add_markers node should subscribe to your odometry to keep track of your robot pose. 

Modify the add_markers node as follows:
- Initially show the marker at the pickup zone
- Hide the marker once your robot reaches the pickup zone 
- Wait 5 seconds to simulate a pickup
- Show the marker at the drop off zone once your robot reaches it 

### Hint

You might need to define a threshold for the position if the robot’s odometry values are noisy.

### Note

There are many ways to solve this problem. To establish communications between the robot and the markers, one method already mentioned is to let your add_markers node subscribe to your robot odometry and keep track of your robot pose. 

Other solutions to this problem might be to use ROS parameters, subscribe to the AMCL pose, or even to publish a new variable that indicates whether or not your robot is at the pickup or drop off zone. Feel free to solve this problem in any way you wish.


