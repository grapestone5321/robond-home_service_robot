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




