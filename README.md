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


## Write a Shell Script

Now it's your turn to experiment with shell scripts and customize them by following these steps:
- Install xterm with sudo apt-get install xterm
- Create a launch.sh file
- Write one or multiple commands to be executed by the terminals
- Turn your script into an executable one with chmod +x launch.sh
- Launch the shell script file with ./launch.sh


## Catkin Workspace Setup

Follow these instructions to prepare your catkin_ws/src directory for the project. Check off each step as you complete it!
- Update your system with sudo apt-get update
- Install the ROS navigation system with sudo apt-get install ros-kinetic-navigation
- Create a catkin_ws if you haven’t done so already
- Grab all the official ROS packages from GitHub
- Install all the packages dependencies
- Build your catkin workspace

## Building Editor

Follow these steps to successfully design your environment with the Building Editor in Gazebo:
- Open a terminal and launch Gazebo
- Click edit and launch the Building Editor
- Design a simple environment
- Apply textures or colors
- Save the Building Editor environment and go back to Gazebo
- Save the Gazebo environment to the World directory of your catkin_ws/src

