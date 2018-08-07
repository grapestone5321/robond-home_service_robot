#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/nvidia/catkin_ws/src/World/home_service_robot.world " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/nvidia/catkin_ws/src/World/my_map.yaml 3d_sensor:=kinect " &
sleep 5
xterm  -e  " rosrun rviz rviz -d /home/nvidia/catkin_ws/src/RvizConfig/navigation.rviz " &
sleep 5
xterm  -e  " rosrun add_markers add_markers " &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects "
