#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" 