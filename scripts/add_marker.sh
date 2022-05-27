#!/bin/sh

export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/robotics_se_project_5/map/NewWorld.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/catkin_ws/src/robotics_se_project_5/map/map.yaml

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e " roslaunch pick_objects view_navigation.launch" &
sleep 5
xterm -e " rosrun add_markers add_markers" 
