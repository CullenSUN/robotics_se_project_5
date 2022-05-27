#!/bin/sh

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=\"$(rospack find add_markers)/../map/NewWorld.world\"" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=\"$(rospack find add_markers)/../map/map.yaml\"" &
sleep 5
xterm -e " roslaunch pick_objects view_navigation.launch" &
sleep 5
xterm -e " rosrun add_markers add_markers" 
