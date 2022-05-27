This project is the 5th project for the "Robotics Software Engineer" NanoDegree course on Udacicy.

In this project, I used a turtlebot_simulator package to setup the basic gazebo environment, mapping and navigation components. I used rviz with a preconfigured workspace to view the simulation about mapping, path planning, and markers in real time. 

I mainly created the following two packages:
- **pick_objects**: set goal for the robot to pick up a simulated target, and then set another goal for the robot to drop off target. The main node is `pick_objects`.
- **add_markers**: add or delete markers from the rviz simulation. The main node is `manage_markers`.

The `pick_objects` node requests the `manage_markers` node to add or delete markers through "UpdateMarker" service. 

##How to run

Clone the following packages to your catkin_ws
- [gmapping](http://wiki.ros.org/turtlebot_gazebo)
- [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
- [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
- [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)
- This repo itself, [robotics_se_project_5](https://github.com/CullenSUN/robotics_se_project_5)
