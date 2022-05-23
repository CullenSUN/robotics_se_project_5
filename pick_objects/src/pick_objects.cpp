#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef move_base_msgs::MoveBaseGoal MoveBaseGoal;

bool move_to_goal(MoveBaseClient& client, float x, float y, float yaw) {
  MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.z = yaw;
  goal.target_pose.pose.orientation.w = 1.0;

  client.sendGoal(goal);
  client.waitForResult();
  if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to goal");
  else
    ROS_INFO("The base failed to move goal for some reason");
  return client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_to_goal(ac, -4.4, -6.0, -1.57);
  move_to_goal(ac, -1.5, 2.0, 0.0);

  // -2, 2.5, 1
  // ac.sendGoal(pick_up_goal);
  // ac.waitForResult();
  // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //   ROS_INFO("Hooray, the base moved to pick_up_goal");
  // else
  //   ROS_INFO("The base failed to move pick_up_goal for some reason");

  // MoveBaseGoal drop_off_goal = makeGoalMsg(-1.5, 2.0, 0.0);
  // // -2, 2.5, 1
  // ac.sendGoal(drop_off_goal);
  // ac.waitForResult();
  // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //   ROS_INFO("Hooray, the base moved to drop_off_goal");
  // else
  //   ROS_INFO("The base failed to move drop_off_goal for some reason");


  return 0;
}
