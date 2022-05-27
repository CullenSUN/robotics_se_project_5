#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <add_markers/UpdateMarker.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef move_base_msgs::MoveBaseGoal MoveBaseGoal;

MoveBaseGoal makeGoalMsg(float x, float y, float yaw) {
  MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.z = yaw;
  goal.target_pose.pose.orientation.w = 1.0;
  return goal;
}

void update_marker(ros::ServiceClient *client, bool add_or_delete, int id, float x, float y) {
  add_markers::UpdateMarker srv;
  srv.request.add_or_delete = add_or_delete;
  srv.request.id = id;
  srv.request.x = x;
  srv.request.y = y;
  if (client->call(srv)) {
    ROS_INFO("response: %s", srv.response.message);
  } else {
    ROS_ERROR("Failed to call service update_marker");
  }
}

int main(int argc, char** argv) {
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<add_markers::UpdateMarker>("update_marker");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  int marker_id = 9;

  MoveBaseGoal pick_up_goal = makeGoalMsg(-5, -7.0, 1.57);
  update_marker(&client, true, -5, -7.0, marker_id);
  ac.sendGoal(pick_up_goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    update_marker(&client, false, -5, -7.0, marker_id);
    ROS_INFO("Hooray, the base moved to pick_up_goal");
  } else {
    ROS_INFO("The base failed to move to pick_up_goal for some reason");
  }

  sleep(5);

  MoveBaseGoal drop_off_goal = makeGoalMsg(-2, 2.5, 0.0);
  ac.sendGoal(drop_off_goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    update_marker(&client, true, -2, 2.5, marker_id);
    ROS_INFO("Hooray, the base moved to drop_off_goal");
  } else {
    ROS_INFO("The base failed to move to drop_off_goal for some reason");
  }
  
  ros::spin();
  return 0;
}
