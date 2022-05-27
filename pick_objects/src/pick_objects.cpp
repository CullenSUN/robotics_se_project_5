#include <ros/ros.h>
#include <tuple>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <add_markers/UpdateMarker.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ObjectPicker {

private:
  ros::ServiceClient client;
  int pick_up_marker_id = 1;
  int drop_off_marker_id = 2;

  // pose.position.x, pose.position.y, pose.orientation.z
  std::tuple<float, float, float> pick_up_pose = std::make_tuple(-5.0, -7.0, 1.57); 
  std::tuple<float, float, float> drop_off_pose = std::make_tuple(-2.0, 2.5, 0.0); 

  move_base_msgs::MoveBaseGoal makeGoalMsg(std::tuple<float, float, float> pose) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = std::get<0>(pose);
    goal.target_pose.pose.position.y = std::get<1>(pose);
    goal.target_pose.pose.orientation.z = std::get<2>(pose);
    goal.target_pose.pose.orientation.w = 1.0;
    return goal;
  }

  void update_marker(bool add_or_delete, int id, float x, float y) {
    add_markers::UpdateMarker srv;
    srv.request.add_or_delete = add_or_delete;
    srv.request.id = id;
    srv.request.x = x;
    srv.request.y = y;
    if (client.call(srv)) {
      ROS_INFO("response: %s", srv.response.message);
    } else {
      ROS_ERROR("Failed to call service update_marker");
    }
  }

public:
  ObjectPicker(ros::NodeHandle *nh) {
    client = nh->serviceClient<add_markers::UpdateMarker>("update_marker");
  }

  void execute_task() {
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal pick_up_goal = makeGoalMsg(pick_up_pose);
    update_marker(true, pick_up_marker_id, std::get<0>(pick_up_pose), std::get<1>(pick_up_pose));
    ac.sendGoal(pick_up_goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      update_marker(false, pick_up_marker_id, std::get<0>(pick_up_pose), std::get<1>(pick_up_pose));
      ROS_INFO("Hooray, the base moved to pick_up_goal");
    } else {
      ROS_INFO("The base failed to move to pick_up_goal for some reason");
    }

    sleep(5);

    move_base_msgs::MoveBaseGoal drop_off_goal = makeGoalMsg(drop_off_pose);
    ac.sendGoal(drop_off_goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      update_marker(true, drop_off_marker_id, std::get<0>(drop_off_pose), std::get<1>(drop_off_pose));
      ROS_INFO("Hooray, the base moved to drop_off_goal");
    } else {
      ROS_INFO("The base failed to move to drop_off_goal for some reason");
    }
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle nh;
  ObjectPicker picker = ObjectPicker(&nh);
  picker.execute_task();
  ros::spin();
  return 0;
}
