#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <add_markers/UpdateMarker.h>

class MarkersManager {
private:
  ros::ServiceServer service;
  ros::Publisher pub;

  visualization_msgs::Marker make_marker(int id, float x, float y, string action) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "home_service_robot_markers";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = action;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
  }

public:
  MarkersManager(ros::NodeHandle *nh) {
    service = nh.advertiseService("/update_marker", &MarkersManager::update, this);
    pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  }

  bool update(add_markers::UpdateMarker::Request &req, add_markers::UpdateMarker::Response &res) {
    string action;
    if (req.add_or_delete) {
      action = visualization_msgs::Marker::ADD;
    } else {
      action = visualization_msgs::Marker::DELETE;
    }
    visualization_msgs::Marker marker = make_marker(req.id, req.x, req.y, action);
    pub.publish(marker);
    ROS_INFO("request: id=%d, x=%f, y=%f, action:%s", req.id, req.x, req.y, action);

    res.success = true;
    res.message = "Marker updated";
    return true;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "manage_markers");
  ros::NodeHandle nh;
  MarkersManager manager = MarkersManager(&nh);
  ros::spin();
}
