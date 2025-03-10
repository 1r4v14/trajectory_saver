#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/MarkerArray.h"
#include "trajectory_visualizer/SaveRequest.h"
#include <fstream>


visualization_msgs::MarkerArray marker_array_;

bool saveCallback(trajectory_visualizer::SaveRequest::Request &req, trajectory_visualizer::SaveRequest::Response &res) {

    ROS_INFO("Service Requestd, Saving..");
  std::ofstream file(req.filename);
  if(!file.is_open()) {
    res.success = false;
    res.filename = "ERROR";
    ROS_WARN("failed to open file");
    return true;
  }
  
  ros::Time now = ros::Time::now();

  for(auto& marker : marker_array_.markers) {

    if((now - marker.header.stamp).toSec() <= req.duration) {

      ROS_INFO("writing pose..");
      file << marker.pose.position.x << "," << marker.pose.position.y << "," << marker.pose.position.z << "," << marker.pose.orientation.x << "," << marker.pose.orientation.y << "," << marker.pose.orientation.z << "," << marker.pose.orientation.w << "\n";

    } 
  }

  file.close();
  res.filename = req.filename;
  res.success = true;
  ROS_INFO("Written to file: %s", req.filename);
  return true;
}


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "trajectory_saver");
  ros::NodeHandle n_;
  ros::Publisher marker_pub_;
  ros::ServiceServer service_ = n_.advertiseService("save_request", saveCallback);
 
  tf2_ros::Buffer tBuffer;
  tf2_ros::TransformListener tfListener(tBuffer);
  
  marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 10);

  ros::Rate rate(10.0);
  static int i = 0; 
  while(n_.ok()) {
    geometry_msgs::TransformStamped transformStamped;
 
    try {
      transformStamped = tBuffer.lookupTransform("odom","base_footprint", ros::Time(0));
      visualization_msgs::Marker marker_;

      marker_.header.frame_id = transformStamped.header.frame_id; 
      marker_.header.stamp = transformStamped.header.stamp;
      marker_.ns = "trajectory";
      marker_.id = i++;
      marker_.type = visualization_msgs::Marker::SPHERE;
      marker_.action = visualization_msgs::Marker::ADD;
      marker_.scale.x = 0.1;
      marker_.scale.y = 0.1;
      marker_.scale.z = 0.1;
      marker_.color.a = 1;
      marker_.color.r = 1;

  marker_.pose.position.x = transformStamped.transform.translation.x;
  marker_.pose.position.y = transformStamped.transform.translation.y;
  marker_.pose.position.z = transformStamped.transform.translation.z;
  marker_.pose.orientation = transformStamped.transform.rotation;
  marker_array_.markers.push_back(marker_); 
  marker_pub_.publish(marker_array_);
  
  ros::spinOnce();


    }
    catch(tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      sleep(0.1);
      continue;
    }
 

    rate.sleep();  
  } 

  return 0;
}
