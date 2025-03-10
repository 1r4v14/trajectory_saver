#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/TransformStamped.h"
#include <fstream>

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "trajectory_reader");
  ros::NodeHandle n_;
  visualization_msgs::Marker marker_;
  visualization_msgs::MarkerArray marker_array_;
  ros::Publisher read_pub_ = n_.advertise<visualization_msgs::MarkerArray>("read_trajectory", 100);
  int i = 0;

  marker_.header.frame_id = "odom";
  marker_.ns = "read_trajectory";
  marker_.type = visualization_msgs::Marker::SPHERE;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.scale.x = 0.1;
  marker_.scale.y = 0.1;
  marker_.scale.z = 0.1;
  marker_.color.a = 1;
  marker_.color.r = 1;

  std::ifstream trajData_(argv[1]);
  std::string line_; 
  while(trajData_.good()) {
    if(read_pub_.getNumSubscribers() >= 1) {
    getline(trajData_, line_, ',');
    float xp = atof(line_.c_str());

    getline(trajData_, line_, ',');
    float yp = atof(line_.c_str());

    getline(trajData_, line_, ',');
    float zp = atof(line_.c_str());

    getline(trajData_, line_, ',');
    float x = atof(line_.c_str());

    getline(trajData_, line_, ',');
    float y = atof(line_.c_str());

    getline(trajData_, line_, ',');
    float z = atof(line_.c_str());

    getline(trajData_, line_, '\n');
    float w = atof(line_.c_str());

    marker_.header.stamp = ros::Time::now();
    marker_.id = i++;
    marker_.pose.position.x = xp;
    marker_.pose.position.y = yp; 
    marker_.pose.position.z = zp;
    marker_.pose.orientation.x = x;
    marker_.pose.orientation.y = y;
    marker_.pose.orientation.z = z;
    marker_.pose.orientation.w = w;

   marker_array_.markers.push_back(marker_); 

   read_pub_.publish(marker_array_);
   }
  }


  return 0;
}
