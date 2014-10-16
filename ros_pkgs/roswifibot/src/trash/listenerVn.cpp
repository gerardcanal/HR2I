#include "std_msgs/String.h"
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "vectornav.h" // it contains link to vn100.h and math.h
#include "vn100.h" //need to access vn100 functions
#include "vncp_services.h"
#include "ros_vectornav.h" //define the interface of the class
#include "ros/ros.h"
#include "roswifibot/vn100_msg.h" //define the message type of vn100



void vnStatusCallback(roswifibot::vn100_msg msg)
{
printf("\n\n-----------------------------------\n");
ROS_INFO("I am getting the Status values");
ROS_INFO("I heard Yaw: [%0.3f]",  msg.yaw );
ROS_INFO("I heard Pitch: [%0.3f]",  msg.pitch );
ROS_INFO("I heard Roll: [%0.3f]",  msg.roll );

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listenerVN");
  ros::NodeHandle n;
  ROS_INFO("Vector NAV Listener subscriber");
  //ros::Subscriber sub = n.subscribe("odom", 10, odomCallback);
 ros::Subscriber sub = n.subscribe("vn_ypr_topic", 10, vnStatusCallback);

  
  ros::spin();

  return 0;
}
