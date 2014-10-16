#include "std_msgs/String.h"
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "libvectornav.h" // it contains link to vn100.h and math.h
#include "vn100.h" //need to access vn100 functions
#include "vncp_services.h"
#include "vectornav.h" //define the interface of the class
#include "ros/ros.h"
#include "roswifibot/vn100_msg.h" //define the message type of vn100
#include "roswifibot/VnQuaternion_msg.h" //define the message type of VectorSpace(Quaternion To get X ,Y, Z, W) 
#include "roswifibot/VnVector3_msg.h" //define the message type of 3 Compnenets used to get acceleration magnetic values through VN

// for async threads
#include <boost/thread.hpp>



void vnStatusCallback(roswifibot::vn100_msg msg)
{
printf("\n\n-----------------------------------\n");
ROS_INFO("I am getting the Status values");
ROS_INFO("I heard Yaw: [%0.3f]",  msg.yaw );
ROS_INFO("I heard Pitch: [%0.3f]",  msg.pitch );
ROS_INFO("I heard Roll: [%0.3f]",  msg.roll );
}

void vnQuaternionCallback(roswifibot::VnQuaternion_msg quanternionMsg)
{
printf("\n\n-----------------------------------\n");
ROS_INFO("I am getting the VectorSpace values");
ROS_INFO("The X is: [%0.3f]",  quanternionMsg.q0);
ROS_INFO("The Y is: [%0.3f]",  quanternionMsg.q1 );
ROS_INFO("THe Z is: [%0.3f]",  quanternionMsg.q2 );
ROS_INFO("THe W is: [%0.3f]",  quanternionMsg.q3 );
}

void vnAccelerationCallback(roswifibot::VnVector3_msg msg)
{
printf("\n\n-----------------------------------\n");
ROS_INFO("I am getting the Acceleration values");
ROS_INFO("Acceleration on X: [%0.3f]",  msg.c0 );
ROS_INFO("Acceleration on Y: [%0.3f]",  msg.c1 );
ROS_INFO("Acceleration on Z: [%0.3f]",  msg.c2 );
}

void vnAngularRateCallback(roswifibot::VnVector3_msg angulRateMsg)
{
printf("\n\n-----------------------------------\n");
ROS_INFO("I am getting the Angular Rate values");
ROS_INFO("AngularRate on X: [%0.3f]",  angulRateMsg.c0 );
ROS_INFO("AngularRate on Y: [%0.3f]",  angulRateMsg.c1 );
ROS_INFO("AngularRate on Z: [%0.3f]",  angulRateMsg.c2 );
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listenerVN");
  ros::NodeHandle n;
  ROS_INFO("Vector NAV Listener subscriber");
  //ros::Subscriber sub = n.subscribe("odom", 10, odomCallback);
 ros::Subscriber subVN = n.subscribe("vn_ypr_topic", 1, vnStatusCallback);
 ros::Subscriber subQuan = n.subscribe("vn_quaternion_topic", 1, vnQuaternionCallback);
 ros::Subscriber subAccel = n.subscribe("vn_acceleration_topic", 1, vnAccelerationCallback);
 ros::Subscriber subAngRate = n.subscribe("vn_angular_rate_topic", 1, vnAngularRateCallback);
      
      ros::AsyncSpinner vnStatusSpinner(4);
      vnStatusSpinner.start();
      ros::Rate threadRate(1);
   //ros::spin();
   //int i=0;
     while (ros::ok()){ // rosok bracket
     threadRate.sleep();
 // ros::spin();
      }
      
  return 0;
}
