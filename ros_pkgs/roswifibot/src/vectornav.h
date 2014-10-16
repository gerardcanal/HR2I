#ifndef SERVER_VECTORNAV_H
#define SERVER_VECTORNAV_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

/* declaring the VectorNav Class for publishing VN data using ROS */
class VectorNav
{
 public:
   /*Class Constructor */  
   VectorNav(const ros::NodeHandle& nh);
  /*Class Destructor */
  ~VectorNav();

 private:
  /*Creating a handle attribute of VectorNav as a ROS Node */
  ros::NodeHandle _VNNodeHandle;

  /*Create an update function for updating VN data*/
  //void update ();
  
  /* Creating an object attribute for publishing VN data using ROS */ 
  ros::Publisher _vectornavYPRPublisher;
  /* Creating an object attribute for publishing VN data using ROS */ 
  ros::Publisher _vectornavQuaternionPublisher;
  /* Creating an object attribute for publishing Acceleration Rate using ROS */ 
  ros::Publisher _vectornavAccelerationPublisher;
  /* Creating an object attribute for publishing Acceleration Rate using ROS */ 
  ros::Publisher _vectornavAngularRatePublisher;
 
  /* Defining the YPR attributes to be published */    
  double _yaw;
  double _pitch;
  double _roll;
  
  /* Defining the Quaternion attributes to be published */    
  double _q0;
  double _q1;
  double _q2;
  double _q3;

  /* Defining the Acceleration and Angular Acceleration to be published */
  double _c0;
  double _c1;
  double _c2;
};

#endif
