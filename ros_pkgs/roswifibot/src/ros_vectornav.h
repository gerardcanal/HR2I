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
  ros::Publisher _vectornavDataPublisher;
  
  /* Defining the attributes to be published */    
  double _yaw;
  double _pitch;
  double _roll;

};

#endif
