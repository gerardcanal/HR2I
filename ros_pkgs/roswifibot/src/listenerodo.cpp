#include "libwifibot.h"
#include "wifibot.h"
#include "ros/ros.h"
#include "roswifibot/WStatus.h"
#include "std_msgs/String.h"
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void odomCallback(const nav_msgs::Odometry odom)
{
ROS_INFO("I am getting the values");
  ROS_INFO("I heard Postion X: [%d]",  odom.pose.pose.position.x);
  ROS_INFO("I heard Postion Y: [%d]",  odom.pose.pose.position.y);

}

void statusCallback(roswifibot::WStatus msg)
{
printf("\n\n-----------------------------------\n");
ROS_INFO("I am getting the Status values");
ROS_INFO("I heard Speed left: [%0.3f]",  msg.speed_front_left );
ROS_INFO("I heard Speed left: [%0.3f]",  msg.speed_front_right );
ROS_INFO("I heard current: [%0.3f]",  msg.current );
ROS_INFO("I heard version: [%d]",  msg.version );
ROS_INFO("I heard odometry left: [%0.3f]",  msg.odometry_left );
ROS_INFO("I heard odometry  right: [%0.3f]",  msg.odometry_right );
ROS_INFO("I heard adc [0]: [%d]",  msg.ADC1);
ROS_INFO("I heard adc [1]: [%d]",  msg.ADC2);
ROS_INFO("I heard adc [2]: [%d]",  msg.ADC3);
ROS_INFO("I heard adc [3]: [%d]",  msg.ADC4);

}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can
   * perform any ROS arguments and name remapping that were provided
   * at the command line. For programmatic remappings you can use a
   * different version of init() which takes remappings directly, but
   * for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the
   * ROS system. The first NodeHandle constructed will fully initialize
   * this node, and the last NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive
   * messages on a given topic.  This invokes a call to the ROS master
   * node, which keeps a registry of who is publishing and who is subscribing.
   * Messages are passed to a callback function, here called chatterCallback.  
   * subscribe() returns a Subscriber object that you must hold on to
   * until you want to unsubscribe. When all copies of the Subscriber
   * object go out of scope, this callback will automatically be
   * unsubscribed from this topic.
   *
   * The second parameter to the subscribe() function is the size of
   * the message queue.  If messages are arriving faster than they are
   * being processed, this is the number of messages that will be
   * buffered up before beginning to throw away the oldest ones.
   */


	ROS_INFO("I called the subscriber");
  //ros::Subscriber sub = n.subscribe("odom", 10, odomCallback);
 ros::Subscriber sub = n.subscribe("status", 100, statusCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this
   * version, all callbacks will be called from within this thread
   * (the main one).  ros::spin() will exit when Ctrl-C is pressed,
   * or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
