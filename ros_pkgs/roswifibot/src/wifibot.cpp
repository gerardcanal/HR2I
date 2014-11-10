#include "libwifibot.h"
#include "wifibot.h"
#include "ros/ros.h"
#include "roswifibot/WStatus.h"
#include "roswifibot/speed_msg.h" //define the message type Speed for passing it to the wifibot 
#include "std_msgs/String.h"
#include <sstream>

#define TWOPI (M_PI * 2)

Wifibot::Wifibot(const ros::NodeHandle& nh)
  : _nh (nh)
  , _updated(false)
  , _speedLeft(0.0)
  , _speedRight(0.0)
{
  // Parameters handler
  ros::NodeHandle pn("~");
  
  // Get device port parameter
  std::string dev;
  if (!pn.getParam("port", dev))
    {
      dev = "/dev/ttyS0";
      ROS_INFO("No device port set. Assuming : %s", dev.c_str());
    }
  
  // get base frame parameter
  std::string frameBase;
  if (!pn.getParam("base_frame", frameBase))
    _frameBase = "base_frame";
  else
    _frameBase = frameBase;
  
  // get entrax parameter
  double entrax;
  if (!pn.getParam("entrax", entrax))
    _entrax = 0.30;
  else
    _entrax = entrax;
  
  ROS_INFO("Wifibot device : %s. Entrax : %0.3f",
	   dev.c_str(), _entrax);

  // Create and configure the driver
  _pDriver = new wifibot::Driver(dev);
  _pDriver->setRelays(false, false, false);
  _pDriver->loopControlSpeed(0.01);  // Default loop control speed
  _pDriver->setPid(0.8, 0.45, 0.0);  // Default PID values
//  _pDriver->setSpeeds(10, 10);  // Default Set Speed
  _pDriver->setTicsPerMeter(5312.0); // Adapt this value according your wheels size
  

// Save initial position
wifibot::driverData st = _pDriver->readData();

	
_odometryLeftLast = st.odometryLeft;
_odometryRightLast = st.odometryRight;

  _position.x = 0;
  _position.y = 0;
  _position.th = 0;

  // Create topics

  _pubOdometry = _nh.advertise<nav_msgs::Odometry>("odom", 1);
  _pubStatus = _nh.advertise<roswifibot::WStatus>("status", 1);
 // _subSpeeds = _nh.subscribe("cmd_vel", 1, &Wifibot::velocityCallback, this);
  _subSpeeds = _nh.subscribe("cmd_speed", 1, &Wifibot::speedCallback, this);
  // the speed subcriber
  ros::AsyncSpinner threadAsyncSpeed(1);
  threadAsyncSpeed.start();
  ros::Rate sleepThreadSpeed(10);
 
 
  //Reset services
  _resetAngle = _nh.advertiseService("reset_angle", &Wifibot::resetAngleCallback, this);
  _resetOdom = _nh.advertiseService("reset_odom", &Wifibot::resetOdomCallback, this);

  ros::Rate r(10);
  
  _timeCurrent = ros::Time::now();
  _timeLast = ros::Time::now();
  


  while (ros::ok())
    {
 //_pDriver->setSpeeds(1, 1);  // Default Set Speed
	
	//ros::spinOnce();

	update();
	sleepThreadSpeed.sleep();

   }
 
}

Wifibot::~Wifibot()
{
  delete _pDriver;
}

// Reset odometry
bool Wifibot::resetAngleCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  _position.th = 0.0;
  return true;
}
bool Wifibot::resetOdomCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  _position.th = 0.0;
  _position.x = 0.0;
  _position.y = 0.0;
  return true;
}

double Wifibot::getSpeedLinear(double left, double right)
{
  return (right + left) / 2.0;
}

double Wifibot::getSpeedAngular(double left, double right)
{
  return (right - left) / _entrax;
}

void Wifibot::computeOdometry(double left, double right)
{
  double dleft = left - _odometryLeftLast;
  double dright = right - _odometryRightLast;

  double distance = getSpeedLinear(dleft, dright);
  
  _position.th += getSpeedAngular(dleft, dright);
  //_position.th -= (float)((int)(_position.th / TWOPI)) * TWOPI;
  _position.th = atan2(sin(_position.th), cos(_position.th));

  
  _position.x += distance * cos(_position.th);
  _position.y += distance * sin(_position.th);

  _odometryLeftLast = left;
  _odometryRightLast = right;
}

void Wifibot::velocityCallback(const geometry_msgs::TwistConstPtr& vel)
{
if (debug == true){
ROS_INFO("The velocity call back function : %0.3f | %0.3f", vel->linear.x, vel->angular.z);
}
  _speedLeft = vel->linear.x - (vel->angular.z * (_entrax / 2.0));
  _speedRight =  vel->linear.x + (vel->angular.z * (_entrax / 2.0));
  _updated = true;
}

// for the speed control

/* this function is tested and is working */
void Wifibot::speedCallback(roswifibot::speed_msg speedValue)
{
  _speedLeft = speedValue.speedLeft;
  _speedRight = speedValue.speedRight;
  //std::cout << "[speedCallback] _speedLeft" << _speedLeft << std::endl;
  //std::cout << "[speedCallback] _speedRight" << _speedRight << std::endl;
  _updated = true;
}
//////////////////
void Wifibot::update()
{
	 
roswifibot::WStatus topicStatus;
  
  // Send speeds only if needed
  	
if (_updated){
  _pDriver->setSpeeds(_speedLeft, _speedRight);
}
_updated = false;

/***********************************************************************************/
/*[Anis]: I added these three lines because, without them, the read operation 
 * is blocked some how and is not updated
 * we need to check for other solutions*/
	std::string dev = "/dev/ttyS0";
	_pDriver->_serial.close();
	_pDriver->_serial.open(dev);
/***********************************************************************************/

//get data from driver
wifibot::driverData st = _pDriver->readData();
/*	if (debug == true){
		ROS_INFO("----- After Read Data in Update Function ");
	}*/  
  _timeCurrent = ros::Time::now();
// Fill status topic
topicStatus.battery_level = st.voltage;
topicStatus.current = st.current; // see libwifibot to adapt this value
	if (debugTopic == true){
		ROS_INFO("Update Function: The Current is:  %0.3f",st.current);
	}
  topicStatus.ADC1 = st.adc[0];
  topicStatus.ADC2 = st.adc[1];
  topicStatus.ADC3 = st.adc[2];
  topicStatus.ADC4 = st.adc[3];
	
  topicStatus.speed_front_left = st.speedFrontLeft;
  topicStatus.speed_front_right = st.speedFrontRight;
  topicStatus.odometry_left = st.odometryLeft;
  topicStatus.odometry_right = st.odometryRight;
  topicStatus.version = st.version;
  topicStatus.relay1 = 0;
  // publish status
  _pubStatus.publish(topicStatus);
  
  // compute position
  computeOdometry(st.odometryLeft, st.odometryRight);
  
  //TRANSFORM we'll publish the transform over tf
/*if (debugTopic == true){
	ROS_INFO(" ---- Before the TF transform publish  in Update Function");
} */

  _odomTf.header.stamp = _timeCurrent;
  _odomTf.header.frame_id = "/odom";
  _odomTf.child_frame_id = _frameBase;
  
  _odomTf.transform.translation.x = _position.x;
  _odomTf.transform.translation.y = _position.y;
  _odomTf.transform.translation.z = 0.0;
  _odomTf.transform.rotation = 
    tf::createQuaternionMsgFromYaw(_position.th);
  
  //send the transform
  _odomBroadcast.sendTransform(_odomTf);
  
if (debugTopic == true){
	ROS_INFO(" ---- After the TF transform publish  in Update Function");
} 

  //TOPIC, we'll publish the odometry message over ROS
  nav_msgs::Odometry odometryTopic;
  odometryTopic.header.stamp = _timeCurrent;
  odometryTopic.header.frame_id = "/odom";
  
  //set the position
  odometryTopic.pose.pose.position.x = _position.x;
  odometryTopic.pose.pose.position.y = _position.y;
  odometryTopic.pose.pose.position.z = 0.0;
  odometryTopic.pose.pose.orientation = 
    tf::createQuaternionMsgFromYaw(_position.th);
    
  //set the velocity
  odometryTopic.child_frame_id = _frameBase;
  odometryTopic.twist.twist.linear.x = getSpeedLinear(
    st.speedFrontLeft, st.speedFrontRight);
  odometryTopic.twist.twist.linear.y = 0.0;
  odometryTopic.twist.twist.angular.z = getSpeedAngular(
    st.speedFrontLeft, st.speedFrontRight);
  if(debug ==true)
{
// ROS_INFO("lin:%0.3f ang:%0.3f",  odometryTopic.twist.twist.linear.x, odometryTopic.twist.twist.angular.z);

}

  //publish the message
  _pubOdometry.publish(odometryTopic);
  // Update last time
  _timeLast = _timeCurrent;


}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "wifibot_base");
	 int count = 0;
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Starting wifibot.cpp " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

  Wifibot _self(ros::NodeHandle("")); 
  
  return 0;
}
