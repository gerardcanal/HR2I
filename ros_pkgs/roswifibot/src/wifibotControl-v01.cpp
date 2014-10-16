#include "ros/ros.h"
# include "wifibotControl.h"


/**********************************************************/
// Function odomCallback 
// Input: nav_msgs
//Output : Printing the Postion accroding to Odometry
/**********************************************************/
class StatusValues
{
  public:
    }; // class ends here
void odomCallback(const nav_msgs::Odometry odom)
{
  /*ROS_INFO("I am getting the values");
  ROS_INFO("I heard Postion X: [%f]",  odom.pose.pose.position.x);
  ROS_INFO("I heard Postion Y: [%f]",  odom.pose.pose.position.y);
*/
  
}

/**********************************************************/
// Function statusCallback 
// Input: roswifibot::WStatus
//Output : Printing the Status of Wifibot
/**********************************************************/
void statusCallback(roswifibot::WStatus msg)
{
/*
ROS_INFO("\n\n-----------------------------------\n");
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
*/
/*
  // Global Status of wifibot is updated
  wifibotGlobalStatus.battery_level  = msg.battery_level;
  wifibotGlobalStatus.current =  msg.current;
  wifibotGlobalStatus.ADC1 = msg.ADC1;
  wifibotGlobalStatus.ADC2 =  msg.ADC2;
  wifibotGlobalStatus.ADC3 = msg.ADC3;
  wifibotGlobalStatus.ADC4 = msg.ADC4;
  wifibotGlobalStatus.speed_front_left =  msg.speed_front_left;
  wifibotGlobalStatus.odometry_left =msg.odometry_left;
  wifibotGlobalStatus.odometry_right = msg.odometry_right ;
  wifibotGlobalStatus.version  =  msg.version;
*/
//assign msg values to global variables

OdometryLeft = msg.odometry_left;
OdometryRight = msg.odometry_right;
//ROS_INFO("OdometryLeft",  OdometryLeft );

}

/**********************************************************/
// Function vnStatusCallback
// Input: roswifibot::vn100_msg
//Output : Printing the Orientation accroding to Vector Nav
/**********************************************************/
void vnStatusCallback(roswifibot::vn100_msg msg)
{
/*printf("\n\n-----------------------------------\n");
ROS_INFO("I am getting the Status values");
ROS_INFO("I heard Yaw: [%0.3f]",  msg.yaw );
ROS_INFO("I heard Pitch: [%0.3f]",  msg.pitch );
ROS_INFO("I heard Roll: [%0.3f]",  msg.roll );
*/
  
}

////////////////////////The Main function ///////////////////////////////////
int main(int argc, char **argv)
{
  StatusValues statVal; // object of class StatusValues
  char c='z';
  
   ros::init(argc, argv, "listener");
   ros::NodeHandle n;
   ROS_INFO("I called the subscriber");
  //ros::Subscriber sub = n.subscribe("odom", 10, odomCallback);
   // --------------------------------The Publisher of Speed
   _speedPublisher= n.advertise<roswifibot::speed_msg>("cmd_speed",1);
   subscriberVN100 = n.subscribe("vn_ypr_topic", 2, vnStatusCallback);
   subscriberStatus = n.subscribe("status", 2, statusCallback);
  
   // the subscriber to odometry
   //ros::Subscriber sub = n.subscribe("status", 10, statusCallback);
     
     // AsyncSpinner timer
      ros::AsyncSpinner status(2);
      status.start();
      ros::Rate threadRate(10);
   //ros::spin();
   int i=0;
while (ros::ok()){ // rosok bracket
  menu();
   c=getchar();
  //If character is m then move the robot
	if (c=='f') {
	//make the robot move forward
	MoveForward(1);
	} // while bracket
	
	if (c=='b') {
	//make the robot move backward
	MoveBackward(1);
	}
	
	if (c=='m') {
	ROS_INFO("Autonomous Move :");
	}
	if (c=='o') {
	ROS_INFO("Printing odometry Status :");
	}
	if (c=='v') {
	ROS_INFO("Printing Vector Nav Status :");
	
	threadRate.sleep();
	//ros::spinOnce();
	}
	if (c=='w') {
	ROS_INFO("printing wifibot status :");
	std::cout <<"[WifibotControl::PRINT W] OdometryLeft="<<OdometryLeft<<std::endl;
	std::cout <<"WifibotControl::[PRINT W] OdometryRight="<<OdometryRight<<std::endl;
	//ros::spinOnce();
	threadRate.sleep();
	//sleep(1);
	//ros::spinOnce();
	//ros::Duration(0.5).sleep();
	}
	if (c=='p') {
	ROS_INFO("printing all values :");
	//ros::Subscriber sub = n.subscribe("status", 10, statusCallback);
	//print global variables
	}
	   
  // ros::spin();
}
  return 0;
}
/**********************************************************/
// Function menu
// Input: 
//Output : Printing the main options of the wifibotControl file
/**********************************************************/
void menu()
{
	printf( "----------------------------------------------------\n");
	printf( "Welcome to ROS Wifibot Control Program\n \n");
	printf( "Choose an action from the menu\n \n");
	printf( "'p','P': Robot and VN Status\n");
	printf( "'o','O': Odometry Status\n");
	printf( "'v','V': VectorNav Status\n");
	printf( "'w','W': Wifibot Status\n");
	printf( "'f','F': Move Forward\n");
	printf( "'m','M': Map Navigating\n");
	printf( "'r','R': Rotate Robot\n");
	printf( "'q','Q': Quit\n");
	printf( "----------------------------------------------------\n");
}
/**********************************************************/
// Function distance2Odometry 
// Input: distance
//Output : Coverting the meters to odometry values
/**********************************************************/
ODOMETRY distance2odometry (DISTANCE distance){
    return static_cast <ODOMETRY> (distance*ODO_ONE_METER)+1;
}
/**********************************************************/
// Function odometry2distance 
// Input: odomotery
//Output : Coverting the odometry to meters
/**********************************************************/
DISTANCE odometry2distance (ODOMETRY odometry){
  return static_cast <DISTANCE> (odometry)/ODO_ONE_METER;
} 
/**********************************************************/
// Function Move Forward
// Input: distance
//Output : Move the robot forward for specified distance
/**********************************************************/
int MoveForward(DISTANCE distance){
	ODOMETRY SDL_0 = OdometryLeft;
	ODOMETRY SDR_0 = OdometryRight;
	while (((OdometryLeft+OdometryRight)/2)<((SDL_0+SDR_0)/2)+distance2odometry(distance)){
	//setMotorSpeed(CLOSED_LOOP_SPEED_FORWARD+DEFAULT_SPEED, CLOSED_LOOP_SPEED_FORWARD+DEFAULT_SPEED);
	//publish the speed to the topic "cmd_speed"
	topicSpeed.speedLeft =SPEED_FORWARD ;
	topicSpeed.speedRight = SPEED_FORWARD ;
	_speedPublisher.publish(topicSpeed);
	//ros::spinOnce();
	}
	topicSpeed.speedLeft =0.0 ;
	topicSpeed.speedRight = 0.0 ;
	_speedPublisher.publish(topicSpeed);
	//setMotorSpeed(0,0);
	//UpdateLocation();
	return (1);
}
/**********************************************************/
// Function Move Forward
// Input: distance
//Output : Move the robot forward for specified distance
/**********************************************************/

int MoveBackward(DISTANCE distance){
	ODOMETRY SDL_0 = OdometryLeft;
	ODOMETRY SDR_0 = OdometryRight;
	while (((OdometryLeft+OdometryRight)/2)>((SDL_0+SDR_0)/2)-distance2odometry(distance)){
	//setMotorSpeed(CLOSED_LOOP_SPEED_BACKWARD+DEFAULT_SPEED, CLOSED_LOOP_SPEED_BACKWARD+DEFAULT_SPEED);
	topicSpeed.speedLeft =SPEED_BACKWARD ;
	topicSpeed.speedRight = SPEED_BACKWARD ;
	_speedPublisher.publish(topicSpeed);
	}
	//setMotorSpeed(0,0);
	//UpdateLocation();
	return (1);
}
