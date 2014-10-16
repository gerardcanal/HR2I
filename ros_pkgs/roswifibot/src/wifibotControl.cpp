#include "ros/ros.h"
# include "wifibotControl.h"

using namespace std;

    // define static data members at global namespace scope
    //all static data members must be defined at global scope
    ros::Publisher wifibotControl::_speedPublisher;
    nav_msgs::Odometry wifibotControl::wifibotPose;
    ros::Subscriber wifibotControl::subscriberStatus;
    ros::Subscriber wifibotControl::subscriberOdom;
    ros::Subscriber wifibotControl::subscriberVN100;
    ros::Subscriber wifibotControl::subscriberQuan;
    ros::Subscriber wifibotControl::subscriberAccel ;
    ros::Subscriber wifibotControl::subscriberAngRate;
 
    // Status Report
    roswifibot::VnQuaternion_msg wifibotControl::quaternionStatus;
    roswifibot::VnVector3_msg wifibotControl::angularRateStatus;
    roswifibot::VnVector3_msg wifibotControl::accelerationStatus;
    roswifibot::speed_msg wifibotControl::topicSpeed; 
    roswifibot::WStatus wifibotControl::wifibotGlobalStatus;
    roswifibot::vn100_msg wifibotControl::vnGlobalStatus;
  

/**********************************************************/
// Function statusCallback 
// Input: roswifibot::WStatus
//Output : Printing the Status of Wifibot
/**********************************************************/
void wifibotControl::statusCallback(roswifibot::WStatus msg){
  // Global Status of wifibot is updated
  wifibotGlobalStatus.battery_level  = msg.battery_level;
  wifibotGlobalStatus.current =  msg.current;
  wifibotGlobalStatus.ADC1 = msg.ADC1;
  wifibotGlobalStatus.ADC2 =  msg.ADC2;
  wifibotGlobalStatus.ADC3 = msg.ADC3;
  wifibotGlobalStatus.ADC4 = msg.ADC4;
  wifibotGlobalStatus.speed_front_left =  msg.speed_front_left;
  wifibotGlobalStatus.speed_front_right =  msg.speed_front_right;
  wifibotGlobalStatus.odometry_left =msg.odometry_left;
  wifibotGlobalStatus.odometry_right = msg.odometry_right ;
  wifibotGlobalStatus.version  =  msg.version;
}

/**********************************************************/
// Function vnStatusCallback
// Input: roswifibot::vn100_msg
//Output : Printing the Orientation accroding to Vector Nav
/**********************************************************/
void wifibotControl::vnStatusCallback(roswifibot::vn100_msg msg){
  vnGlobalStatus.yaw = msg.yaw;
  vnGlobalStatus.pitch = msg.pitch;
  vnGlobalStatus.roll = msg.roll;
}

/**********************************************************/
// Function odomCallback
// Input: const nav_msgs::Odometry odom
//Output : getting location X,Y of the robot calculated in wifibot.cpp
/**********************************************************/
void wifibotControl::odomCallback(const nav_msgs::Odometry odom){
  wifibotPose.pose.pose.position.x=odom.pose.pose.position.x;
  wifibotPose.pose.pose.position.y=odom.pose.pose.position.y; 
}
/**********************************************************/
// Function vnQuaternionCallback
// Input: coswifibot::VnQuaternion_msg vSpaceMsg
//Output : getting location X,Y,Z,W of the robot calculated in vn100.c
/**********************************************************/

void wifibotControl::vnQuaternionCallback(roswifibot::VnQuaternion_msg quaternionMsg){
quaternionStatus.q0 = quaternionMsg.q0;
quaternionStatus.q1 = quaternionMsg.q1;
quaternionStatus.q2 = quaternionMsg.q2;
quaternionStatus.q3 = quaternionMsg.q3;
}

void wifibotControl::vnAccelerationCallback(roswifibot::VnVector3_msg msg){
  accelerationStatus.c0 = msg.c0;
  accelerationStatus.c1 = msg.c1;
  accelerationStatus.c2 = msg.c2;
}

void wifibotControl::vnAngularRateCallback(roswifibot::VnVector3_msg angulRateMsg){
  angularRateStatus.c0 = angulRateMsg.c0;
  angularRateStatus.c1 = angulRateMsg.c1;
  angularRateStatus.c2 = angulRateMsg.c2;
}

/**********************************************************/
// Function menu
// Input: 
//Output : Printing the main options of the wifibotControl file
/**********************************************************/
void wifibotControl::menu()
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
int wifibotControl::Move(DISTANCE distance, SPEED speed){
	
	ODOMETRY SDL_0 = wifibotGlobalStatus.odometry_left;
	ODOMETRY SDR_0 = wifibotGlobalStatus.odometry_right;

	//if the speed if positive, then move forward
	if (speed>0.0){
	    while (((wifibotGlobalStatus.odometry_left+wifibotGlobalStatus.odometry_right)/2)<((SDL_0+SDR_0)/2)+(distance)){
	    //keep publish the speed until the robot goes the specified distance
	    topicSpeed.speedLeft = speed;
	    topicSpeed.speedRight =  speed;
	    _speedPublisher.publish(topicSpeed);
	    //ros::spinOnce();
	    }
	} else 
	  if (speed<0.0){
	      while (((wifibotGlobalStatus.odometry_left+wifibotGlobalStatus.odometry_right)/2)>((SDL_0+SDR_0)/2)-(distance)){
	      //setMotorSpeed(CLOSED_LOOP_SPEED_BACKWARD+DEFAULT_SPEED, CLOSED_LOOP_SPEED_BACKWARD+DEFAULT_SPEED);
	      topicSpeed.speedLeft =speed ;
	      topicSpeed.speedRight = speed ;
	      _speedPublisher.publish(topicSpeed);
	      }
	  }
	
	return 1;
}


int wifibotControl::printWifibotStatus(){
  cout <<"*********************** WIFIBOT STATUS ***********************" <<endl;   
  cout << "wifibotGlobalStatus.battery_level:  "<< wifibotGlobalStatus.battery_level<<endl;
    cout <<"wifibotGlobalStatus.current: "<<wifibotGlobalStatus.current <<endl;
    cout <<"wifibotGlobalStatus.ADC1: " << wifibotGlobalStatus.ADC1<<endl;
    cout <<"wifibotGlobalStatus.ADC2: "<< wifibotGlobalStatus.ADC2<<endl;
    cout <<"wifibotGlobalStatus.ADC3: " << wifibotGlobalStatus.ADC3<<endl;
    cout <<"wifibotGlobalStatus.ADC4: " << wifibotGlobalStatus.ADC4<<endl;
    cout <<"wifibotGlobalStatus.speed_front_left: "  << wifibotGlobalStatus.speed_front_left<<endl;
    cout <<"wifibotGlobalStatus.speed_front_right: "  << wifibotGlobalStatus.speed_front_right<<endl;
    cout <<"wifibotGlobalStatus.odometry_left: " << wifibotGlobalStatus.odometry_left<<endl;
    cout <<"wifibotGlobalStatus.odometry_right: "<< wifibotGlobalStatus.odometry_right<<endl;
    cout <<"wifibotGlobalStatus.version: " << wifibotGlobalStatus.version<<endl;
    cout <<"*************************************" <<endl; 
    cout <<"X Coordinate: " <<  wifibotPose.pose.pose.position.x<<endl;
    cout <<"Y Coordinate: " <<  wifibotPose.pose.pose.position.y<<endl;
    cout <<"*************************************" <<endl; 
    cout <<"Yaw Value VN: " <<  vnGlobalStatus.yaw<<endl;
    cout <<"Pitch Value VN: " <<  vnGlobalStatus.pitch<<endl;
    cout <<"Roll Value VN: " <<  vnGlobalStatus.roll<<endl;
    cout <<"*************************************" <<endl; 
    cout <<"Quaternion q0: " << quaternionStatus.q0<<endl;
    cout <<"Quaternion q1: " << quaternionStatus.q1<<endl;
    cout <<"Quaternion q2: " << quaternionStatus.q2<<endl;
    cout <<"Quaternion q3: " << quaternionStatus.q3<<endl;
    cout <<"*************************************" <<endl; 
    cout <<"Acceleration on X: " <<  accelerationStatus.c0<<endl;
    cout <<"Acceleration on Y: " <<  accelerationStatus.c1<<endl;
    cout <<"Acceleration on Z: " <<  accelerationStatus.c2<<endl;
    cout <<"*************************************" <<endl; 
    cout <<"AngularRate on X: " <<  angularRateStatus.c0<<endl;
    cout <<"AngularRate on Y: " <<  angularRateStatus.c1<<endl;
    cout <<"AngularRate on Z: " << angularRateStatus.c2<<endl;
    cout <<"************************************************************" <<endl; 
return 1;	
  
}

////////////////////////The Main function ///////////////////////////////////
int main(int argc, char **argv)
{
 
  char c='z';
  
   ros::init(argc, argv, "listener");
   ros::NodeHandle n;
   ROS_INFO("Starting wifibotControl.cpp!");
  
  wifibotControl wfc;
  //publish the speed
  wifibotControl::_speedPublisher= n.advertise<roswifibot::speed_msg>("cmd_speed",1);
   //subscribe to vn100 topic published by wifibot.cpp
   wifibotControl::subscriberVN100 = n.subscribe("vn_ypr_topic", 2, &wifibotControl::vnStatusCallback, &wfc);
   //subscribe to Angular Rate topic published by wifibot.cpp
   wifibotControl::subscriberAngRate = n.subscribe("vn_angular_rate_topic", 2, &wifibotControl::vnAngularRateCallback, &wfc);
   //subscribe to Acceleration published by wifibot.cpp
   wifibotControl::subscriberAccel = n.subscribe("vn_acceleration_topic", 2, &wifibotControl::vnAccelerationCallback, &wfc);
   //subscribe to vectorSpaceStatus topic published by wifibot.cpp
   wifibotControl::subscriberQuan = n.subscribe("vn_quaternion_topic", 2, &wifibotControl::vnQuaternionCallback, &wfc);
   //subscribe to wifibot status topic published by wifibot.cpp
   wifibotControl::subscriberStatus = n.subscribe("status", 2, &wifibotControl::statusCallback, &wfc);
   //subscribe to wifibot odom (X,Y coordinates) topic published by wifibot.cpp
   wifibotControl::subscriberOdom = n.subscribe("odom", 2, &wifibotControl::odomCallback, &wfc);
  
     
   // AsyncSpinner 
   //imer
      ros::AsyncSpinner statusSpinner(6);
      statusSpinner.start();
      ros::Rate threadRate(10);
   //ros::spin();
   //int i=0;
   
  while (ros::ok()){ // rosok bracket
    //menu();
      DISTANCE dist=0.0;
      SPEED spd = 0.0;
      cout <<"Enter a Character and press Enter to move. Enter q to Quit ..." <<endl;
    
	cin >> c;
	if (c=='q') return 1;
        if(c=='f'){
	    cout<< "Enter the distance to Move";
	    cin >>dist;
	    cout<< "Enter the speed: [0.2 is recommended]" ;
	    cin >>spd;
	    wifibotControl::printWifibotStatus();
	    threadRate.sleep();
	    wifibotControl::Move(dist,spd);
	    threadRate.sleep();
	    wifibotControl::printWifibotStatus();
	}
	if(c=='p'){
	   threadRate.sleep();
	 wifibotControl::printWifibotStatus();
	}
	if(c=='l'){
	  threadRate.sleep();
	 cout <<"X Coordinate: " <<  wifibotControl::wifibotPose.pose.pose.position.x<<endl;
	 cout <<"Y Coordinate: " <<  wifibotControl::wifibotPose.pose.pose.position.y<<endl;
	}
	if(c=='s'){
	  threadRate.sleep();
	 cout <<"VectorSpace CoOrdinate X: " << wifibotControl::quaternionStatus.q0<<endl;
         cout <<"VectorSpace CoOrdinate Y: " << wifibotControl::quaternionStatus.q1<<endl;
         cout <<"VectorSpace CoOrdinate Z: " << wifibotControl::quaternionStatus.q2<<endl;
         cout <<"VectorSpace CoOrdinate W: " << wifibotControl::quaternionStatus.q3<<endl;
        }
        if(c=='a'){
	  threadRate.sleep();
	 cout <<"Acceleration on CoOrdinate X: " << wifibotControl::accelerationStatus.c0<<endl;
         cout <<"Acceleration on CoOrdinate Y: " << wifibotControl::accelerationStatus.c1<<endl;
         cout <<"Acceleration on CoOrdinate Z: " << wifibotControl::accelerationStatus.c2<<endl;
         }
          if(c=='t'){
	  threadRate.sleep();
	 cout <<"AngularRate on CoOrdinate X: " << wifibotControl::angularRateStatus.c0<<endl;
         cout <<"AngularRate on CoOrdinate Y: " << wifibotControl::angularRateStatus.c1<<endl;
         cout <<"AngularRate on CoOrdinate Z: " << wifibotControl::angularRateStatus.c2<<endl;
         }
        if(c=='v'){
	    threadRate.sleep();
	    cout <<"Yaw Value VN: " <<  wifibotControl::vnGlobalStatus.yaw<<endl;
	    cout <<"Pitch Value VN: " <<  wifibotControl::vnGlobalStatus.pitch<<endl;
	    cout <<"Roll Value VN: " <<  wifibotControl::vnGlobalStatus.roll<<endl;
	  }
   
    
  }
     statusSpinner.stop();
  return 0;
}

