#include "libvectornav.h" // it contains link to vn100.h and math.h
#include "vn100.h" //need to access vn100 functions
#include "vncp_services.h"
#include "vectornav.h" //define the interface of the class
#include "ros/ros.h"
#include "roswifibot/vn100_msg.h" //define the message type of vn100
#include "roswifibot/VnQuaternion_msg.h" //define the message type of VectorSpace(Quaternion To get X ,Y, Z, W) 
#include "roswifibot/VnVector3_msg.h" //define the message type of 3 Compnenets used to get acceleration magnetic values through VN


using namespace std;
VectorNav::VectorNav(const ros::NodeHandle& nh)
  : _VNNodeHandle (nh)
  , _yaw(0.0)
  , _pitch(0.0)
  , _roll(0.0)
  , _q0(0.0)
  , _q1(0.0)
  , _q2(0.0)
  , _q3(0.0)
  ,_c0(0.0)
  ,_c1(0.0)
  ,_c2(0.0)
  {
	VnQuaternion quaternion;// for Quaternion. They named quaternion components with x,y,z and w in the structured defined in vn_kinmatics.h
        VnYpr ypr; // for Yaw pith role
	VnVector3 acceleration; // for accelration 
	VnVector3 angularRate; // for getting the angular rate
	
	//char bufferCmd[256];
	//char strShellName[256];
         //findCOMPort ();
        const char* const COM_PORT = "//dev//ttyUSB0";
        const int BAUD_RATE = 115200;
        Vn100 vn100;
	
        // open VN100 serial port
        int err_val = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
        // if the port does not open, use the following command is terminal
        // sudo chmod 777 /dev/ttyUSB0
        // assuming that VN100 is connected to /dev/ttyUSB0 
	if  (err_val !=0){
	printf("[Error: VectorNav Constructor] Unable to open COM Port for VN-100. \n Possible Solutions:\n");
	printf("    1- Change permission of Serial Port of VN 100. \n       Try command [sudo chmod 777 /dev/ttyUSB0] if /dev/ttyUSB0 is the COM port for VN-100\n");
	printf("     2- Check the connection of VN-100 to Serial Port. \n");
	printf("     3- Check that the port number is VN-100 to Serial Port. \n");	   
	}
	else 	
        printf("[VectorNav Connection Success] COM_PORT = %s \n", COM_PORT);
       /*if  (err_val ==0)
	 for (i = 0; i < 2; i++) {
		vn100_getYawPitchRoll(&vn100, &ypr);
		printf("Test YPR: %+#7.2f %+#7.2f %+#7.2f\n", ypr.yaw, ypr.pitch, ypr.roll);
		sleep(1);
	 }
	 // printing the vector space throught VectorNav
	 for (i = 0; i < 2; i++) {
		  cout << " Pritning the Vector Space (Quaternion)  VectorNav  "<<endl;
		  vn100_getQuaternion(&vn100, &vectorSpace);
		printf("Vector Space: %+#7.2f  %+#7.2f %+#7.2f  %+#7.2f\n", vectorSpace.x, vectorSpace.y, vectorSpace.z,vectorSpace.w);
		sleep(1);
	 }
	 
	  for (i = 0; i < 2; i++) {
		  cout << " Pritning the Acceleration  VectorNav  "<<endl;
		   vn100_getAcceleration(&vn100, &accelerationVN);
		printf("Acceleration: %+#7.2f  %+#7.2f %+#7.2f \n", accelerationVN.c0, accelerationVN.c1, accelerationVN.c2);
		sleep(1);
	 }*/

       /******** Delcare VN-100 publishers ****************/
       
       //Tell the roscore master that we are going to be publishing a message of type roswifibot/vn100_msg on the topic vn_ypr_topic.
	_vectornavYPRPublisher = _VNNodeHandle.advertise<roswifibot::vn100_msg>("vn_ypr_topic", 2);
	//create a message to be published for yaw, pitch roll
	roswifibot::vn100_msg vn100_topic_object;

       //Tell the roscore master that we are going to be publishing a message of type roswifibot::VnQuaternion_msg on the topic vn_vspace_topic.
	_vectornavQuaternionPublisher = _VNNodeHandle.advertise<roswifibot::VnQuaternion_msg>("vn_quaternion_topic", 2);
	//create a message to be published for quaternion
	roswifibot::VnQuaternion_msg vn100_topic_quaternion_object;

	//Tell the roscore master that we are going to be publishing a message of type roswifibot::VnVector3_msg on the topic vn_acceleration_topic.
	_vectornavAccelerationPublisher = _VNNodeHandle.advertise<roswifibot::VnVector3_msg>("vn_acceleration_topic", 2);
	//create a message to be published for acceleration
	roswifibot::VnVector3_msg vn100_accelration_topic_object;

	//Tell the roscore master that we are going to be publishing a message of type roswifibot::VnVector3_msg on the topic vn_acceleration_topic.
	_vectornavAngularRatePublisher = _VNNodeHandle.advertise<roswifibot::VnVector3_msg>("vn_angular_rate_topic", 2);
	//create a message to be published for angular rate
	roswifibot::VnVector3_msg vn100_angular_rate_topic_object;

	/******** Start and loop publishing ****************/
	
	while (ros::ok()){
	
	  //vn100_getCalibratedImuMeasurements(&vn100, VnVector3* magnetic, &accelerationVN, VnVector3* angularRate, double* temperature);

	// Publsishing the Quaternion VectorSpace
	vn100_getQuaternion(&vn100, &quaternion);
	vn100_topic_quaternion_object.q0 = quaternion.x; //q0 defined as x in vn_kinmatics.h of vectornav library
	vn100_topic_quaternion_object.q1 = quaternion.y; //q1 defined as y in vn_kinmatics.h of vectornav library
	vn100_topic_quaternion_object.q2 = quaternion.z; //q2 defined as y in vn_kinmatics.h of vectornav library
	vn100_topic_quaternion_object.q3 = quaternion.w; //q3 defined as y in vn_kinmatics.h of vectornav library
	// publishing the topic Quaternion
      _vectornavQuaternionPublisher.publish(vn100_topic_quaternion_object);

	  vn100_getYawPitchRoll(&vn100, &ypr);      
	  vn100_topic_object.yaw = ypr.yaw; //getting the yaw value
	  vn100_topic_object.pitch = ypr.pitch; //getting the pitch value
	  vn100_topic_object.roll = ypr.roll; //getting the roll value
  	_vectornavYPRPublisher.publish(vn100_topic_object);
  
	// publishing the topic YPR accelration that  shows Acceleration Measurements register.
	   vn100_getAcceleration(&vn100, &acceleration);
	   vn100_accelration_topic_object.c0=acceleration.c0;
	   vn100_accelration_topic_object.c0=acceleration.c0;
	   vn100_accelration_topic_object.c0=acceleration.c0;
	   //publising the acceleration topic
	   _vectornavAccelerationPublisher.publish(vn100_accelration_topic_object);

	   // publishing the angular rate
	   vn100_getAngularRate(&vn100, &angularRate);
	   vn100_angular_rate_topic_object.c0=angularRate.c0;
	   vn100_angular_rate_topic_object.c1=angularRate.c1;
	   vn100_angular_rate_topic_object.c2=angularRate.c2;
	  
	   //publising the acceleration topic
	   _vectornavAngularRatePublisher.publish(vn100_angular_rate_topic_object);

	  
	   
	   
	}
	//vn100_disconnect(&vn100);


}

VectorNav::~VectorNav()
{
  ROS_INFO("[VectorNav::~VectorNav()] Destructor Called! ");
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "vectornav_base");
  ROS_INFO("Welcome to VectorNav ROS Driver");
  VectorNav _self(ros::NodeHandle("")); 
  
  return 0;
}
