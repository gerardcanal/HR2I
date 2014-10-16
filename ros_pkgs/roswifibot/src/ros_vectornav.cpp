#include "vectornav.h" // it contains link to vn100.h and math.h
#include "vn100.h" //need to access vn100 functions
#include "vncp_services.h"
#include "ros_vectornav.h" //define the interface of the class
#include "ros/ros.h"
#include "roswifibot/vn100_msg.h" //define the message type of vn100


VectorNav::VectorNav(const ros::NodeHandle& nh)
  : _VNNodeHandle (nh)
  , _yaw(0.0)
  , _pitch(0.0)
  , _roll(0.0)
{

        VnYpr ypr;
	int i;
	char bufferCmd[256];
	char strShellName[256];
         //findCOMPort ();
        const char* const COM_PORT = "//dev//ttyUSB0";
        const int BAUD_RATE = 115200;
        Vn100 vn100;
	
        // open VN100 serial port
        int err_val = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
        // if the port does not open, use the following command is terminal
        // sudo chmod 777 /dev/ttyUSB0
        // assuming that VN100 is connected to /dev/ttyUSB0 
        //printf("COM_PORT = %s err_vale = %d\n", COM_PORT, err_val);
       if  (err_val ==0)
	 for (i = 0; i < 2; i++) {
		vn100_getYawPitchRoll(&vn100, &ypr);
		printf("Test YPR: %+#7.2f %+#7.2f %+#7.2f\n", ypr.yaw, ypr.pitch, ypr.roll);
		sleep(1);
	 }
	 if  (err_val !=0){
	printf("[Error: VectorNav Constructor] Unable to open COM Port for VN-100. \n Possible Solutions:\n");
	printf("    1- Change permission of Serial Port of VN 100. \n       Try command [sudo chmod 777 /dev/ttyUSB0] if /dev/ttyUSB0 is the COM port for VN-100\n");
	printf("     2- Check the connection of VN-100 to Serial Port. \n");
	printf("     3- Check that the port number is VN-100 to Serial Port. \n");
	 //system
	   
	}
       //Tell the roscore master that we are going to be publishing a message of type roswifibot/vn100_msg on the topic vn_ypr_topic.
_vectornavDataPublisher = _VNNodeHandle.advertise<roswifibot::vn100_msg>("vn_ypr_topic", 10);
roswifibot::vn100_msg vn100_topic_object;
	while (ros::ok()){

	vn100_getYawPitchRoll(&vn100, &ypr);      	
	vn100_topic_object.yaw = ypr.yaw; //getting the yaw value
	vn100_topic_object.pitch = ypr.pitch; //getting the pitch value
	vn100_topic_object.roll = ypr.roll; //getting the roll value
 
 	//printf("Test YPR: %+#7.2f %+#7.2f %+#7.2f\n", ypr.yaw, ypr.pitch, ypr.roll);

 _vectornavDataPublisher.publish(vn100_topic_object);
sleep (1);
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
