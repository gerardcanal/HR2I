#include "std_msgs/String.h"
#include <sstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
// for Wifibot
#include "libwifibot.h"
#include "wifibot.h" //define the interface of the class wifibot
#include "roswifibot/WStatus.h" //define the message type of vn100
// for Vector Nav
#include "libvectornav.h" // it contains link to vn100.h and math.h
#include "vn100.h" //need to access vn100 functions
#include "vncp_services.h"
#include "vectornav.h" //define the interface of the class
#include "roswifibot/vn100_msg.h" //define the message type of vn100
#include "roswifibot/speed_msg.h" //define the message type Speed for passing it to the wifibot
#include "roswifibot/VnQuaternion_msg.h" //define the message type of VectorSpace(Quaternion To get X ,Y, Z, W) 
#include "roswifibot/VnVector3_msg.h" //define the message type of 3 Compnenets used to get acceleration magnetic values through VN


// for subscriber handling
#include <ros/callback_queue.h>
// for async threads
#include <boost/thread.hpp>
// for speed control
#define SPEED_FORWARD     2.0
#define SPEED_FORWARD_MS  0.2

#define SPEED_BACKWARD    220
#define SPEED_BACKWARD_MS  0.2

#define SPEED_MS  0.2

#define NO_OF_ITERATIONS 40
#define SET_TO_ZERO      0

#define SET_MOTOR_IN_THREAD_MODE 1
#define SET_MOTOR_INLINE_MODE 2

#define VN_YAW_OFFSET 9
#define METER_TO_CM 100
#define ODO_ONE_METER 4900
#define DEFAULT_SPEED 2
#define DEFAULT_ROTATE_SPEED 6
#define CLOSED_LOOP_SPEED_FORWARD 192  // 128+1*64
#define CLOSED_LOOP_SPEED_BACKWARD 128 // 128+ 0*64
#define OPEN_LOOP_SPEED_FORWARD 64     // 0+1*64
#define OPEN_LOOP_SPEED_BACKWARD 0     // 0+ 0*64
#define ODO_ONE_DEGREE 17
#define PI 3.14159265


#define ODO_ZERO_DEGREE 0
#define ODO_FORTYFIVE_DEGREE 900
#define ODO_SIXTY_DEGREE 1150
#define ODO_NINTY_DEGREE 2000
#define ODO_ONEEIGHTY_DEGREE 4500


//Define new type for distances
typedef  float DISTANCE;
//Define new type for odometry
typedef double ODOMETRY;
//Define new type for Bearing
typedef  double BEARING;
//Define new type for speed
typedef float SPEED;
//using namespace std;
//Define new type for Bearing
typedef  double COORDINATE;


class wifibotControl{
public:
    // Create an publisher object for publishing the speed
    static ros::Publisher _speedPublisher;
    //Create a message object for the publishing the speed 
    static roswifibot::speed_msg topicSpeed; 
   //Create a message object for the publishing the Wifibot Status 
    static roswifibot::WStatus wifibotGlobalStatus;
   //Create a message object for the publishing the VN Oreintation Status 
    static roswifibot::vn100_msg vnGlobalStatus;
   //Create a message object for the publishing the Vector Space and Quaternion Status 
     static roswifibot::VnQuaternion_msg quaternionStatus;
   //Create a message object for the publishing the VN Angular Status 
     static roswifibot::VnVector3_msg angularRateStatus;
   //Create a message object for the publishing the VN Acceleration Status 
   static roswifibot::VnVector3_msg accelerationStatus;
    //Create a message object for the publishing the Wifibot position 
    static nav_msgs::Odometry wifibotPose;

    //Declare Subscriber for the Robot Status
    static ros::Subscriber subscriberStatus;
    //Declare Subscriber for the Odometry (Wifibot Position)
    static ros::Subscriber subscriberOdom;
    //Declare Subscriber for VectorNav
    static ros::Subscriber subscriberVN100;
    //Declare Subscriber for Quanternion (Vector Space)
    static ros::Subscriber subscriberQuan;
    //Declare Subscriber for Acceleration
    static ros::Subscriber subscriberAccel ;
    //Declare Subscriber for Angular Rate
    static ros::Subscriber subscriberAngRate;
    
    //Callback function for the robot status subscriber
      void statusCallback(roswifibot::WStatus msg);
    //Callback function for the vectornav subscriber
      void vnStatusCallback(roswifibot::vn100_msg msg);
    //Callback function for the odometry subscriber
     void odomCallback(const nav_msgs::Odometry odom);
     //Callback function for the VN Quaternion Subscriber
     void vnQuaternionCallback(roswifibot::VnQuaternion_msg vSpaceMsg);
     //Callback function for the VN acceleration Subscriber
     void vnAccelerationCallback(roswifibot::VnVector3_msg msg);
     //Callback function for the VN Angular Rate Subscriber
     void vnAngularRateCallback(roswifibot::VnVector3_msg angulRateMsg);
     
    //Print the control menu
    static void menu();
    //Print Wifibot Status and Position
    static int printWifibotStatus();
    // Move the robot for a certain distance with a certain speed
    //For backward, put negative value
    //Allowed values -1.2 until 1.2
    static int Move(DISTANCE distance, SPEED speed);

};