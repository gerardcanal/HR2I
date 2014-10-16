#include "std_msgs/String.h"
#include <sstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
// for Wifibot

// for speed control
#define SPEED_FORWARD     10.0
#define SPEED_BACKWARD    220
#define NO_OF_ITERATIONS 40
#define SET_TO_ZERO      0

#define SET_MOTOR_IN_THREAD_MODE 1
#define SET_MOTOR_INLINE_MODE 2

#define VN_YAW_OFFSET 9
#define METER_TO_CM 100
#define ODO_ONE_METER 4900
#define DEFAULT_SPEED 10
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
typedef long ODOMETRY;
//Define new type for Bearing
typedef  double BEARING;
//Define new type for speed
typedef unsigned char SPEED;
//using namespace std;
//Define new type for Bearing
typedef  double COORDINATE;


//Converts from distance in meter to odometry
ODOMETRY distance2odometry (DISTANCE distance);

//Converts from odometry to distance
DISTANCE odometry2distance (ODOMETRY odometry);


