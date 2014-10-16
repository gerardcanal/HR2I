#ifndef WIFIBOTLIB_H
#define WIFIBOTLIB_H


#include <math.h>



#define BUF_SIZE 255
#define PORT_RS232_USB "COM1"
#define RS232_SPEED CBR_19200

//Define new type for distances
typedef  float DISTANCE;
//Define new type for odometry
typedef double ODOMETRY;
//Define new type for Bearing
typedef  double BEARING;
//Define new type for speed
typedef float SPEED;
//Define new type for Bearing
typedef  double COORDINATE;


#define VN_YAW_OFFSET 9
#define METER_TO_CM 100
#define ODO_ONE_METER 4900
#define DEFAULT_SPEED 0.2 //in meter/sec
#define DEFAULT_ROTATE_SPEED 6
#define PI 3.14159265


using namespace std;


BEARING adjustBearing (BEARING);

BEARING adjust360YawFromVN100 (BEARING);
BEARING adjust360nYawFromVN100 (BEARING);
BEARING adjust180YawFromVN100 (BEARING);
BEARING degreeToRadian(BEARING);

//Converts from distance in meter to odometry
ODOMETRY distance2odometry (DISTANCE distance);

//Converts from odometry to distance
DISTANCE odometry2distance (ODOMETRY odometry);

//float adjust180YawFromVN100 (float);

//functions for calcuation of distance and Bearing
DISTANCE calculateDistance(COORDINATE x1,COORDINATE y1,COORDINATE x2,COORDINATE y2);
BEARING calculateBearing(BEARING x1,BEARING y1,BEARING x2,BEARING y2);

#endif;
