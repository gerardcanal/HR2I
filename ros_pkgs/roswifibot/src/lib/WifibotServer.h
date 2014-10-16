#ifndef WIFIBOT
#define WIFIBOT

#include <stdio.h>
#include <math.h>
#include "lib/RobotLocation.h"
#include "lib/WifibotLib.h"
#include "../include/vectornav.h"
#include <strsafe.h>


class WifibotServer {

public:
	
	WifibotServer(void);
	~WifibotServer(void);
	
	/*member functions for speed control */
	void setSpeedLeft(SPEED SpL);
	SPEED getSpeedLeft();
	void setSpeedRight(SPEED SpR);
	SPEED getSpeedRight();
	void setSpeed(SPEED SpL, SPEED SpR);
	
	
	
	/*member functions for robot navigation */
	//Move the robot for a specified distance (in meter)
	int MoveForward(DISTANCE distance);
	int MoveBackward(DISTANCE distance);
	int MoveForward(DISTANCE distance, SPEED SpL, SPEED SpR);
	int MoveBackward(DISTANCE distance, SPEED SpL, SPEED SpR);
	
	int Rotate (BEARING rotAngle);
	
	/* Function to move the robot from one point to another */
	int MoveEdge (RobotLocation , RobotLocation);
	
	/* Function for autonmous robot navigation */
	void Navigate(SPEED speed);
	
	/* Function for waypoint path following  */
	void FollowPath (char * mapfile);
	
	/* members functions to print robot status */
	void printRobotStatus ();
	
	/* Function to update the location of the robot */
	void UpdateLocation();
	
		
	SensorData SDL; // Left Sensor Data
	SensorData SDR; // Right Sensor Data
	HANDLE hUSB;  // Handle to Robot Serial Port
	Vn100 hVN100; //Handle to VN 100 VectorNav Device
	RobotLocation Location;
	unsigned char SpeedLeft; // Left Motor Speed
	unsigned char SpeedRight; // Right Motor Speed

	
private:


	

	

	

	
	
	/* members functions and data members for VectorNav IMU VN100 manipulations*/
	bool isVN100; /*check whether VN100 is connected and used or not*/
	
/*********************************************************/
/* Function: connectVN100()                              */
/* Input   : nothing                                     */
/* Output  : returns zero if disconnection is successful */
/*********************************************************/

bool connectVN100(); /* returns zero if connection is successful*/

/*****************************************************/
/* Function: disconnectVN100()                          */
/* Input   : nothing                                 */
/* Output  : returns zero if connection is successful*/
/*****************************************************/

bool disconnectVN100();

/*****************************************************/
/* Function: getYawVN100()                           */
/* Input   : nothing                                 */
/* Output  : returns ya value from vectornav IMU     */
/*****************************************************/

double getYawVN100();

};
#endif

	
