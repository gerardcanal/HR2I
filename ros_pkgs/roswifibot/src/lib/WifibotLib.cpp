/*****************************************************************************************************/
/* Conversion.cpp : Library needed for conversion operations in Surv-Track                           */
/*                                                                                                   */
/* "Copyright (c) 2011-2012 Al-Imam Mohamed bin Saud Islamic University, Riyadh, Saudi Arabia".      */
/* All rights reserved.                                                                              */
/*                                                                                                   */
/* Authors:	Yasir Kiyani <kayaniyasir@rtrackp.com>, Anis Koubaa <aska@isep.ipp.pt>,                  */
/* Date:        December 25 2011                                                                     */
/* Desc:        Library needed for conversion operations in Surv-Track                               */
/*                                                                                                   */
/*****************************************************************************************************/

#include "stdafx.h"
#include "WifibotLib.h"

using namespace std; //needed in linux to access math functions such as sqrt 



BEARING adjustBearing (BEARING angle){
	if (angle >= 180) 
		angle = angle - 360;
	else 
		if (angle < -180) 
		angle = angle + 360;

	return angle;

}

BEARING adjust360YawFromVN100 (BEARING angle){
		if (angle < 0) 
			angle = angle + 360;
	return angle;

}

BEARING adjust360nYawFromVN100 (BEARING angle){
		if (angle > 0) 
			angle = angle - 360;
	return angle;

}

BEARING adjust180YawFromVN100 (BEARING angle){
	if (angle >= 180) 
		angle = angle - 360;
	else 
		if (angle < -180) 
		angle = angle + 360;

	return angle;

}
BEARING degreeToRadian (BEARING deg ) {
  
  return deg * (PIv / 180.0);
}
 
 
 //convert from distance in meter to odometry
ODOMETRY distance2odometry (DISTANCE distance){
    return static_cast <ODOMETRY> (distance*ODO_ONE_METER)+1;
}

//convert from odometry in meter to distance
DISTANCE odometry2distance (ODOMETRY odometry){
  return static_cast <DISTANCE> (odometry)/ODO_ONE_METER;
} 

/**********************************************************/
// Function calculateDistance 
// Input: Coordinates of 2 points
//Output : Calculate the distance between 2 points
/**********************************************************/
DISTANCE calculateDistance(COORDINATE x1,COORDINATE y1,COORDINATE x2,COORDINATE y2)
{
	return sqrt (((x2 - x1) * (x2 - x1))+((y2 - y1) * (y2 - y1)));
}

/**********************************************************/
// Function calculateBearing
// Input: Coordinates of 2 points
//Output : Calculate the degree (bearing) between 2 points
// it represents the next orientation fo the robot if it is 
// in point X1,Y1 and will move to point X2,Y2
/**********************************************************/
BEARING calculateBearing(BEARING x1,BEARING y1,BEARING x2,BEARING y2)
{
	BEARING bearing = atan2((y2 - y1),(x2 - x1));
	//if(bearing < 0) bearing += 2 * 3.14159;
	bearing *= 180.0 / 3.14159;
	return (-bearing); //positive bearing will make a left turn, and negative bearing will make a right turn
}