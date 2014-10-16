/*****************************************************************************************************/
/*libRosWifibotControl.cpp : Library needed for conversion operations anf Global Definitons             */
/*                                                                                                   */
/* "Copyright (c) 2013-2014 COINS Research Group, Riyadh, Saudi Arabia".                             */
/* All rights reserved.                                                                              */
/*                                                                                                   */
/* Authors:	Yasir Javed <kayaniyasir@yahoo.com>, Anis Koubaa <aska@isep.ipp.pt>,                  */
/* Date:        February 02 2013                                                                     */
/* Desc:        Library needed for conversion operations and defining Variables, HANLDES and Macros  */
/*                                                                                                   */
/*****************************************************************************************************/

# include "libRosWifibotControl.h"

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