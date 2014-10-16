#include <stdio.h>
//#include <ros/ros.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <iostream>
#include <iomanip>

# include "libwifibot.h"


using namespace std;
int printStatus(wifibot::driverData st);
int main(int argc,char **argv)
{
//ros::init(argc,argv,"testwifibot_node");
//ros::NodeHandle n;
wifibot::Driver *pDriver;
string connectionPort="/dev/ttyS0";
int baudrate = 9600;
int b;
  //ROS_INFO("Creation of variables");
   
// Creating the driver
pDriver = new wifibot::Driver(connectionPort);
// getting the data
cout << "Start Reading Data. Handle: " << pDriver->_serial._handle<<endl;

wifibot::driverData st = pDriver->readData();
cout << "----------------------------------------------------" <<endl;
cout << "Wifbot Lab Robot Status Report\n " <<endl;
//ROS_INFO("The Odometry was %0.3f",st.odometryLeft);
cout << left << setw(16) <<"Current Level: "<< left << setw(5) << st.current <<endl;
cout << left << setw(16) <<"Voltage Level: "<< left << setw(5) << st.voltage <<endl;
cout << left << setw(16) << "Odometry Left: "<< left << setw(5) << st.odometryLeft <<endl;
cout << left << setw(16) << "Odometry Right: "<< left << setw(5) << st.odometryRight <<endl;
cout << left << setw(16) << "Speed Left: "<< left << setw(5) << st.speedFrontLeft <<endl;
cout << left << setw(16) << "Speed Right: "<< left << setw(5) << st.speedFrontRight <<endl;
cout << left << setw(16) << "Version: "<< left << setw(5) << st.version <<endl;
cout << left << setw(16) << "IR Right: "<< left << setw(5) << st.adc[0] <<endl;
cout << left << setw(16) << "IR Left: "<< left << setw(5) << st.adc[2] <<endl;
cout << "----------------------------------------------------" <<endl;


return 1;
}
int printStatus (wifibot::driverData st)
{
wifibot::driverData sta = st;
return 1;
}

