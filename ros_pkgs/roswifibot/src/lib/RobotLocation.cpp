#include "RobotLocation.h"

RobotLocation::RobotLocation(void)
{
	setLocation(0, 0, 0);
}

RobotLocation::~RobotLocation(void)
{
}


void RobotLocation::setX(COORDINATE RX){
	X=RX;
}
COORDINATE RobotLocation::getX(){
return X;
}

void RobotLocation::setY(COORDINATE RY){
	Y=RY;
}
COORDINATE RobotLocation::getY(){
return Y;
}

void RobotLocation::setZ(COORDINATE RZ){
	Z=RZ;
}
COORDINATE RobotLocation::getZ(){
return Z;
}

void RobotLocation::setOrientation(BEARING Angle){
	Orientation=Angle;
}
BEARING RobotLocation::getOrientation(){
return Orientation;
}

void RobotLocation::setLocation(COORDINATE RX, COORDINATE RY, BEARING Angle){
	setX(RX);
	setY(RY);
	setZ(0);
	setOrientation(Angle);
}