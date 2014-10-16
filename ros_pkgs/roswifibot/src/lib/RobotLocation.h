#ifndef ROBOTLOCATION_H
#define ROBOTLOCATION_H

#include "WifibotLib.h"

class RobotLocation
{
public:
	RobotLocation(void);
	~RobotLocation(void);
	
	void setX(COORDINATE RX);
	COORDINATE getX();
	
	void setY(COORDINATE RY);
	COORDINATE getY();
	
	void setZ(COORDINATE RZ);
	COORDINATE getZ();
	
	void setOrientation(BEARING Angle);
	BEARING getOrientation();
	
	void setLocation(COORDINATE RX, COORDINATE RY, BEARING Angle);
	RobotLocation getCoordinate();
	
private:
	COORDINATE X;
	COORDINATE Y;
	COORDINATE Z;
	BEARING Orientation;
};

#endif
