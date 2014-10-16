#include <iostream>
#include <iomanip>
#include "WifibotServer.h"
#include "WifibotLib.h"
using namespace std;

/***************** Global Variables *********************************/

Mutex mutexSend;
Mutex mutexRcv;
ODOMETRY PreviousOdometry=0;

/*********************************************************************/

WifibotServer::WifibotServer(void){
	
	
	setSetMotorMode(SET_MOTOR_IN_THREAD_MODE);
	cout<<"Set Motor Mode Done ..."<<endl;
	cout<<"Loading Wifibot Server ..."<<endl;
	cout<<"Connecting to Serial Port ..."<<endl;
	hUSB = SetupRS232CommPort((LPCSTR)PORT_RS232_USB);
	cout<<"Connection Established hUSB = "<< hUSB <<endl;
	cout<<"Starting RS232 Thread ... " <<endl;
	init_RS232_33f();
	Sleep(500);
	cout<<"RS232 Thread Started. " <<endl;
	cout<<"Starting TCP Thread ... " <<endl;
	init_tcp_com();
	Sleep(500);
	cout<<"TCP Thread Started. " <<endl;
	cout<<"Connecting to VN100 Device ..."<<endl;
	Sleep(500);
	cout<<"Starting Surv-Track Thread ... " <<endl;
	SurvTrackTCPPort = 15400;
	init_survtrack_com();
	cout<<"Surv-Track Thread Started. " <<endl;
	Sleep(500);
	isVN100=false;
	isVN100=connectVN100(); 
	if (isVN100)
		cout<<"Connected with Success to to VN100 Device. Current Yaw: "<< getYawVN100() <<endl;
		else
			cout<<"Not connected to VN100 Device. Check VN100 Connection or COM Port Number. Current Yaw: "<< getYawVN100() <<endl;
	
	setMotorSpeed(0,0);
	watchdog = 0;
	cycles = 0;
	cout<<"Initialize Location of the Robot"<<endl;
	Location.setX(0);
	Location.setY(0);
	Location.setZ(0);
	Location.setOrientation(getYawVN100());
}

WifibotServer::~WifibotServer(void){
	bool discon = disconnectVN100();
	setMotorSpeed(0,0);
}

void WifibotServer::setSpeedLeft(SPEED SpL){
	SpeedLeft=SpL;
}

SPEED WifibotServer::getSpeedLeft(){
	return SpeedLeft;
}
	
void WifibotServer::setSpeedRight(SPEED SpR){
	SpeedRight=SpR;
}

SPEED WifibotServer::getSpeedRight(){
	return SpeedRight;
}

void WifibotServer::setSpeed(SPEED SpL, SPEED SpR){
	setSpeedLeft(SpL);
	setSpeedRight(SpR);
}


void WifibotServer::setSetMotorMode(short mode){
	
	SetMotorMode=SET_MOTOR_IN_THREAD_MODE;
	if (mode == SET_MOTOR_INLINE_MODE)
		SetMotorMode= SET_MOTOR_INLINE_MODE;
		
}

short WifibotServer::getSetMotorMode(){
	return SetMotorMode;
}

void WifibotServer::setMotorSpeed(SPEED SpL, SPEED SpR){
	if (getSetMotorMode () == SET_MOTOR_IN_THREAD_MODE){
		setSpeed(SpL, SpR);
	}
	else if (getSetMotorMode () == SET_MOTOR_INLINE_MODE){
		SetMotorRS23233f_low_res(SpL, SpR);
	}
	else {
		cout <<"Problem: setMotorMode not found ..." << endl;
	}
}

void WifibotServer::setMotorSpeed(SPEED SpL, SPEED SpR, short motorMode){
	setSetMotorMode(motorMode);
	setMotorSpeed(SpL, SpR);
}


int WifibotServer::MoveForward(DISTANCE distance){
	ODOMETRY SDL_0 = SDL.Odometry;
	ODOMETRY SDR_0 = SDR.Odometry;
	while (((SDL.Odometry+SDR.Odometry)/2)<((SDL_0+SDR_0)/2)+distance2odometry(distance)){
		setMotorSpeed(CLOSED_LOOP_SPEED_FORWARD+DEFAULT_SPEED, CLOSED_LOOP_SPEED_FORWARD+DEFAULT_SPEED);
	}
	setMotorSpeed(0,0);
	UpdateLocation();
	return (1);
}

int WifibotServer::MoveBackward(DISTANCE distance){
	ODOMETRY SDL_0 = SDL.Odometry;
	ODOMETRY SDR_0 = SDR.Odometry;
	while (((SDL.Odometry+SDR.Odometry)/2)>((SDL_0+SDR_0)/2)-distance2odometry(distance)){
		setMotorSpeed(CLOSED_LOOP_SPEED_BACKWARD+DEFAULT_SPEED, CLOSED_LOOP_SPEED_BACKWARD+DEFAULT_SPEED);
	}
	setMotorSpeed(0,0);
	UpdateLocation();
	return (1);
}

int WifibotServer::MoveForward(DISTANCE distance, SPEED SpL, SPEED SpR){
	ODOMETRY SDL_0 = SDL.Odometry;
	ODOMETRY SDR_0 = SDR.Odometry;
	while (((SDL.Odometry+SDR.Odometry)/2)<((SDL_0+SDR_0)/2)+distance2odometry(distance)){
		setMotorSpeed(CLOSED_LOOP_SPEED_FORWARD+SpL, CLOSED_LOOP_SPEED_FORWARD+SpR);
	}
	setMotorSpeed(0,0);
	UpdateLocation();
	return (1);
}

int WifibotServer::MoveBackward(DISTANCE distance, SPEED SpL, SPEED SpR){
	ODOMETRY SDL_0 = SDL.Odometry;
	ODOMETRY SDR_0 = SDR.Odometry;
	while (((SDL.Odometry+SDR.Odometry)/2)>((SDL_0+SDR_0)/2)-distance2odometry(distance)){
		setMotorSpeed(CLOSED_LOOP_SPEED_BACKWARD+SpL, CLOSED_LOOP_SPEED_BACKWARD+SpR);
	}
	setMotorSpeed(0,0);
	UpdateLocation();
	return (1);
}

int WifibotServer::Rotate(BEARING rotAngle){
	BEARING initialAngle, newangle;
	if (!isVN100){ /* if VN 100 is not connected, do not rotate and print an error message */
			cout << "Cannot rotate. No IMU Found. Check Whether VN 100 is Connected or Not." <<endl;
			return (0);
		}
	else {
	
			
			/*adjust rotation angle to be within the range -180 and 180*/
			rotAngle = adjustBearing (rotAngle);
			
			/*read the current yaw in the range -180 and 180 degrees*/
			initialAngle = getYawVN100();
			printf("Yaw: %+#7.2f | Initial Angle = %7.2f \n", getYawVN100(), initialAngle);			
			Location.setOrientation(initialAngle);

			newangle = initialAngle + rotAngle; 
 	
			if (rotAngle <0)//turn left
			{
				if (newangle < -180) 
						while(Location.getOrientation() > newangle +  VN_YAW_OFFSET)
						{
							//SetMotorRS23233f_low_res(DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_BACKWARD,DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_FORWARD);
							setMotorSpeed(DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_BACKWARD,DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_FORWARD);		
							//adjust the angle to be in range [0 .. 360]
							Location.setOrientation(adjust360nYawFromVN100(getYawVN100()));
							printf("Turn Left 01: Yaw: %+#7.2f | Current Angle = %7.2f \n", getYawVN100(), Location.getOrientation());
						}
					else if (newangle > - 180)
						while(Location.getOrientation() > newangle +  VN_YAW_OFFSET)
						{
							//SetMotorRS23233f_low_res(DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_BACKWARD,DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_FORWARD);
							setMotorSpeed(DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_BACKWARD,DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_FORWARD);
							Sleep(20);
							Location.setOrientation(getYawVN100());
							printf("Turn Left 02: YPR: %+#7.2f | Current Angle = %7.2f \n", getYawVN100(), Location.getOrientation());
						}
				}

			else 
				if (rotAngle >0)//turn right
				{
				
					if (newangle > 180) 
						while(Location.getOrientation() < newangle -  VN_YAW_OFFSET)
						{
							//SetMotorRS23233f_low_res(DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_FORWARD,DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_BACKWARD);			
							setMotorSpeed(DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_FORWARD,DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_BACKWARD);
							//adjust the angle to be in range [0 .. 360]
							Location.setOrientation(adjust360YawFromVN100(getYawVN100()));
							printf("Turn Right 01: Yaw: %+#7.2f | Current Angle = %7.2f \n", getYawVN100(), Location.getOrientation());
						}
					else if (newangle < 180)
						while(Location.getOrientation() < newangle -  VN_YAW_OFFSET)
						{
							//Sleep(1);
							//SetMotorRS23233f_low_res(DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_FORWARD,DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_BACKWARD);
							setMotorSpeed(DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_FORWARD,DEFAULT_ROTATE_SPEED+CLOSED_LOOP_SPEED_BACKWARD);
							//Read new Yaw value
							Sleep(20);
							Location.setOrientation(getYawVN100());
							printf("Turn Right 02: YPR: %+#7.2f | Current Angle = %7.2f \n", getYawVN100(), Location.getOrientation());
						}
				}
		StopMotorRS23233f();
		//Sleep(500);
		printf("Yaw: %+#7.2f | Current Angle = %7.2f \n", getYawVN100(), Location.getOrientation());
		UpdateLocation();
		return (1);			
	}
}



/* Function to move the robot from one waypoint to another */
int WifibotServer::MoveEdge (RobotLocation initialLocation, RobotLocation targetLocation){
	
	DISTANCE distance = 0;
	BEARING nextOrientation = 0;
	BEARING rotationAngle = 0;
	
	if (!isVN100){ /* if VN 100 is not connected, do not rotate and print an error message */
			cout << "[WifibotServer::MoveEdge] Cannot Move Edge. No IMU Found. Check Whether VN 100 is Connected or Not." <<endl;
			return (0);
		}
	else {
	
		distance= calculateDistance(initialLocation.getX(),initialLocation.getY(),targetLocation.getX(),targetLocation.getY());
		nextOrientation= calculateBearing(initialLocation.getX(),initialLocation.getY(),targetLocation.getX(),targetLocation.getY());
		//cout << "distance: " << distance << " next robot orientation : "	<<nextOrientation << endl;	
		rotationAngle = nextOrientation - getYawVN100();
		//cout << "distance: " << distance << "rotation angle: "	<<rotationAngle << endl;
		Rotate(rotationAngle);
		Sleep(400);
		MoveForward(distance);
		Sleep(400);
		return (1);
	}
}


/* Function for waypoint path following  */
void WifibotServer::FollowPath(char * mapfile){

	ifstream inStreamMap; // indata is like cin
	double xr1,yr1;// coordinate of the initial location 
	double xr2,yr2;// coordinate of the destination location 
	
	inStreamMap.open(mapfile); // opens the file
    
	RobotLocation initialLocation; 
	RobotLocation targetLocation;	
	initialLocation.setOrientation(0);
	
	if(!inStreamMap) 
	{      
	 //file couldn't be opened
	  cerr << "Error: file could not be opened. Check file name. Program Exit." << endl;
	  exit(1);
	}
	while ( !inStreamMap.eof() )   // keep reading until end-of-file
		{ 
				inStreamMap >> xr1 >> yr1 >> xr2>> yr2;
				cout << " X0  " << xr1 << " Y0  "<< yr1 <<" X1 "<< xr2 << " Y1 "<< yr2 <<endl;
				
				initialLocation.setX(xr1);
				initialLocation.setY(yr1);
				targetLocation.setX(xr2);
				targetLocation.setY(yr2);
				
				MoveEdge (initialLocation,targetLocation);
				cout << "Step Finished" << endl;
				inStreamMap.ignore(1);	
				Sleep(400);
	}
		

}


void WifibotServer::printRobotStatus ()
	{
	cout << "----------------------------------------------------" <<endl;
	cout << "Wifbot Lab Robot Status Report\n " <<endl;
	cout << "Left Side: " <<endl;
	cout << left << setw(16) <<"Battery Level: "<< left << setw(5) << SDL.BatLevel <<endl;
	cout << left << setw(16) << "IR Sensor Front: "<< left << setw(5) <<SDL.IRFront <<endl;
	//cout << left << setw(16) << "IR Sensor 02: "<< left << setw(5) << SDL.IRBack <<endl;
	cout << left << setw(16) << "Odometry: "<< left << setw(5) << SDL.Odometry <<endl;
	cout << left << setw(16) << "Speed Front: "<< left << setw(5) << SDL.SpeedFront <<endl;
	cout << left << setw(16) << "Current: "<< left << setw(5) << SDL.Current <<endl;
	//cout << left << setw(16) << "Version: "<< left << setw(5) << SDL.Version <<endl;
	cout << left << setw(16) << "------------" <<endl;
	cout << left << setw(16) << "Right Side: " <<endl;
	cout << left << setw(16) << "Battery Level: "<< left << setw(5) << SDR.BatLevel <<endl;
	cout << left << setw(16) << "IR Sensor Front: "<< left << setw(5) << SDR.IRFront <<endl;
	//cout << left << setw(16) << "IR Sensor 02: "<< left << setw(5) << SDR.IRBack <<endl;
	cout << left << setw(16) << "Odometry: "<< left << setw(5) << SDR.Odometry <<endl;
	cout << left << setw(16) << "Speed Front: "<< left << setw(5) << SDR.SpeedFront <<endl;
	cout << left << setw(16) << "Current: "<< left << setw(5) << SDR.Current <<endl;
	cout << left << setw(16) << "Orientation : "<< left << setw(5) << getYawVN100() <<endl;
	cout << left << setw(16) << "Position X: "<< left << setw(5) << Location.getX()<<endl;
	cout << left << setw(16) << "Position Y: "<< left << setw(5) << Location.getY()<<endl;
//	cout << left << setw(16) << "Position X: "<< left << setw(5) << intruderPosition.X <<endl;
//	cout << left << setw(16) << "Position Y: "<< left << setw(5) << intruderPosition.Y <<endl;

	//cout << left << setw(16) << "Version: "<< left << setw(5) << SDR.Version <<endl;
	cout << left << setw(16) << "----------------------------------------------------" <<endl;

	}


void WifibotServer::Navigate(SPEED speed){

	SPEED speed_left = speed;
	SPEED speed_right= speed;
	int Left_IR_AVG; //filtered left IR values
	int Right_IR_AVG; // filtered right IR values
	
	// get initial robot status
	Sleep(50);
	Left_IR_AVG = SDL.IRFront; // init average left IR
	Right_IR_AVG = SDR.IRFront; // init average right IR

	bool navigate = true;
	
	while (navigate){
	
		Left_IR_AVG = (Left_IR_AVG +SDL.IRFront)/2;
		Right_IR_AVG = (Right_IR_AVG +SDR.IRFront)/2;
		speed_left = speed;
		speed_right = speed;
	
	//mutexSend.acquire();
		if (max(Left_IR_AVG,Right_IR_AVG)  <120) { // if close to an obstacle, move backward in closed loop
			speed_left = speed;
			speed_right = speed;
			
			if ((Left_IR_AVG - Right_IR_AVG)>40){
			speed_left = speed / 2;
			}
			if ((Right_IR_AVG - Left_IR_AVG)>40){
			speed_right = speed / 2;
			}
			setMotorSpeed(speed_left,speed_right);
		}else
		if (max(Left_IR_AVG,Right_IR_AVG)  <150) { // if close to an obstacle, move backward in closed loop
			speed_left = speed;
			speed_right = speed;
			
			if ((Left_IR_AVG - Right_IR_AVG)>40){
			speed_left = speed / 4;
			}
			if ((Right_IR_AVG - Left_IR_AVG)>40){
			speed_right = speed / 4;
			}
			setMotorSpeed(speed_left,speed_right);
		}		
		else 
		if (max(Left_IR_AVG,Right_IR_AVG)  <190){ //if obstacle very close
			
			if ((Left_IR_AVG - Right_IR_AVG)>0){
					speed_left = CLOSED_LOOP_SPEED_FORWARD+DEFAULT_SPEED/2;
					speed_right = CLOSED_LOOP_SPEED_BACKWARD+DEFAULT_SPEED;
					
					while (SDL.IRFront > 40){ // move back to the right
						setMotorSpeed(speed_left, speed_right);
						Sleep(10);
					}
			}
			else if ((Right_IR_AVG - Left_IR_AVG)>0){
					speed_left = CLOSED_LOOP_SPEED_BACKWARD+DEFAULT_SPEED;
					speed_right = CLOSED_LOOP_SPEED_FORWARD+(DEFAULT_SPEED/2);
					
					while (SDR.IRFront > 40){
						setMotorSpeed(speed_left, speed_right);
						Sleep(10);
					}
			}
			//navigate = false;
		}
		
		else {
			navigate=false;
			setMotorSpeed(0, 0);
			cout  << "Stopped ..."<< endl;
		}
	//mutexSend.release();
		cout  << "LeftSpeed: "<<  static_cast<int>(SpeedLeft) << endl;
		cout  << "RightSpeed: "<<  static_cast<int>(SpeedRight) << endl;
		cout  << "SDL.IRFront: "<<  static_cast<int>(SDL.IRFront) << endl;
		cout  << "SDR.IRFront: "<<  static_cast<int>(SDR.IRFront) << endl;
		
		cout  << endl;
	Sleep(10);
	}

}


/* Function to update the location of the robot */
void WifibotServer::UpdateLocation(){

	DISTANCE travelledDistance;
	COORDINATE XtempPosition,YtempPosition;	
	//cout <<"Start Location"<<endl;
	travelledDistance =  (((static_cast<DISTANCE>(SDL.Odometry)+static_cast<DISTANCE>(SDR.Odometry))/2)-static_cast<DISTANCE>(PreviousOdometry))/ODO_ONE_METER;
	//cout <<"travelledDistance " <<travelledDistance <<endl;
	/* calculate new locations. we multiply by 100 (i.e. METER_TO_CM) to convert to centimer*/
	XtempPosition = (travelledDistance * cos(degreeToRadian(-Location.getOrientation())))*METER_TO_CM; 
	YtempPosition = (travelledDistance * sin(degreeToRadian(-Location.getOrientation())))*METER_TO_CM;

	if(travelledDistance > 0.3)
	{
		//cout <<"Before Location Update Forward Move " <<Location.getX()<<", " <<Location.getY()<<"      Temp: "<<XtempPosition<<","<< YtempPosition<<endl;
		Location.setX(Location.getX() + XtempPosition);
		Location.setY(Location.getY() + YtempPosition);
		PreviousOdometry = ((SDL.Odometry+SDR.Odometry)/2);
		//cout <<"Location Updated Forward Move  "<<Location.getX()<<", " <<Location.getY()<<"      Temp: "<<XtempPosition<<","<< YtempPosition<<endl;
	}else
	
	if(travelledDistance < 0.3)
	{
		//cout <<"Before Location Update Backward Move " <<Location.getX()<<", " <<Location.getY()<<"      Temp: "<<XtempPosition<<","<< YtempPosition<<endl;
		Location.setX(Location.getX() + XtempPosition);
		Location.setY(Location.getY() + YtempPosition);
		PreviousOdometry = ((SDL.Odometry+SDR.Odometry)/2);
		//cout <<"Location Updated Backward Move " <<Location.getX()<<", " <<Location.getY()<<"      Temp: "<<XtempPosition<<","<< YtempPosition<<endl;
	}
		
}

/* Connecting to the Robot */
HANDLE WifibotServer::SetupRS232CommPort( LPCSTR comport)
{
	HANDLE hCom;
	DCB dcb;
	COMMTIMEOUTS ct;
	//if (!hCom) CloseHandle(hCom);
	hCom = CreateFileA( comport, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	if (hCom<0)  {
		cout<< "Program Exit at WifibotServer::SetupRS232CommPort(). Thread_RS23233f problem ";
		exit(0);
	}
	GetCommState(hCom, &dcb);          dcb.BaudRate = RS232_SPEED;
	dcb.fParity = FALSE;           	   dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;          dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fDsrSensitivity = FALSE;       dcb.fOutX = FALSE;
	dcb.fInX = FALSE;	               dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fAbortOnError = FALSE;         dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;  	       dcb.StopBits = 0;
	
	SetCommState(hCom, &dcb);          GetCommTimeouts(hCom, &ct);
	ct.ReadIntervalTimeout = 500;      ct.ReadTotalTimeoutMultiplier =500; 
	ct.ReadTotalTimeoutConstant = 500; SetCommTimeouts(hCom, &ct);
	SetCommMask(hCom, EV_RXCHAR);
	
	return hCom;
}

int WifibotServer::GetMotorRS23233f()
{	    
	DWORD n;
	BYTE sbuf[30];//unsigned char for LINUX
	int res= 0;

	do {
		ReadFile(hUSB, &sbuf, 1, &n, NULL);
	}while(sbuf[0]!=255);

	res = ReadFile(hUSB, &sbuf, 21, &n, NULL);

	short mycrcrcv = (short)((sbuf[20] << 8) + sbuf[19]);
	short mycrcsend = Crc16(sbuf,19);

	if (mycrcrcv!=mycrcsend)
	{
		do {
			ReadFile(hUSB, &sbuf, 1, &n, NULL);
		}while(sbuf[0]!=255);
	}
	else {
		SDL.setSpeedFront((int)((sbuf[1] << 8) + sbuf[0]));
		if (SDL.getSpeedFront() > 32767) SDL.setSpeedFront(SDL.getSpeedFront()-65536);
		SDL.setBatLevel(sbuf[2]);
		SDL.setIRFront(sbuf[3]);
		SDL.setIRBack(sbuf[4]);
		SDL.setOdometry(((((long)sbuf[8] << 24))+(((long)sbuf[7] << 16))+(((long)sbuf[6] << 8))+((long)sbuf[5])));

		SDR.SpeedFront=(int)(sbuf[10] << 8) + sbuf[9];
		if (SDR.SpeedFront > 32767) SDR.SpeedFront=SDR.SpeedFront-65536;
		SDR.BatLevel=0;
		SDR.IRFront=sbuf[11];
		SDR.IRBack=sbuf[12];
		SDR.Odometry=((((long)sbuf[16] << 24))+(((long)sbuf[15] << 16))+(((long)sbuf[14] << 8))+((long)sbuf[13]));
		SDL.Current=sbuf[17];
		SDR.Current=sbuf[17];
		SDL.Version=sbuf[18];
		SDR.Version=sbuf[18];
	}
	return res;
}


int WifibotServer::SetMotorRS23233f_low_res(BYTE speed1,BYTE speed2)
{
	DWORD n;
	BYTE sbuf[30];
	BYTE tt=0;
	sbuf[0] = 255;
	sbuf[1] = 0x07;

	int tmp1 = 8*(speed1&0x3F);
	int tmp2 = 8*(speed2&0x3F);
	if (speed2&0x80) tt=tt+32;
	if (speed2&0x40) tt=tt+16;
	sbuf[2] = (BYTE)tmp1;
	sbuf[3] = (BYTE)(tmp1 >> 8);

	sbuf[4] = (BYTE)tmp2;
	sbuf[5] = (BYTE)(tmp2 >> 8);

	sbuf[6] = (speed1&0x80) + (speed1&0x40) + tt +0+0+8;//+1 Relay ON +8 10ms pid mode ;

	short mycrcsend = Crc16(sbuf+1,6);

	sbuf[7] = (BYTE)mycrcsend;
	sbuf[8] = (BYTE)(mycrcsend >> 8);

	int res = WriteFile(hUSB, &sbuf, 9,&n, NULL);
	//cout <<"res low" <<res <<endl;
	return res;
}

int WifibotServer::SetMotorRS23233f(short speed1,short speed2,BYTE SpeedFlag)
{
	DWORD n;
	BYTE sbuf[10];
	BYTE tt=0;
	sbuf[0] = 255;
	sbuf[1] = 0x07;
	cout <<"SpeedFlag " <<SpeedFlag <<endl;
	if (speed2&0x80) tt=tt+32;
	if (speed2&0x40) tt=tt+16;
	
	sbuf[9] = (speed1&0x80) + (speed1&0x40) + tt +0+0+8;
	printf("buf 9 = %d \n", sbuf[9]);
	
	sbuf[2] = (BYTE)speed1;
	sbuf[3] = (BYTE)(speed1 >> 8);
	sbuf[4] = (BYTE)speed2;
	sbuf[5] = (BYTE)(speed2 >> 8);
	
	sbuf[6] = SpeedFlag;
	
	short mycrcsend = Crc16(sbuf+1,6);
	sbuf[7] = (BYTE)mycrcsend;
	sbuf[8] = (BYTE)(mycrcsend >> 8);
	
	int res = WriteFile(hUSB, &sbuf, 9,&n, NULL);
	return res;
}

int WifibotServer::SetMotorPIDRS23233f(BYTE speed1,BYTE speed2,BYTE pp,BYTE ii,BYTE dd,short maxspeed)
{
	DWORD n;
	BYTE sbuf[30];

	sbuf[0] = 255;
	sbuf[1] = 0x09;
	sbuf[2] = speed1;
	sbuf[3] = speed2;
	sbuf[4] = pp;
	sbuf[5] = ii;
	sbuf[6] = dd;
	sbuf[7] = (BYTE)maxspeed;
	sbuf[8] = (BYTE)(maxspeed >> 8);
	short mycrcsend = Crc16(sbuf+1,8);
	sbuf[9] = (BYTE)mycrcsend;
	sbuf[10] = (BYTE)(mycrcsend >> 8);
	WriteFile(hUSB, &sbuf, 11, &n, NULL);
	return sbuf[0];
}

int WifibotServer::StopMotorRS23233f()
{
	SetMotorRS23233f_low_res(0x00,0x00);
	SpeedLeft=0;
	SpeedRight=0;
	return 0;
}
/********************************************************* THREADS **************************************************/
void WifibotServer::Thread_TCP(void)
{
	WSADATA wsa;
	SOCKET sock;
	SOCKADDR_IN sin;
	int sinsize;
	int recvMsgSize=0;

	//HANDLE hUSB = (HANDLE*)lpParam;
	//Server initialization
	WSAStartup(MAKEWORD(2,2),&wsa);
	//Create Socket
	sock=socket(PF_INET,SOCK_STREAM,0);
	sin.sin_family=AF_INET;
	//Starting Socket Binding
	sin.sin_port=htons((unsigned short)15020);
	sin.sin_addr.s_addr=INADDR_ANY;
	bind(sock,(SOCKADDR*)&sin,sizeof(sin));
	sinsize=sizeof(sin);
	printf("Thread_TCP started ... \n");
	while(1)
	{
		listen(sock,1);
		SOCKET hAccept = accept(sock, NULL, NULL);
		printf("TCP Client has connected to the robot ... \n");
		Sleep(500);
		do{
			if ((recvMsgSize = recv(hAccept, (char*)buffso_rcv, 2, 0)) < 1) shutdown(hAccept,1);
			//mutexSend.acquire();
			SpeedLeft=buffso_rcv[0];
			SpeedRight=buffso_rcv[1];
			//mutexSend.release();
			watchdog = 0;
			cycles = 3;
			//mutexRcv.acquire();
			send(hAccept,buffso_send,15,0);
			//mutexRcv.acquire();
		}while(recvMsgSize>0);
	}
}


DWORD WINAPI WifibotServer::Thread_TCP(LPVOID lpParam)
{
   WifibotServer *me = (WifibotServer *)lpParam;
   me->Thread_TCP();
   return 0;
}


void WifibotServer::Thread_Dog(void)
{
	cout << "Thread_Dog started ..." <<endl;
	while (1) {
		if(watchdog < cycles) watchdog = watchdog + 1;
		else if(watchdog == cycles)
		{
			SpeedLeft = 0;
			SpeedRight = 0;
		}//end of else if watchdog
		Sleep(400);
	}
	//return 0;
}


DWORD WINAPI WifibotServer::Thread_Dog(LPVOID lpParam)
{
   WifibotServer *me = (WifibotServer *)lpParam;
   me->Thread_Dog();
   return 0;
}


void WifibotServer::Thread_RS23233f(void)
{
	
	//DWORD startTime, endTime;
	//DWORD elapsedTime;
	//LARGE_INTEGER freq;
	//LARGE_INTEGER startTime1, endTime1;
	//LARGE_INTEGER elapsedTime1, elapsedMilliseconds;
	
	SpeedLeft=0;SpeedRight=0;
	SetMotorRS23233f_low_res(0x00,0x00);
	Sleep(200);
	SetMotorPIDRS23233f(0x00,0x00,80,45,0,360);
	SetMotorPIDRS23233f(0x00,0x00,80,45,0,360);
	Sleep(200);
	int foo=0;
	int res=0;
	cout << "Thread_RS23233f started ..." <<endl;
	while(1)
	{
#ifdef debug_msg 
		printf("speed1 %d speed2 %d\n",speed1,speed2);
#endif
		//QueryPerformanceFrequency(&freq);
		//QueryPerformanceCounter(&startTime1);
		//startTime = timeGetTime();
		mutexSend.acquire();
		/*the line below must be disabled if you want to control the robot speed from itself*/
		/*the line below must be enable when you want to control the robot speed from a client that will send speed1 and speed2*/
		
		if (SetMotorMode == SET_MOTOR_IN_THREAD_MODE)
			res = SetMotorRS23233f_low_res(SpeedLeft,SpeedRight);
		
		res = GetMotorRS23233f();
		
		mutexSend.release();
		//We prepare the data to be sent to the user
		// run some code that takes many milliseconds
		//endTime = timeGetTime();
		//elapsedTime = endTime - startTime;

		//QueryPerformanceCounter(&endTime1);
		//elapsedTime1.QuadPart = endTime1.QuadPart - startTime1.QuadPart;
		//elapsedMilliseconds.QuadPart = (1000 * elapsedTime1.QuadPart) / freq.QuadPart;

		mutexRcv.acquire();	 

		buffso_send[0]=(char)(SDL.BatLevel);//GetADC(hUSB,0x48); not speed but batery level
		buffso_send[1]=(char)(SDL.SpeedFront/5);
		buffso_send[2]=(char)(SDL.Current);
		buffso_send[3]=(char)(SDR.SpeedFront/5);
		buffso_send[4]=(char)(SDL.Current);
		buffso_send[5]=(char)SDL.IRFront;
		buffso_send[6]=(char)SDR.IRFront;
		buffso_send[7]=(char)SDL.Odometry;
		buffso_send[8]=(char)(SDL.Odometry >> 8);
		buffso_send[9]=(char)(SDL.Odometry >> 16);
		buffso_send[10]=(char)(SDL.Odometry >> 24);
		buffso_send[11]=(char)SDR.Odometry;
		buffso_send[12]=(char)(SDR.Odometry >> 8);
		buffso_send[13]=(char)(SDR.Odometry >> 16);
		buffso_send[14]=(char)(SDR.Odometry >> 24);
#ifdef debug_msg 
		printf("codeurL= %ld codeurR= %ld speed1 %d odo %ld %ld bat = %d IR_R = %d IR_L = %d time to set and get I2C = %d %d res %d\n",buffso_sendOUT[1],buffso_sendOUT[2],speed1,SDR.odometry,SDL.odometry,SDL.BatLevel,SDR.IRFront,SDL.IRFront,elapsedTime,elapsedMilliseconds.QuadPart,res);
		printf("speedL %d R%d\n",SDL.SpeedFront,SDR.SpeedFront);
#endif
		mutexRcv.release();	
	}
}


DWORD WINAPI WifibotServer::Thread_RS23233f(LPVOID lpParam)
{
   WifibotServer *me = (WifibotServer *)lpParam;
   me->Thread_RS23233f();
   return 0;
}


/********************************************************************************************************/
// Function robotNavigationTCPServer
// Input: 
//Output : send the robot current status to the  surv-track client application on port 15400 each 200 ms
/********************************************************************************************************/

void WifibotServer::Thread_SurvTrackTCPServer(void){

	cout << "Starting up Surv-Track TCP Server\r\n";

	WSADATA wsa;
	SOCKET sock;
	SOCKADDR_IN socketinput;
	int sinsize;
	int recvMsgSize=0;
	char tcpSendBuffer [21];
	char tcpReceiveBuffer [1];
	
	
	/* start socket  */
	
	WSAStartup(MAKEWORD(2,2),&wsa);
    sock=socket(PF_INET,SOCK_STREAM,0);
	socketinput.sin_family=AF_INET;	
	socketinput.sin_port=htons((unsigned short) SurvTrackTCPPort);	
	socketinput.sin_addr.s_addr=INADDR_ANY;	
	bind(sock,(SOCKADDR*)&socketinput,sizeof(socketinput));	
	sinsize=sizeof(socketinput);
	cout<<"SurvTrackTCPServerThread: TCP connection started - Port"<<SurvTrackTCPPort <<". Waiting connections..."<<endl;

	/* start listening socket*/
	while(1)
	{
		listen(sock,1);
		SOCKET hAccept = accept(sock, NULL, NULL);
		printf("\n A TCP Client has connected to the SurvTrack TCP Server ... \n");
			
		do{
			if ((recvMsgSize = recv(hAccept, (char*)tcpReceiveBuffer, 2, 0)) < 1) shutdown(hAccept,1);			
			//Sleep(200);
			//update the location of the robot based on odometry (traveled distance) and vectornav (orientation)
			UpdateLocation();
			
			//fill the sending buffer with robot information
			tcpSendBuffer[0]=(char)(SDL.BatLevel);//GetADC(hUSB,0x48); not speed but batery level
			tcpSendBuffer[1]=(char)(SDL.SpeedFront/5);
			tcpSendBuffer[2]=(char)(SDL.Current);
			tcpSendBuffer[3]=(char)(SDR.SpeedFront/5);
			tcpSendBuffer[4]=(char)(SDL.Current);
			tcpSendBuffer[5]=(char)SDL.IRFront;
			tcpSendBuffer[6]=(char)SDR.IRFront;
			tcpSendBuffer[7]=(char)SDL.Odometry;
			tcpSendBuffer[8]=(char)(SDL.Odometry >> 8);
			tcpSendBuffer[9]=(char)(SDL.Odometry >> 16);
			tcpSendBuffer[10]=(char)(SDL.Odometry >> 24);
			tcpSendBuffer[11]=(char)SDR.Odometry;
			tcpSendBuffer[12]=(char)(SDR.Odometry >> 8);
			tcpSendBuffer[13]=(char)(SDR.Odometry >> 16);
			tcpSendBuffer[14]=(char)(SDR.Odometry >> 24);
			//fill in orientation buffer
			tcpSendBuffer[15]=(char)(adjust360YawFromVN100(Location.getOrientation()));
			tcpSendBuffer[16]=(char)(int(adjust360YawFromVN100(Location.getOrientation()))>> 8);
			//fill in sending buffer with location X and Y.
			tcpSendBuffer[17]=(char)(Location.getX());
			tcpSendBuffer[18]=(char)(int(Location.getX())>>8);
			tcpSendBuffer[19]=(char)(Location.getY());
			tcpSendBuffer[20]=(char)(int(Location.getY())>>8);
			//send the buffer to the client.
			send(hAccept,tcpSendBuffer,21,0);
		}while(recvMsgSize>0);
	}
	
}


DWORD WINAPI WifibotServer::Thread_SurvTrackTCPServer(LPVOID lpParam) {
   WifibotServer *me = (WifibotServer *)lpParam;
   me->Thread_SurvTrackTCPServer();
   return 0;
	
}


int WifibotServer::init_survtrack_com(){

	int uExitCode = 1;
	Handle_Of_Thread_SurvTrack = CreateThread( NULL, 0, Thread_SurvTrackTCPServer, this, 0, NULL);  
	if (Handle_Of_Thread_SurvTrack == NULL)  {
		cout<< "Program Exit at WifibotServer::init_survtrack_com(). Handle_Of_Thread_SurvTrack problem ";
		ExitProcess(uExitCode);
		}
	return 1;

}

int WifibotServer::init_tcp_com()
{
	int uExitCode = 1;
	Handle_Of_Thread_TCP = CreateThread( NULL, 0, Thread_TCP, this, 0, NULL);  
	if ( Handle_Of_Thread_TCP == NULL){
		cout<< "Program Exit at WifibotServer::init_tcp_com(). Thread_TCP problem ";
		ExitProcess(uExitCode);
		}

	//Handle_Of_Thread_Dog = CreateThread( NULL, 0, Thread_Dog, this, 0, NULL);  
	//if ( Handle_Of_Thread_Dog == NULL) {
	//	cout<< "Program Exit at WifibotServer::init_tcp_com(). Thread_Dog problem ";
	//	ExitProcess(uExitCode);
	//	}
	return 1;	
}

int WifibotServer::init_RS232_33f()
{
	int uExitCode = 1;
	Handle_Of_Thread_RS23233f = CreateThread( NULL, 0, Thread_RS23233f, this, 0, NULL);  
	if (Handle_Of_Thread_RS23233f == NULL)  {
		cout<< "Program Exit at WifibotServer::init_RS232_33f(). Thread_RS23233f problem ";
		ExitProcess(uExitCode);
		}
	return 1;	
}


short WifibotServer::Crc16(unsigned char *Adresse_tab , unsigned char Taille_max)
{
	unsigned int Crc = 0xFFFF;
	unsigned int Polynome = 0xA001;
	unsigned int CptOctet = 0;
	unsigned int CptBit = 0;
	unsigned int Parity= 0;

	Crc = 0xFFFF;
	Polynome = 0xA001;
	for ( CptOctet= 0 ; CptOctet < Taille_max ; CptOctet++)
	{
		Crc ^= *( Adresse_tab + CptOctet);

		for ( CptBit = 0; CptBit <= 7 ; CptBit++)
		{
			Parity= Crc;
			Crc >>= 1;
			if (Parity%2 == true) Crc ^= Polynome;
		}
	}
	return(Crc);
}

/* members functions and data members for VectorNav IMU VN100 manipulations*/
/*****************************************************/
/* Function: connectVN100()                          */
/* Input   : nothing                                 */
/* Output  : returns zero if connection is successful*/
/*****************************************************/
bool WifibotServer::connectVN100(){
	VN_ERROR_CODE EC = vn100_connect(&hVN100, COM_PORT_VN, BAUD_RATE);
	//cout <<"        EC = "<< EC << endl;
	if (EC == VNERR_NO_ERROR) return true;
	else return false;
}

/*****************************************************/
/* Function: getYawVN100()                           */
/* Input   : nothing                                 */
/* Output  : returns ya value from vectornav IMU     */
/*****************************************************/
double WifibotServer::getYawVN100(){
	VnYpr ypr;
	if (isVN100) {
			vn100_getAttitudeYawPitchRoll(&hVN100, &ypr);
			return ypr.yaw;
		}
		else return 0.0;
}

/*****************************************************/
/* Function: disconnectVN100()                          */
/* Input   : nothing                                 */
/* Output  : returns zero if connection is successful*/
/*****************************************************/

bool WifibotServer::disconnectVN100(){
	if (isVN100) {
			VN_ERROR_CODE EC = vn100_disconnect(&hVN100);
			if (EC == VNERR_NO_ERROR) return true;
				else return false;
		}
		else return false;
}
