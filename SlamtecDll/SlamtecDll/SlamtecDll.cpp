// SlamtecDll.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include "SlamtecDll.h"
#include "bitmap_image.hpp"		//Slamtec bitmap functions
#include <iostream>				//for cout
#include <fstream>				//for cout to a file

using namespace rpos::core;
using namespace rpos::actions;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

//
//TODO: If this DLL is dynamically linked against the MFC DLLs,
//		any functions exported from this DLL which call into
//		MFC must have the AFX_MANAGE_STATE macro added at the
//		very beginning of the function.
//
//		For example:
//
//		extern "C" BOOL PASCAL EXPORT ExportedFunction()
//		{
//			AFX_MANAGE_STATE(AfxGetStaticModuleState());
//			// normal function body here
//		}
//
//		It is very important that this macro appear in each
//		function, prior to any calls into MFC.  This means that
//		it must appear as the first statement within the 
//		function, even before any object variable declarations
//		as their constructors may generate calls into the MFC
//		DLL.
//
//		Please see MFC Technical Notes 33 and 58 for additional
//		details.
//

// CSlamtecDllApp

extern "C" __declspec(dllexport) int connectSlamtec(const char* ipAddr, int port, char* errStr, size_t errStrLen);
extern "C" __declspec(dllexport) void disconnect();
extern "C" __declspec(dllexport) void forward();
extern "C" __declspec(dllexport) void left();
extern "C" __declspec(dllexport) void right();
extern "C" __declspec(dllexport) void back();
extern "C" __declspec(dllexport) void moveToFloat(float xMove, float yMove);
extern "C" __declspec(dllexport) void moveToFloatWithYaw(float xMove, float yMove, float yaw);
extern "C" __declspec(dllexport) void moveToInteger(int xMove, int yMove);
extern "C" __declspec(dllexport) void rotateWithOpt(float angle, MoveOptionsStruct moStruct);
extern "C" __declspec(dllexport) void rotate(float angle);
extern "C" __declspec(dllexport) void rotateToWithOpt(float angle, MoveOptionsStruct moStruct);
extern "C" __declspec(dllexport) void rotateTo(float angle);
extern "C" __declspec(dllexport) void wakeup();
extern "C" __declspec(dllexport) void cancelMoveAction();
extern "C" __declspec(dllexport) int getMoveActionStatus();
extern "C" __declspec(dllexport) const char* getMoveActionError();
extern "C" __declspec(dllexport) int waitUntilMoveActionDone();

extern "C" __declspec(dllexport) int battery();
extern "C" __declspec(dllexport) bool getBatteryIsCharging();
extern "C" __declspec(dllexport) int getBoardTemperature();
extern "C" __declspec(dllexport) int getLocalizationQuality();
extern "C" __declspec(dllexport) float odometry();
extern "C" __declspec(dllexport) ExportPose pose();
extern "C" __declspec(dllexport) float heading();
extern "C" __declspec(dllexport) void home();
extern "C" __declspec(dllexport) int getSpeed();
extern "C" __declspec(dllexport) int setSpeed(int speed);
extern "C" __declspec(dllexport) LaserPointsStruct getLaserScan(int *size);
extern "C" __declspec(dllexport) int clearSlamtecMap();
extern "C" __declspec(dllexport) int loadSlamtecMap(const char* str_mapName);
extern "C" __declspec(dllexport) int saveSlamtecMap(const char* str_mapName);
extern "C" __declspec(dllexport) int recoverLocalization(float left, float bottom, float width, float height);
extern "C" __declspec(dllexport) int setUpdate(int enable);
extern "C" __declspec(dllexport) void freeIt(void *ptr);
extern "C" __declspec(dllexport) SensorValueStruct getSensorValue(int id);

BEGIN_MESSAGE_MAP(CSlamtecDllApp, CWinApp)
END_MESSAGE_MAP()

// CSlamtecDllApp construction

CSlamtecDllApp::CSlamtecDllApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}

// The one and only CSlamtecDllApp object

CSlamtecDllApp theApp;

// CSlamtecDllApp initialization

BOOL CSlamtecDllApp::InitInstance()
{
	CWinApp::InitInstance();

	return TRUE;
}

//####################################################################
//####################################################################
// connects to Slamtec
//	Input: none
//	Output:	int =>	0 = ok, 1 = timeout, 2 = fail
extern "C" __declspec(dllexport) int connectSlamtec(const char* ipAddr, int port, char* errStr, size_t errStrLen)
{
	try{
		sdp = SlamwareCorePlatform::connect(ipAddr, port);		//connect
	}
	catch(ConnectionTimeOutException& e)				//if connection fails
	{
		strncpy_s(errStr, errStrLen, e.what(), _TRUNCATE);
		return 1;
	}
	catch(ConnectionFailException& e)					//if connection fails
	{
		strncpy_s(errStr, errStrLen, e.what(), _TRUNCATE);
		return 2;
	}

	return 0;
}

//---------------------------------------------------------
// disconnects Slamtec
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void disconnect()
{
	sdp.disconnect();
}

//-----------------------------------------------------
// move forward
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void forward()
{
	ACTION_DIRECTION actionDirection = FORWARD;
	Direction direction(actionDirection);
	MoveAction moveBy = sdp.moveBy(direction);
}

//-----------------------------------------------------
// move left
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void left()
{
	ACTION_DIRECTION actionDirection = TURNLEFT;
	Direction direction(actionDirection);
	MoveAction moveBy = sdp.moveBy(direction);
}

//-----------------------------------------------------
// move right
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void right()
{
	ACTION_DIRECTION actionDirection = TURNRIGHT;
	Direction direction(actionDirection);
	MoveAction moveBy = sdp.moveBy(direction);
}

//-----------------------------------------------------
// move backward
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void back()
{
	ACTION_DIRECTION actionDirection = BACKWARD;
	Direction direction(actionDirection);
	MoveAction moveBy = sdp.moveBy(direction);
}


//-----------------------------------------------------
// move to x,y using float with yaw
//	Input: float x and y
//	Output:	none
extern "C" __declspec(dllexport) void moveToFloatWithYaw(float xMove, float yMove, float yaw)
{
	Location loc(xMove, yMove);		//get location
	MoveOptions options;
	options.flag = MoveOptionFlagWithYaw;
	MoveAction moveTo = sdp.moveTo(loc, options, yaw);		//move there and end at specified yaw
}

//-----------------------------------------------------
// move to x,y using float
//	Input: float x and y
//	Output:	none
extern "C" __declspec(dllexport) void moveToFloat(float xMove, float yMove)
{
	Location loc(xMove, yMove);		//get loacation
	MoveAction moveTo = sdp.moveTo(loc, false, true);		//move there
}

//-----------------------------------------------------
// move to x,y using float using integer
//	Input: string x and y in integer where value = floating pt * 1000
//	Output:	none
extern "C" __declspec(dllexport) void moveToInteger(int xMove, int yMove)
{
	Location loc(xMove / 1000.0, yMove / 1000.0);		//get loacation
	MoveAction moveTo = sdp.moveTo(loc, false, true);		//move there
}


//----------------------------------------------------------------------
// rotateToWithOpt
//	Input: float angle in radians, rotates to heading specified, MoveOptions
//	Output:	none
extern "C" __declspec(dllexport) void rotateToWithOpt(float angle, MoveOptionsStruct moStruct)
{
	MoveOptions mo;
	mo.flag = static_cast<MoveOptionFlag>(moStruct.flag);
	mo.speed_ratio = moStruct.speed_ratio;
	sdp.rotateTo(angle, mo);
}
//----------------------------------------------------------------------
// rotateTo
//	Input: float angle in radians, rotates to heading specified
//	Output:	none
extern "C" __declspec(dllexport) void rotateTo(float angle)
{
	sdp.rotateTo(angle);
}

//----------------------------------------------------------------------
// rotateWithOpt
//	Input: float angle in radians, positive valus goes left, negative value goes right, MoveOptions
//	Output:	none
extern "C" __declspec(dllexport) void rotateWithOpt(float angle, MoveOptionsStruct moStruct)
{
	MoveOptions mo;
	mo.flag = static_cast<MoveOptionFlag>(moStruct.flag);
	mo.speed_ratio = moStruct.speed_ratio;
	sdp.rotate(angle, mo);
}

//----------------------------------------------------------------------
// rotate
//	Input: float angle in radians, positive valus goes left, negative value goes right
//	Output:	none
extern "C" __declspec(dllexport) void rotate(float angle)
{
	sdp.rotate(angle);
}

//----------------------------------------------------------------------
// wakeup
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void wakeup()
{
	sdp.wakeUp();
}

//----------------------------------------------------------------------
// cancelCurrentAction
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void cancelMoveAction()
{
	 MoveAction moveAction = sdp.getCurrentAction();
	 moveAction.cancel();
}

//----------------------------------------------------------------------
// getMoveActionStatus
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) int getMoveActionStatus()
{
	MoveAction moveAction = sdp.getCurrentAction();
	return moveAction.getStatus();
}

//----------------------------------------------------------------------
// getMoveActionError
//	Input: none
//	Output:	const char* - error string
extern "C" __declspec(dllexport) const char* getMoveActionError()
{
	MoveAction moveAction = sdp.getCurrentAction();
	return _strdup(moveAction.getReason().c_str());
}

//----------------------------------------------------------------------
// waitUntilMoveActionDone
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) int waitUntilMoveActionDone()
{
	MoveAction moveAction = sdp.getCurrentAction();
	return moveAction.waitUntilDone();
}

//----------------------------------------------------------
// get battery charging status 
//	Input: none
//	Output:	bool => true if charging, false if not
extern "C" __declspec(dllexport) bool getBatteryIsCharging()
{
	return sdp.getBatteryIsCharging();	//returns bool
}

//----------------------------------------------------------
// get battery percent
//	Input: none
//	Output:	int => battery percent
extern "C" __declspec(dllexport) int battery()
{
	return sdp.getBatteryPercentage();	//returns int
}

//----------------------------------------------------------
// get board temperature
//	Input: none
//	Output:	int => board temperature
extern "C" __declspec(dllexport) int getBoardTemperature()
{
	return sdp.getBoardTemperature();	//returns int
}

//----------------------------------------------------------
// get localization quality
//	Input: none
//	Output:	int => localization quality
extern "C" __declspec(dllexport) int getLocalizationQuality()
{
	return sdp.getLocalizationQuality();	//returns int
}

//----------------------------------------------------------
// get odometry
//	Input: none
//	Output:	float => odometry
extern "C" __declspec(dllexport) float odometry()
{
	return static_cast<float>(sdp.getOdometry());
}



//----------------------------------------------------------
// get pose
//	Input: none
//	Output:	ExportPose => pose
extern "C" __declspec(dllexport) ExportPose pose()
{
	ExportPose retPose;
	Pose pose = sdp.getPose();
	retPose.x = static_cast<float>(pose.x());
	retPose.y = static_cast<float>(pose.y());
	retPose.yaw = static_cast<float>(pose.yaw() * 180.0f / M_PI);

	//str_temp.Format("Pose: x=%f, y=%f, yaw=%f, heading=%f\r\n", pose.x(), pose.y(), pose.yaw(), pose.yaw()*180.0f/M_PI);
	return retPose;
}

//----------------------------------------------------------
// get heading
//	Input: none
//	Output:	float => heading
extern "C" __declspec(dllexport) float heading()
{
	Pose pose = sdp.getPose();
	return static_cast<float>(pose.yaw() * 180.0f / M_PI);
}

//----------------------------------------------------------
// go home
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void home()
{
	sdp.goHome();
}

//----------------------------------------------------------
// get speed
//	Input: none
//	Output:	int => 0 = invalid, 1 = low, 2 = med, 3 = high
extern "C" __declspec(dllexport) int getSpeed()
{
	string std_s = sdp.getSystemParameter(SYSPARAM_ROBOT_SPEED);
	if(std_s == SYSVAL_ROBOT_SPEED_LOW)
		return 1;
	if(std_s == SYSVAL_ROBOT_SPEED_MEDIUM)
		return 2;
	if(std_s == SYSVAL_ROBOT_SPEED_HIGH)
		return 3;
	return 0;
}

//----------------------------------------------------------
// set speed
//	Input: speed 1 = low, 2 = med, 3 = high
//	Output:	int => 0 = invalid speed, 1 = low set, 2 = med set, 3 = hi set
extern "C" __declspec(dllexport) int setSpeed(int speed)
{
	if(speed == 1)
	{
		bool bRet3 = sdp.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_LOW);
		return 1;
	}
	if(speed == 2)
	{
		bool bRet3 = sdp.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_MEDIUM);
		return 2;
	}
	if(speed == 3)
	{
		bool bRet3 = sdp.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_HIGH);
		return 3;
	}
	return  0;
}

//----------------------------------------------------------
// get laser scan
//
//	Input: none
//	Output: "C://SDP//cout.txt"
extern "C" __declspec(dllexport) LaserPointsStruct getLaserScan(int* size)
{
	LaserPointsStruct lps;
	rpos::features::system_resource::LaserScan laser_scan = sdp.getLaserScan();
	std::vector<LaserPoint> laser_points = laser_scan.getLaserPoints();
	int i = 0;
	for(std::vector<LaserPoint>::iterator it = laser_points.begin(); i < MAX_NUM_LASER_POINTS && it != laser_points.end(); ++it)
	{
		if (it->valid())
		{
			lps.angle[i] = it->angle();
			lps.distance[i] = it->distance();
			++i;
		}
	}
	
	lps.size = min(i, MAX_NUM_LASER_POINTS);
	return lps;
}

//----------------------------------------------------------
// clear Slamtec map
// clear map
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) int clearSlamtecMap()
{
	return !sdp.clearMap();
}

//----------------------------------------------------------
// load Slamtec map
// open map file and upload to SDP
//	Input: filename
//	Output:	int => 0 = success, 1 = err
extern "C" __declspec(dllexport) int loadSlamtecMap(const char* str_mapName)
{
	//get map file
	Pose pose = sdp.getPose();
	CompositeMapReader composite_map_reader;
	std::string error_message;
	boost::shared_ptr<CompositeMap> composite_map(composite_map_reader.loadFile(error_message, str_mapName));
	if (composite_map)
	{
		sdp.setCompositeMap((*composite_map), pose);	
	}
    return !composite_map;
}

//----------------------------------------------------------
// save Slamtec map
// save map file
//	Input: filename
//	Output:	int => 0 = success, 1 = err
extern "C" __declspec(dllexport) int saveSlamtecMap(const char* str_mapName)
{
	// Saving CompositeMap to Local File...
	CompositeMap composite_map = sdp.getCompositeMap();
	CompositeMapWriter composite_map_writer;
	std::string error_message;
	bool result = composite_map_writer.saveFile(error_message, str_mapName, composite_map);	
    return !result;
}

//-----------------------------------------------------------
// Recover localization of the robot given a hint rectangle of the area it is thought to be in
//  Input: float left (x origin), float bottom (y origin), float width of rectangle, float height of rectangle
//         if empty rectangle is passed in then robot will relocate in 20x20 area which takes a minute or more.
extern "C" __declspec(dllexport) int recoverLocalization(float left, float bottom, float width, float height)
{
	RectangleF area(left, bottom, width, height);
	MoveAction action = sdp.recoverLocalization(area);
	return action.waitUntilDone();
}

//-----------------------------------------------------------
// Set whether SLAMWARE system will perform map update to the default map kind (EXPLORER).
// SLAMWARE system enters localization enhanced mode once map update is disabled.

extern "C" __declspec(dllexport) int setUpdate(int enable)
{
	int result = 0;
	result = static_cast<int>(sdp.setMapUpdate(enable));
	if(!result)
	{
		std::cerr << "setMapUpdate(true) failed." << endl;
	}
	
	return result;
}

//-----------------------------------------------------------
// Call free on pointer allocated by malloc (e.g. _strdup)
extern "C" __declspec(dllexport) void freeIt(void *ptr)
{
	if (ptr)
	{
		free(ptr);
	}
}

//-----------------------------------------------------------
extern "C" __declspec(dllexport) SensorValueStruct getSensorValue(int id)
{
	ImpactSensorValue sensorValue;
	bool result = sdp.getSensorValue(id, sensorValue);

	SensorValueStruct svs;
	svs.time = 0;
	svs.value = 0;

	if (result)
	{
		svs.time = sensorValue.time;
		svs.value = sensorValue.value;
	}

	return svs;
}
