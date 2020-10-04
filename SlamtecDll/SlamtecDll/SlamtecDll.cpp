// SlamtecDll.cpp : Defines the initialization routines for the DLL.
//

#include "stdafx.h"
#include "SlamtecDll.h"
#include "bitmap_image.hpp"		//Slamtec bitmap functions
#include <iostream>				//for cout
#include <fstream>				//for cout to a file

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
extern "C" __declspec(dllexport) void movetoFloat(float xMove, float yMove);
extern "C" __declspec(dllexport) void movetoInteger(int xMove, int yMove);
extern "C" __declspec(dllexport) void rotate(float angle);
extern "C" __declspec(dllexport) void wakeup();
extern "C" __declspec(dllexport) void cancelMoveAction();
extern "C" __declspec(dllexport) int getMoveActionStatus();
extern "C" __declspec(dllexport) const char* getMoveActionError();
extern "C" __declspec(dllexport) void waitUntilMoveActionDone();

extern "C" __declspec(dllexport) int battery();
extern "C" __declspec(dllexport) float odometry();
extern "C" __declspec(dllexport) const char* SlamtecLocation();
extern "C" __declspec(dllexport) float pose();
extern "C" __declspec(dllexport) void home();
extern "C" __declspec(dllexport) int getSpeed();
extern "C" __declspec(dllexport) int setSpeed(int speed);
extern "C" __declspec(dllexport) int getLaserScan();
extern "C" __declspec(dllexport) int clearSlamtecMap();
extern "C" __declspec(dllexport) int loadSlamtecMap(const char* str_mapName);
extern "C" __declspec(dllexport) int saveSlamtecMap(const char* str_mapName);

	//  the following is for laser scan, make cout goto a file
	std::ofstream file;
	std::streambuf* sbuf = std::cout.rdbuf();

	//............. MAP .........................
	const char *opt_file_name;
	const float map_resolution = 0.05;
	bool opt_show_help = false;
	bool opt_get_stcm = false;
	bool opt_set_stcm = false;
	bool opt_modify_stcm = false;

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
	rpos::core::ACTION_DIRECTION actionDirection = rpos::core::ACTION_DIRECTION::FORWARD;
	rpos::core::Direction direction(actionDirection);
	rpos::actions::MoveAction moveBy = sdp.moveBy(direction);
}

//-----------------------------------------------------
// move left
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void left()
{
	rpos::core::ACTION_DIRECTION actionDirection = rpos::core::ACTION_DIRECTION::TURNLEFT;
	rpos::core::Direction direction(actionDirection);
	rpos::actions::MoveAction moveBy = sdp.moveBy(direction);
}

//-----------------------------------------------------
// move right
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void right()
{
	rpos::core::ACTION_DIRECTION actionDirection = rpos::core::ACTION_DIRECTION::TURNRIGHT;
	rpos::core::Direction direction(actionDirection);
	rpos::actions::MoveAction moveBy = sdp.moveBy(direction);
}

//-----------------------------------------------------
// move backward
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void back()
{
	rpos::core::ACTION_DIRECTION actionDirection = rpos::core::ACTION_DIRECTION::BACKWARD;
	rpos::core::Direction direction(actionDirection);
	rpos::actions::MoveAction moveBy = sdp.moveBy(direction);
}


//-----------------------------------------------------
// move to x,y using float
//	Input: float x and y
//	Output:	none
extern "C" __declspec(dllexport) void movetoFloat(float xMove, float yMove)
{
	rpos::core::Location loc(xMove, yMove);		//get loacation
	rpos::actions::MoveAction moveTo = sdp.moveTo(loc, false, true);		//move there
}

//-----------------------------------------------------
// move to x,y using float using integer
//	Input: string x and y in integer where value = floating pt * 1000
//	Output:	none
extern "C" __declspec(dllexport) void movetoInteger(int xMove, int yMove)
{
	rpos::core::Location loc(xMove / 1000.0, yMove / 1000.0);		//get loacation
	rpos::actions::MoveAction moveTo = sdp.moveTo(loc, false, true);		//move there
}

//----------------------------------------------------------------------
// rotate
//	Input: float angle, positive valus goes left, negative value goes right
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
	 rpos::actions::MoveAction moveAction = sdp.getCurrentAction();
	 moveAction.cancel();
}

//----------------------------------------------------------------------
// getMoveActionStatus
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) int getMoveActionStatus()
{
	rpos::actions::MoveAction moveAction = sdp.getCurrentAction();
	return moveAction.getStatus();
}

//----------------------------------------------------------------------
// getMoveActionError
//	Input: none
//	Output:	const char* - error string
extern "C" __declspec(dllexport) const char* getMoveActionError()
{
	rpos::actions::MoveAction moveAction = sdp.getCurrentAction();
	return strdup(moveAction.getReason().c_str());
}

//----------------------------------------------------------------------
// waitUntilMoveActionDone
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) void waitUntilMoveActionDone()
{
	rpos::actions::MoveAction moveAction = sdp.getCurrentAction();
	moveAction.waitUntilDone();
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
// get odometry
//	Input: none
//	Output:	float => odometry
extern "C" __declspec(dllexport) float odometry()
{
	return sdp.getOdometry();
}

//----------------------------------------------------------
// get location
//	Input: none
//	Output:	float => x
extern "C" __declspec(dllexport) const char* SlamtecLocation()
{
	rpos::core::Location location = sdp.getLocation();
	CString str_temp;
	str_temp.Format("%f,%f", location.x(), location.y());
	return _strdup((LPCTSTR)str_temp);
}

//----------------------------------------------------------
// get pose
//	Input: none
//	Output:	float => heading
extern "C" __declspec(dllexport) float pose()
{
	rpos::core::Pose pose = sdp.getPose();
	//str_temp.Format("Pose: x=%f, y=%f, yaw=%f, heading=%f\r\n", pose.x(), pose.y(), pose.yaw(), pose.yaw()*180.0f/M_PI);
	float heading = pose.yaw()*180.0f/M_PI;
	return heading;
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
	if(std_s == "low")
		return 1;
	if(std_s == "medium")
		return 2;
	if(std_s == "high")
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
extern "C" __declspec(dllexport) int getLaserScan()
{
	rpos::features::system_resource::LaserScan laser_scan = sdp.getLaserScan();
	std::vector<rpos::core::LaserPoint> laser_points = laser_scan.getLaserPoints();
	for(std::vector<rpos::core::LaserPoint>::iterator it = laser_points.begin(); it != laser_points.end(); ++it)
	{
		std::cout << "Angle: " << it->angle() << "; Distance: " << it->distance() << "; is Valid: " << it->valid() << std::endl;
	}

	return 0;
}

//----------------------------------------------------------
// clear Slamtec map
// clear map
//	Input: none
//	Output:	none
extern "C" __declspec(dllexport) int clearSlamtecMap()
{
	sdp.clearMap();
	return 0;
}

//----------------------------------------------------------
// load Slamtec map
// open map file and upload to SDP
//	Input: filename
//	Output:	int => 0 = success, 1 = err
extern "C" __declspec(dllexport) int loadSlamtecMap(const char* str_mapName)
{
	//get map file
	opt_file_name = str_mapName;

	opt_set_stcm = true;
	opt_get_stcm = false;
	opt_modify_stcm = false;
	rpos::core::Pose pose = sdp.getPose();
	CompositeMapReader composite_map_reader;
	std::string error_message;
	boost::shared_ptr<CompositeMap> composite_map(composite_map_reader.loadFile(error_message, opt_file_name));
	if (composite_map)
	{
		sdp.setCompositeMap((*composite_map), pose);	
	}
    return 0;
}

//----------------------------------------------------------
// save Slamtec map
// save map file
//	Input: filename
//	Output:	int => 0 = success, 1 = err
extern "C" __declspec(dllexport) int saveSlamtecMap(const char* str_mapName)
{
	opt_file_name = str_mapName;
	opt_get_stcm = true;
	opt_set_stcm = false;
	opt_modify_stcm = false;
	// Saving CompositeMap to Local File...
	CompositeMap composite_map = sdp.getCompositeMap();
	CompositeMapWriter composite_map_writer;
	std::string error_message;
	bool result = composite_map_writer.saveFile(error_message, opt_file_name, composite_map);	
    return 0;
}

