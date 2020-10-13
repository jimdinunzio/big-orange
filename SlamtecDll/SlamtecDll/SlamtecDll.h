// SlamtecDll.h : main header file for the SlamtecDll DLL
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <rpos/features/location_provider/map.h>
//#include <rpos/robot_platforms/objects/composite_map.h>
#include <rpos/robot_platforms/objects/composite_map_reader.h>
#include <rpos/robot_platforms/objects/composite_map_writer.h>
//#include <rpos/robot_platforms/objects/grid_map_layer.h>
//#include <rpos/robot_platforms/objects/line_map_layer.h>
//#include <rpos/robot_platforms/objects/pose_map_layer.h>
//#include <rpos/robot_platforms/objects/points_map_layer.h>

using namespace std;									//for connect
using namespace rpos::robot_platforms;					//for everything
using namespace rpos::robot_platforms::objects;			//for map
//using namespace rpos::features;
using namespace rpos::features::location_provider;		//for locations, map
//using namespace rpos::features::artifact_provider;		//for artifacts (walls, etc)
////using namespace rpos::features::system_resource;		//for docking, health, network
//using namespace rpos::features::impact_sensor;			//for sensors
//using namespace rpos::features::motion_planner;			//for movement, health
//using namespace rpos::features::artifact_provider;		//for artifacts, health
//using namespace rpos::core;								//for map
//using namespace rpos::system::types;					//for map (_U8)

SlamwareCorePlatform sdp;

// CSlamtecDllApp
// See SlamtecDll.cpp for the implementation of this class
//

class CSlamtecDllApp : public CWinApp
{
public:
	CSlamtecDllApp();

// Overrides
public:
	virtual BOOL InitInstance();

	DECLARE_MESSAGE_MAP()
};

typedef struct ExportPose
{
	float x;
	float y;
	float yaw;
} ExportPose;
