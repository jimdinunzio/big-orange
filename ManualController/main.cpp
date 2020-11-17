#include <regex>
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <Windows.h>
#include <Xinput.h>
#include <math.h>

using namespace rpos::robot_platforms;
using namespace rpos::features::motion_planner;
using namespace rpos::features::artifact_provider;

std::string ip_address = "";
const char *ip_regex = "\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}";
const double pi = 3.1415927;

void ShowHelp(std::string app_name) {
	std::cout << "Manual Controller Daemon" << std::endl << \
		"Usage: " << app_name << " <slamware_address>" << std::endl;
}

bool ParseCommandLine(int argc, const char *argv[]) {
	bool opt_show_help = false;
	for (int pos = 1; pos < argc; ++pos) {
		const char *current = argv[pos];
		if (strcmp(current, "-h") == 0)
			opt_show_help = true;
	    else 
			ip_address = current;		
	}
	std::regex reg(ip_regex);
	if (!opt_show_help && !std::regex_match(ip_address, reg)) 
		opt_show_help = true;
	if (opt_show_help) {
		ShowHelp("controller_daemon");
		return false;
	}
	return true;
}

bool getContrState(int dwUserIndex, XINPUT_STATE& state)
{
	DWORD dwResult;    
	ZeroMemory( &state, sizeof(XINPUT_STATE) );

	// Simply get the state of the controller from XInput.
	dwResult = XInputGetState( dwUserIndex, &state );

	if( dwResult == ERROR_SUCCESS )
	{
		// Controller is connected
		return true;
	}
	else
	{
		// Controller is not connected
		return false;
	}
}

float getNormTrigger(unsigned char v, const short threshhold)
{
	float normV;

	if(v > threshhold)
	{
		normV = (v - threshhold) / (255.0f - threshhold);
	}
	else
	{
		normV = 0.f;
	}
	return normV;
}


void getNormThumb(short x, short y, const short deadzone, float& normX, float& normY, float& mag)
{
	//determine how far the controller is pushed
	float magnitude = sqrt((float)(x*x + y*y));

	//determine the direction the controller is pushed
	normX = x / magnitude;
	normY = y / magnitude;

	float normalizedMagnitude = 0;

	//check if the controller is outside a circular dead zone
	if (magnitude > deadzone)
	{
		//clip the magnitude at its expected maximum value
		if (magnitude > 32767) 
			magnitude = 32767;

		//adjust magnitude relative to the end of the dead zone
		magnitude -= deadzone;

		//optionally normalize the magnitude with respect to its expected range
		//giving a magnitude value of 0.0 to 1.0
		mag = magnitude / (32767 - deadzone);
	}
	else //if the controller is in the deadzone zero out the magnitude
	{
		mag = 0.0;
	}
}

/*
typedef struct _XINPUT_STATE {
  DWORD          dwPacketNumber;
  XINPUT_GAMEPAD Gamepad;
} XINPUT_STATE, *PXINPUT_STATE;

typedef struct _XINPUT_GAMEPAD {
  WORD  wButtons;
  BYTE  bLeftTrigger;
  BYTE  bRightTrigger;
  SHORT sThumbLX;
  SHORT sThumbLY;
  SHORT sThumbRX;
  SHORT sThumbRY;
} XINPUT_GAMEPAD, *PXINPUT_GAMEPAD;

*/
int main(int argc, const char *argv[]) {
	if (!ParseCommandLine(argc, argv)) return 1;

	XINPUT_STATE contr_state, last_contr_state;
	ZeroMemory( &last_contr_state, sizeof(XINPUT_STATE) );
	bool connected = false;
	while(1)
	{
		std::cout << "Trying to connect to controller 0..." << std::endl;
		connected = getContrState( 0, contr_state );
		if(connected)
		{
			std::cout << "Connected to controller 0" << std::endl;
			break;
		}
		else // !connected
		{
			Sleep(2000);
		}
	}

	std::cout << "Connecting SDP @ " << ip_address << "..."<< std::endl;
	try {
		SlamwareCorePlatform sdp = SlamwareCorePlatform::connect(ip_address, 1445);
		std::cout <<"SDK Version: " << sdp.getSDKVersion() << std::endl;
		std::cout <<"SDP Version: " << sdp.getSDPVersion() << std::endl;
		
		// if any action running, cancel it.
		rpos::actions::MoveAction action = sdp.getCurrentAction();
		if (action)
			action.cancel();

		float lastRightTrigger = 0.0f;
		// main control loop
		while(1)
		{
			Sleep(50);
			connected = getContrState(0, contr_state);
			if(connected)
			{
				// Red B button is the kill switch to robot
				if((contr_state.Gamepad.wButtons & XINPUT_GAMEPAD_B) != 0)
				{
					sdp.getCurrentAction().cancel();
					continue;
				}
				float rightTrigger = getNormTrigger(contr_state.Gamepad.bRightTrigger, XINPUT_GAMEPAD_TRIGGER_THRESHOLD);
				float leftThumbMag;
				float leftThumbNormX;
				float leftThumbNormY;
				getNormThumb(contr_state.Gamepad.sThumbLX, 
								contr_state.Gamepad.sThumbLY,
								XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE,
								leftThumbNormX, 
								leftThumbNormY, 
								leftThumbMag);

				// compute theta and rotate 90 degrees counter clockwise so 0 degrees is North (or forward)
				float theta = atan2f(leftThumbNormY, leftThumbNormX) - (float)M_PI / 2.f;
				if(leftThumbMag == 0.0f)
					theta = 0.0f;
				
				// normalize angle
				theta = atan2(sin(theta), cos(theta));
				// scale to use 0-100 range for trigger	
				rightTrigger *= 100.f;

				rpos::actions::MoveAction curAction = sdp.getCurrentAction();

				if(lastRightTrigger > 0.f && rightTrigger == 0.f)
				{
					curAction.cancel();
					lastRightTrigger = rightTrigger;
					continue;
				}
				else if(rightTrigger == 0.f)
				{
					continue;
				}

				if(rightTrigger <= 44.f)
					sdp.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_LOW);
				else if(rightTrigger > 44.f && rightTrigger <= 88)
					sdp.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_MEDIUM);
				else // rightTrigger > 88
					sdp.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_HIGH);

				MoveOptions moveOptions;
				moveOptions.flag = MoveOptionFlagAppending;
				moveOptions.speed_ratio = rightTrigger * 100;

#if USE_4_DIRECTION_MODE
				theta = theta * 180. / M_PI;
				rpos::core::Direction direction;
				
				if (theta >= -45. && theta < 0. || theta >= 0. && theta < 45.)
					direction = rpos::core::FORWARD;
				else if (theta >= 45. && theta < 135.)
					direction = rpos::core::TURNLEFT;
				else if (theta >= 135. && theta < 180. || theta >= -180 && theta < -135.)
					direction = rpos::core::BACKWARD;
				else // if (theta < -45 && theta > -135)
					direction = rpos::core::TURNRIGHT;
				rpos::actions::MoveAction action = sdp.moveBy(direction);
#else
				// Move robot in theta angle direction. e.g. 0 is forward, 180 is backward, 90 is rotate left, 45 is forward while turning
				// This works well for controlling by analog thumb stick.
				rpos::features::motion_planner::MoveOptions options;
				rpos::actions::MoveAction action = sdp.moveBy(theta, options);
#endif

#ifdef DEBUG
				std::cout << "MoveBy theta = " << (float)(theta * 180.f / M_PI) << " degrees. Throttle = " << rightTrigger << std::endl;
				//std::cout << "Moving " << direction.direction() << std::endl;
#endif

				if (action.getStatus() == rpos::core::ActionStatusError)
				std::cout << "Action Failed: " << action.getReason() << std::endl;
				//		bool bRet2 =sdp.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_HIGH);
				//action.waitUntilDone();

				//std::cout << "Left Trigger: " << (int)contr_state.Gamepad.bLeftTrigger << std::endl;
				//std::cout << "Right Trigger: " << (int)contr_state.Gamepad.bRightTrigger << std::endl;
				//std::cout << "Left Thumbstick: ( " << contr_state.Gamepad.sThumbLX << ", " << contr_state.Gamepad.sThumbLY << " )" << std::endl;
				//std::cout << "Right Thumbstick: ( " << contr_state.Gamepad.sThumbRX << ", " << contr_state.Gamepad.sThumbRY << " )" << std::endl;
				lastRightTrigger = rightTrigger;
			}
			else // !connected
			{
				std::cout << "Lost connection to controller 0. Will try to reconnect in 2s." << std::endl;
				Sleep(2000);
			}
//			last_contr_state = contr_state;
		}

	}
	catch (const rpos::system::detail::ExceptionBase& e) {		
		std::cout << e.what() << std::endl;
		return 1;	
	}
	return 0;
} 
	
				