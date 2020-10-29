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
//		rpos::actions::MoveAction action = sdp.getCurrentAction();

		while(1)
		{
			connected = getContrState(0, contr_state);
			if(connected)
			{
				if(contr_state.dwPacketNumber != last_contr_state.dwPacketNumber)
				{
					// something has changed
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

					float theta = atan2(leftThumbNormY, leftThumbNormX) - M_PI / 2.0f;

					std::cout << "Throttle: " << rightTrigger * 100.0 << "%" << std::endl;
					std::cout << "Theta: " << theta * 180.0f / M_PI << " degrees." << std::endl;

					MoveOptions moveOptions;
					moveOptions.speed_ratio = rightTrigger;
					rpos::actions::MoveAction action = sdp.moveBy(theta, moveOptions);
					if (action.getStatus() == rpos::core::ActionStatusError)
					std::cout << "Action Failed: " << action.getReason() << std::endl;
					//		bool bRet2 =sdp.setSystemParameter(SYSPARAM_ROBOT_SPEED, SYSVAL_ROBOT_SPEED_HIGH);
					action.waitUntilDone();

					//std::cout << "Left Trigger: " << (int)contr_state.Gamepad.bLeftTrigger << std::endl;
					//std::cout << "Right Trigger: " << (int)contr_state.Gamepad.bRightTrigger << std::endl;
					//std::cout << "Left Thumbstick: ( " << contr_state.Gamepad.sThumbLX << ", " << contr_state.Gamepad.sThumbLY << " )" << std::endl;
					//std::cout << "Right Thumbstick: ( " << contr_state.Gamepad.sThumbRX << ", " << contr_state.Gamepad.sThumbRY << " )" << std::endl;
				}
				else
				{
					// nothing has changed
				}
			}
			else // !connected
			{
				std::cout << "Lost connection to controller 0. Will try to reconnect in 2s." << std::endl;
				Sleep(2000);
			}
			Sleep(15);
			last_contr_state = contr_state;
		}

	}
	catch (const rpos::system::detail::ExceptionBase& e) {		
		std::cout << e.what() << std::endl;
		return 1;	
	}
	return 0;
} 
	
				