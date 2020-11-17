#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <rpos/robot_platforms/slamware_core_platform.h>
#include <Windows.h>
#include "NuiApi.h"

//#define DEBUG

INuiSensor*             g_pNuiSensor;
HANDLE					g_pDepthStreamHandle;
HANDLE                  g_hNextDepthFrameEvent;

static const int        cDepthWidth  = 320;
static const int        cDepthHeight = 240;
static const size_t NUM_FRAME_DATA = cDepthWidth * cDepthHeight;

// The NUI image resolution must be the same as the cDepthWidth x cDepthHeight values.
static const NUI_IMAGE_RESOLUTION cNUI_IMAGE_RESOLUTION = NUI_IMAGE_RESOLUTION_320x240;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
    if ( pInterfaceToRelease != NULL )
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

void shutdown()
{
	if (g_pNuiSensor)
    {
        g_pNuiSensor->NuiShutdown();
    }

    if (g_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(g_hNextDepthFrameEvent);
    }

    SafeRelease(g_pNuiSensor);
}

/// <summary>
/// Create the first connected Kinect found 
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT createFirstConnected()
{
    INuiSensor * pNuiSensor;
    HRESULT hr;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr))
    {
        return hr;
    }

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr))
        {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr)
        {
            g_pNuiSensor = pNuiSensor;
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }

    if (NULL != g_pNuiSensor)
    {
        // Initialize the Kinect and specify that we'll be using depth
        hr = g_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH); 
        if (SUCCEEDED(hr))
        {
            // Create an event that will be signaled when depth data is available
            g_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			
            // Open a depth image stream to receive depth frames
            hr = g_pNuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_DEPTH,
                cNUI_IMAGE_RESOLUTION,
                0,
                2,
                g_hNextDepthFrameEvent,
                &g_pDepthStreamHandle);
			if (SUCCEEDED(hr))
			{
				std::cout << "Connected to Kinect Sensor." << std::endl;
			}
        }
    }

    if (NULL == g_pNuiSensor || FAILED(hr))
    {
        std::cout << "No ready Kinect found!" << std::endl;
		shutdown();
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new depth data
/// </summary>
void processDepth(std::vector<float>& frame_buffer)
{
    HRESULT hr;
    NUI_IMAGE_FRAME imageFrame;

    // Attempt to get the depth frame
    hr = g_pNuiSensor->NuiImageStreamGetNextFrame(g_pDepthStreamHandle, 0, &imageFrame);
    if (FAILED(hr))
    {
        return;
    }

    BOOL nearMode;
    INuiFrameTexture* pTexture;

    // Get the depth image pixel texture
    hr = g_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
        g_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
    if (FAILED(hr))
    {
        goto ReleaseFrame;
    }

    NUI_LOCKED_RECT LockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
    pTexture->LockRect(0, &LockedRect, NULL, 0);

    // Make sure we've received valid data
    if (LockedRect.Pitch != 0)
    {
        // Get the min and max reliable depth for the current frame
        int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
        int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

        const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);

		const int cTwoDepthWidth = cDepthWidth * 2;

		float* pFb = &frame_buffer[cDepthWidth];
		USHORT depth;
		for(int y = 0; y < cDepthHeight; ++y)
		{
			for(int x = 0; x < cDepthWidth; ++x)
			{
				depth = pBufferRun->depth;
				*--pFb = depth >= minDepth && depth <= maxDepth ? depth / 1000.0f : 0.0f;
	            ++pBufferRun;
			}
			pFb += cTwoDepthWidth;
		}
    }

    // We're done with the texture so unlock it
    pTexture->UnlockRect(0);
    pTexture->Release();

ReleaseFrame:
    // Release the frame
    g_pNuiSensor->NuiImageStreamReleaseFrame(g_pDepthStreamHandle, &imageFrame);
}


/// <summary>
/// Main processing function
/// </summary>
void update(std::vector<float>& frame_buffer)
{
    if (NULL == g_pNuiSensor)
    {
        return;
    }

    if ( WAIT_OBJECT_0 == WaitForSingleObject(g_hNextDepthFrameEvent, 0) )
    {
        processDepth(frame_buffer);
    }
}

int main(int argc, char* argv[])
{
	try
	{
		rpos::robot_platforms::SlamwareCorePlatform platform = rpos::robot_platforms::SlamwareCorePlatform::connect("192.168.11.1", 1445);
		
		rpos::message::depth_camera::DepthCameraFrame frame;         
		frame.minValidDistance = 0.8f;                   //frame.minValidDistance = camera_attr.minValidDistance;
		frame.maxValidDistance = 4.0f;
		frame.minFovPitch      = -45/2*3.14f/180;
		frame.maxFovPitch      = 45/2*3.14f/180;
		frame.minFovYaw        = -58/2*3.14f/180;
		frame.maxFovYaw        = 58/2*3.14f/180;
		frame.cols             = cDepthWidth;
		frame.rows             = cDepthHeight;

		
	    const int eventCount = 1;
	    HANDLE hEvents[eventCount];

		// Look for a connected Kinect, and create it if found
        while(FAILED(createFirstConnected()))
		{
			std::cout << "Trying to connect to kinect sensor..." << std::endl;
			boost::this_thread::sleep(boost::posix_time::millisec(2000));
		}

		frame.data.resize(NUM_FRAME_DATA);
		std::vector<float> frame_buffer(cDepthWidth * cDepthHeight, 0);
		const boost::posix_time::time_duration cMinTimeBetweenPublishing = boost::posix_time::millisec(105);
		const boost::posix_time::time_duration cZeroTime = boost::posix_time::millisec(0);
		boost::posix_time::ptime lastTime = boost::posix_time::microsec_clock::local_time();
#ifdef DEBUG
		const boost::posix_time::time_duration cOneSecond = boost::posix_time::millisec(1000);
		boost::posix_time::ptime lastSecond = boost::posix_time::microsec_clock::local_time();
#endif
		int pubCount= 0;
		while (1)
		{
			hEvents[0] = g_hNextDepthFrameEvent;

			// Check to see if we have a Kinect event (hEvents)
			// Update() will check for Kinect events individually, in case more than one are signalled
			MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLEVENTS);

			// Explicitly check the Kinect frame event since MsgWaitForMultipleObjects
			// can return for other reasons even though it is signaled.
			update(frame.data);

			int sensorId = 3; //TODO: Config your own SensorId
			
			// Publish one single frame no faster than 10/s.
			boost::posix_time::time_duration waitTime = cMinTimeBetweenPublishing - 
				(boost::posix_time::microsec_clock::local_time() - lastTime);
			if(waitTime > cZeroTime)
			{
				//std::cout << "curTime = " << boost::posix_time::microsec_clock::local_time() << "lasttime = " << lastTime << ", sleep waiting " << waitTime << " between publising" << std::endl;
				boost::this_thread::sleep(waitTime);
			}
			lastTime = boost::posix_time::microsec_clock::local_time();
			platform.publishDepthCamFrame(sensorId, frame);
#ifdef DEBUG
			pubCount++;
			if(boost::posix_time::microsec_clock::local_time() - lastSecond >= cOneSecond)
			{
				std::cout << pubCount << " publishes per second" << std::endl;
				pubCount = 0;
				lastSecond = boost::posix_time::microsec_clock::local_time();
			}
#endif
		}

	}
	catch(const std::exception& ex)
	{
		std::string ex_str(ex.what());
	}

	shutdown();
	return 0;
}