#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <rpos/robot_platforms/slamware_core_platform.h>
//#include <Windows.h>
#include <cstdio>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
using namespace std;

//#define DEBUG

static const int        cDepthWidth  = 320;
static const int        cDepthHeight = 200;
static const size_t NUM_FRAME_DATA = cDepthWidth * cDepthHeight;

/// <summary>
/// Main processing function
/// </summary>
void update(std::vector<float>& frame_buffer, uint16_t* depth_frame)
{
	uint16_t* pDepth = depth_frame;
	float* pFrameBuffer = &frame_buffer[0];
	for (int y = 0; y < cDepthHeight; ++y)
	{
		for (int x = 0; x < cDepthWidth; ++x)
		{
			*pFrameBuffer++ = *pDepth / 1000.0f;
			pDepth += 2; // skip every other column
		}
		pDepth += (cDepthWidth << 1); // skip every other row
	}
}

// Closer - in minimum depth, disparity range is doubled(from 95 to 190) :
bool extended_disparity = false;
// Better accuracy for longer distance, fractional disparity 32 - levels:
bool subpixel = false;
// Better handling for occlusions:
bool lr_check = false;


int main(int argc, char* argv[])
{
	try
	{
		rpos::robot_platforms::SlamwareCorePlatform platform = rpos::robot_platforms::SlamwareCorePlatform::connect("192.168.11.1", 1445);
		
		rpos::message::depth_camera::DepthCameraFrame frame;         
		frame.minValidDistance = 0.35f;                   //frame.minValidDistance = camera_attr.minValidDistance;
		frame.maxValidDistance = 2.0f;
		frame.minFovPitch      = -81.f/2*M_PI/180.f;
		frame.maxFovPitch      = 81.f/2*M_PI/180.f;
		frame.minFovYaw        = -71.8f/2*M_PI /180.f;
		frame.maxFovYaw        = 71.8f/2*M_PI/180.f;
		frame.cols             = cDepthWidth;
		frame.rows             = cDepthHeight;

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
		dai::Pipeline p;
		std::vector<std::string> queueNames;

		auto left = p.create<dai::node::MonoCamera>();
		left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
		left->setBoardSocket(dai::CameraBoardSocket::LEFT);

		auto right = p.create<dai::node::MonoCamera>();
		right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
		right->setBoardSocket(dai::CameraBoardSocket::RIGHT);

		auto stereo = p.create<dai::node::StereoDepth>();
		stereo->setConfidenceThreshold(200);
		stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::KERNEL_7x7);
		stereo->setLeftRightCheck(lr_check);

		stereo->setExtendedDisparity(extended_disparity);

		stereo->setSubpixel(subpixel);

		left->out.link(stereo->left);
		right->out.link(stereo->right);

		auto depthOut = p.create<dai::node::XLinkOut>();
		depthOut->setStreamName("depth");
		queueNames.push_back("depth");
		stereo->depth.link(depthOut->input);
	
		// Connect to device and start pipeline
		dai::Device d(p);

		// Sets queues size and behavior
		for (const auto& name : queueNames) 
		{
			d.getOutputQueue(name, 4, false);
		}

		std::unordered_map<std::string, uint16_t*> depthFrame;

		while (1)
		{
			std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

			auto queueEvents = d.getQueueEvents(queueNames);
			for (const auto& name : queueEvents)
			{
				auto packets = d.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
				auto count = packets.size();
				if (count > 0) 
				{
					latestPacket[name] = packets[count - 1];
				}
			}

			for (const auto& name : queueNames) 
			{
				if (latestPacket.find(name) != latestPacket.end()) 
				{
					if (name == "depth") {
						depthFrame[name] = reinterpret_cast<uint16_t*>(&latestPacket[name]->getData()[0]);
					}
				}
			}

			update(frame.data, depthFrame["depth"]);

			int sensorId = 3; //TODO: Config your own SensorId
			
			// Publish one single frame no faster than 10/s.
			boost::posix_time::time_duration waitTime = cMinTimeBetweenPublishing - 
				(boost::posix_time::microsec_clock::local_time() - lastTime);
			if(waitTime > cZeroTime)
			{
#ifdef DEBUG
				std::cout << "curTime = " << boost::posix_time::microsec_clock::local_time() << "lasttime = " << lastTime << ", sleep waiting " << waitTime << " between publising" << std::endl;
#endif
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

	return 0;
}