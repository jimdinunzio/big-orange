#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <rpos/robot_platforms/slamware_core_platform.h>
//#include <Windows.h>
#include <cstdio>

#define PUBLISH_TO_SLAMTEC

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
using namespace std;

//#define DEBUG

static const std::string DEPTH_DEVICE_ID = "14442C10E18CC0D200";
static const std::string SLAMWARE_IP_ADDR_STR = "192.168.11.1";
static const int DEFAULT_CONF_THRESHOLD = 200;
static const int SLAMWARE_PORT = 1445;

static const int        cDepthWidth  = 320;
static const int        cDepthHeight = 200;
static const size_t NUM_FRAME_DATA = cDepthWidth * cDepthHeight;

// Closer - in minimum depth, disparity range is doubled(from 95 to 190) :
static const bool extended_disparity = false;
// Better accuracy for longer distance, fractional disparity 32 - levels:
static const bool subpixel = false;
// Better handling for occlusions:
static const bool lr_check = false;

/// <summary>
/// Main processing function
/// </summary>
void updateSlamtecBuffer(std::vector<float>& slamtec_depth_buffer, const cv::Mat& cv_depth_frame)
{
	assert(cv_depth_frame.depth() == CV_16U);

	int nRows = cv_depth_frame.rows;
	int nCols = cv_depth_frame.cols;

	assert(slamtec_depth_buffer.size() == nRows * nCols);

	if (cv_depth_frame.isContinuous())
	{
		nCols *= nRows;
		nRows = 1;
	}

	int i, j;
	const uint16_t* p;
	float* p_slamtec = &slamtec_depth_buffer[0];

	for (i = 0; i < nRows; ++i)
	{
		p = cv_depth_frame.ptr<uint16_t>(i);
		for (j = 0; j < nCols; ++j)
		{
			*p_slamtec++ = *p++ / 1000.0f; // convert to meters
		}
	}
}

float deg2rad(float deg)
{
	return deg * (float)M_PI / 180.f;
}

static void onFilterChange(int pos, void* ptr)
{
	dai::StereoDepthProperties::MedianFilter *pFilter = static_cast<dai::StereoDepthProperties::MedianFilter*>(ptr);
	switch (pos)
	{
	case 0:
		*pFilter = dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF;
		break;
	case 1:
		*pFilter = dai::StereoDepthProperties::MedianFilter::KERNEL_3x3;
		break;
	case 2:
		*pFilter = dai::StereoDepthProperties::MedianFilter::KERNEL_5x5;
		break;
	case 3:
		*pFilter = dai::StereoDepthProperties::MedianFilter::KERNEL_7x7;
		break;
	}
}

int main(int argc, char* argv[])
{
	while (1)
	{
		try
		{
#ifdef PUBLISH_TO_SLAMTEC
			rpos::robot_platforms::SlamwareCorePlatform platform = rpos::robot_platforms::SlamwareCorePlatform::connect(SLAMWARE_IP_ADDR_STR, SLAMWARE_PORT);
			std::cout << "Connected to Slamware Core at " << SLAMWARE_IP_ADDR_STR << " port: " << SLAMWARE_PORT << endl;
			std::cout << "SDK Version: " << platform.getSDKVersion() << std::endl;
			std::cout << "SDP Version: " << platform.getSDPVersion() << std::endl;
#endif

			rpos::message::depth_camera::DepthCameraFrame slamtecDepthFrame;
			slamtecDepthFrame.minValidDistance = 0.35f;                   //slamtecDepthFrame.minValidDistance = camera_attr.minValidDistance;
			slamtecDepthFrame.maxValidDistance = 4.0f;
			slamtecDepthFrame.minFovPitch = deg2rad(-50.f / 2.f);
			slamtecDepthFrame.maxFovPitch = deg2rad(50.f / 2.f);
			slamtecDepthFrame.minFovYaw = deg2rad(-71.86f / 2.f);
			slamtecDepthFrame.maxFovYaw = deg2rad(71.86f / 2.f);
			slamtecDepthFrame.cols = cDepthWidth;
			slamtecDepthFrame.rows = cDepthHeight;

			slamtecDepthFrame.data.resize(NUM_FRAME_DATA);
			const boost::posix_time::time_duration cMinTimeBetweenPublishing = boost::posix_time::millisec(105);
			const boost::posix_time::time_duration cZeroTime = boost::posix_time::millisec(0);
			boost::posix_time::ptime lastTime = boost::posix_time::microsec_clock::local_time();
#ifdef DEBUG
			const boost::posix_time::time_duration cOneSecond = boost::posix_time::millisec(1000);
			boost::posix_time::ptime lastSecond = boost::posix_time::microsec_clock::local_time();
#endif
			int pubCount = 0;
			dai::Pipeline p;
			std::vector<std::string> queueNames;

			auto left = p.create<dai::node::MonoCamera>();
			left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
			left->setBoardSocket(dai::CameraBoardSocket::LEFT);
			left->initialControl.setManualFocus(130);

			auto right = p.create<dai::node::MonoCamera>();
			right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
			right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
			right->initialControl.setManualFocus(130);

			auto stereo = p.create<dai::node::StereoDepth>();
			stereo->setLeftRightCheck(lr_check);
			stereo->setExtendedDisparity(extended_disparity);
			stereo->setSubpixel(subpixel);

			left->out.link(stereo->left);
			right->out.link(stereo->right);
			
			auto leftOut = p.create<dai::node::XLinkOut>();
			leftOut->setStreamName("left");
			queueNames.push_back("left");
			left->out.link(leftOut->input);

			auto depthOut = p.create<dai::node::XLinkOut>();
			depthOut->setStreamName("depth");
			queueNames.push_back("depth");
			stereo->depth.link(depthOut->input);

			//auto disparityOut = p.create<dai::node::XLinkOut>();
			//disparityOut->setStreamName("disparity");
			//queueNames.push_back("disparity");
			//stereo->disparity.link(disparityOut->input);

			// set up stereo depth config
			auto xinStereoDepthConfig = p.create<dai::node::XLinkIn>();
			xinStereoDepthConfig->setStreamName("stereoDepthConfig");
			xinStereoDepthConfig->out.link(stereo->inputConfig);

			std::tuple<bool, dai::DeviceInfo> devInfo = dai::Device::getDeviceByMxId(DEPTH_DEVICE_ID);
			bool found = std::get<0>(devInfo);
			std::cerr << "device " << DEPTH_DEVICE_ID << ", found = " << found << endl;
			if (!found)
			{
				std::cerr << "Cannot find obstacle avoidance depth camera." << endl;
				return 1;
			}

			// Connect to device and start pipeline
			dai::Device *d = new dai::Device(p, std::get<1>(devInfo));

			// Sets queues size and behavior
			for (const auto& name : queueNames)
			{
				d->getOutputQueue(name, 4, false);
			}

			// input queues
			auto stereoDepthConfigInQueue = d->getInputQueue("stereoDepthConfig");

			cv::Mat depthFrameColor(200, 320, CV_8UC1), depthFrameCv(400, 640, CV_16UC1), halfDepthFrameCv(200, 320, CV_16UC1);
			//cv::Mat disparityFrame(400, 640, CV_8UC1);

			dai::StereoDepthConfig currentStereoDepthConfig = dai::StereoDepthConfig();

			int confThresh = DEFAULT_CONF_THRESHOLD;
			int lastConfThresh = 0;
			int bilateralSigma = stereo->initialConfig.getBilateralFilterSigma();
			int lastBilateralSigma = bilateralSigma;
			
			const cv::String depthColorWindowName = "depth_color";
			cv::namedWindow(depthColorWindowName);
			
			cv::createTrackbar("Confidence", depthColorWindowName, &confThresh, 255);
			cv::setTrackbarPos("Confidence", depthColorWindowName, confThresh);
			stereo->initialConfig.setConfidenceThreshold(confThresh);
			currentStereoDepthConfig.setConfidenceThreshold(confThresh);

			cv::createTrackbar("Bilateral sigma", depthColorWindowName, &bilateralSigma, 250);
			cv::setTrackbarPos("Bilateral sigma", depthColorWindowName, bilateralSigma);

			dai::StereoDepthProperties::MedianFilter medianFilter = dai::StereoDepthProperties::MedianFilter::KERNEL_5x5;
			dai::StereoDepthProperties::MedianFilter lastMedianFilter = medianFilter;
			cv::createTrackbar("Filter", depthColorWindowName, nullptr, 3, onFilterChange, &medianFilter);
			cv::setTrackbarPos("Filter", depthColorWindowName, 2);
			stereo->initialConfig.setMedianFilter(medianFilter);
			currentStereoDepthConfig.setMedianFilter(medianFilter);

			while (1)
			{
				std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

				if (confThresh != lastConfThresh || medianFilter != lastMedianFilter || bilateralSigma != lastBilateralSigma)
				{
					currentStereoDepthConfig.setConfidenceThreshold(confThresh);
					currentStereoDepthConfig.setMedianFilter(medianFilter);
					currentStereoDepthConfig.setBilateralFilterSigma(bilateralSigma);
					stereoDepthConfigInQueue->send(currentStereoDepthConfig);
					lastConfThresh = confThresh;
					lastMedianFilter = medianFilter;
					lastBilateralSigma = bilateralSigma;
				}

				auto queueEvents = d->getQueueEvents(queueNames);
				for (const auto& name : queueEvents)
				{
					auto packets = d->getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
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
							depthFrameCv = latestPacket[name]->getCvFrame();
							cv::resize(depthFrameCv, halfDepthFrameCv, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
							
							updateSlamtecBuffer(slamtecDepthFrame.data, halfDepthFrameCv);

							cv::normalize(depthFrameCv, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
							cv::equalizeHist(depthFrameColor, depthFrameColor);
							cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_JET);
							cv::imshow(depthColorWindowName, depthFrameColor);
						}
						//else if (name == "disparity") {
							//disparityFrame = latestPacket[name]->getCvFrame();
							//disparityFrame.convertTo(disparityFrame, CV_8UC1, 255. / stereo->getMaxDisparity());
							//// Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
							//cv::applyColorMap(disparityFrame, disparityFrame, cv::COLORMAP_JET);
							//cv::imshow("disparity_color", disparityFrame);
						//}
						else if (name == "left") 
						{
							cv::imshow("left", latestPacket[name]->getFrame());
						}
					}
				}

				const int sensorId = 3; //TODO: Config your own SensorId

				// Publish one single frame no faster than 10/s.
				boost::posix_time::time_duration waitTime = cMinTimeBetweenPublishing -
					(boost::posix_time::microsec_clock::local_time() - lastTime);
				if (waitTime > cZeroTime)
				{
#ifdef DEBUG
					std::cout << "curTime = " << boost::posix_time::microsec_clock::local_time() << "lasttime = " << lastTime << ", sleep waiting " << waitTime << " between publising" << std::endl;
#endif
					boost::this_thread::sleep(waitTime);
				}
				lastTime = boost::posix_time::microsec_clock::local_time();
#ifdef PUBLISH_TO_SLAMTEC
				platform.publishDepthCamFrame(sensorId, slamtecDepthFrame);
#endif
#ifdef DEBUG
				pubCount++;
				if (boost::posix_time::microsec_clock::local_time() - lastSecond >= cOneSecond)
				{
					std::cout << pubCount << " publishes per second" << std::endl;
					pubCount = 0;
					lastSecond = boost::posix_time::microsec_clock::local_time();
				}
#endif
				cv::waitKey(50);
			}
		}
		catch (const rpos::robot_platforms::ConnectionTimeOutException& ex)
		{
			std::string ex_str(ex.what());
			std::cout << "Exception thrown: " << ex_str;
			return 1;
		}
		catch (const rpos::robot_platforms::ConnectionFailException& ex)
		{
			std::string ex_str(ex.what());
			std::cout << "Exception thrown: " << ex_str;
			return 1;
		}
		catch (const std::exception& ex)
		{
			std::string ex_str(ex.what());
			std::cout << "Exception thrown: " << ex_str << endl;
			std::cout << "Attempting to restart publishing depth data\n" << endl;
		}
	}
	return 0;
}