#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <rpos/robot_platforms/slamware_core_platform.h>
//#include <Windows.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <cstdio>

#define ENABLE_SLAMTEC_PUBLISHING

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
using namespace std;

//#define DEBUG

static const std::string DEPTH_DEVICE_ID = "14442C10E18CC0D200";
static const std::string SLAMWARE_IP_ADDR_STR = "192.168.11.1";
static const int DEPTH_CONFIDENCE_THRESHOLD = 255;
static const int SLAMWARE_PORT = 1445;


static const int        cDepthWidth  = 320;
static const int        cDepthHeight = 200;
static const size_t		NUM_FRAME_DATA = cDepthWidth * cDepthHeight;
static const float		dWlsLambda = 8000;
static const float		dWlsSigma = 1.5;

// Closer - in minimum depth, disparity range is doubled(from 95 to 190) :
static const bool extended_disparity = false;
// Better accuracy for longer distance, fractional disparity 32 - levels:
static const bool subpixel = false;
// Better handling for occlusions:
static const bool lr_check = false;

static const int baseline = 75; //mm
static const int disp_levels = extended_disparity ? 192 : 96;
static const float hfov = 71.86f;

/// <summary>
/// Main processing function
/// </summary>
void updateSlamtecBuffer(std::vector<float>& slamtec_depth_buffer, const cv::Mat& cv_depth_frame)
{
	assert(cv_depth_frame.type() == CV_16SC1);

	int nRows = cv_depth_frame.rows;
	int nCols = cv_depth_frame.cols;

	assert(slamtec_depth_buffer.size() == nRows * nCols);

	if (cv_depth_frame.isContinuous())
	{
		nCols *= nRows;
		nRows = 1;
	}

	int i, j;
	const int16_t* p;
	float* p_slamtec = &slamtec_depth_buffer[0];

	for (i = 0; i < nRows; ++i)
	{
		p = cv_depth_frame.ptr<int16_t>(i);
		for (j = 0; j < nCols; ++j)
		{
			*p_slamtec++ = *p++ / 1000.0f; // convert to meters
		}
	}
}


// Converts filtered disparity input (16SC1 and 1 pixel = 16) to depth in millimeters
void convertFilteredDisp16SToDepth(cv::Mat& dispDepth, float depthScaleFactor)
{
	assert(dispDepth.type() == CV_16SC1);

	int nRows = dispDepth.rows;
	int nCols = dispDepth.cols;
	
	if (dispDepth.isContinuous())
	{
		nCols *= nRows;
		nRows = 1;
	}

	int i, j;

	for (i = 0; i < nRows; ++i)
	{
		int16_t* p = dispDepth.ptr<int16_t>(i);

		for (j = 0; j < nCols; ++j, ++p)
		{
			int16_t shiftedP = *p >> 4;
			if (shiftedP != 0)
			{
				*p = max(static_cast<int16_t>(depthScaleFactor / shiftedP), short(0));
			}
			else
			{
				*p = 0;
			}
			
		}
	}
}

float deg2rad(float deg)
{
	return deg * (float)M_PI / 180.f;
}

int main(int argc, char* argv[])
{
	while (1)
	{
		try
		{
#ifdef ENABLE_SLAMTEC_PUBLISHING
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

			auto left = p.create<dai::node::MonoCamera>();
			left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
			left->setBoardSocket(dai::CameraBoardSocket::LEFT);
			//left->initialControl.setManualFocus(130);

			auto right = p.create<dai::node::MonoCamera>();
			right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
			right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
			//right->initialControl.setManualFocus(130);

			auto stereo = p.create<dai::node::StereoDepth>();
			stereo->setConfidenceThreshold(DEPTH_CONFIDENCE_THRESHOLD);
			//stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::KERNEL_3x3);
			stereo->setLeftRightCheck(lr_check);
			stereo->setExtendedDisparity(extended_disparity);
			stereo->setSubpixel(subpixel);
			stereo->setRectifyEdgeFillColor(0); // Black, to better see the cutout from rectification (black stripe on the edges)

			left->out.link(stereo->left);
			right->out.link(stereo->right);
			
			auto rectifiedLeftOut = p.create<dai::node::XLinkOut>();
			rectifiedLeftOut->setStreamName("rectifiedLeft");
			stereo->rectifiedLeft.link(rectifiedLeftOut->input);

			auto disparityOut = p.create<dai::node::XLinkOut>();
			disparityOut->setStreamName("disparity");
			stereo->disparity.link(disparityOut->input);

			std::tuple<bool, dai::DeviceInfo> devInfo = dai::Device::getDeviceByMxId(DEPTH_DEVICE_ID);
			bool found = std::get<0>(devInfo);
			std::cerr << "device " << DEPTH_DEVICE_ID << ", found = " << found << endl;
			if (!found)
			{
				std::cerr << "Cannot find obstacle avoidance depth camera." << endl;
				return 1;
			}

			float wlsLambda = dWlsLambda;
			float wlsSigma = dWlsSigma;

			// Connect to device and start pipeline
			dai::Device d(p, std::get<1>(devInfo));

			std::shared_ptr<dai::DataOutputQueue> qLeft = d.getOutputQueue("rectifiedLeft", 4, false);
			std::shared_ptr<dai::DataOutputQueue> qDisparity = d.getOutputQueue("disparity", 4, false);

			cv::Mat disparityFrame, depthFrameColor(400, 640, CV_8UC1), disparity16SFrame(400, 640, CV_16SC1), filteredDispDepth16S(400, 640, CV_16SC1),
				halfFilteredDispDepth16S(200, 320, CV_16SC1);
			cv::Mat rectifiedLeftFrame;

			class WlsFilter 
			{
				
			public: 
				const cv::String wlsStream = "wlsFilter";

				WlsFilter(float lambda, float sigma)
					: mLambda(lambda), mSigma(sigma)
				{
					mWlsFilter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
					if (!mWlsFilter)
					{
						throw exception("wls_filter is null");
					}

					cv::namedWindow(wlsStream);
					cv::createTrackbar("Lambda", wlsStream, nullptr, 255, &WlsFilter::onTrackbarChangeLambda, this);
					cv::createTrackbar("Sigma", wlsStream, nullptr, 100, &WlsFilter::onTrackbarChangeSigma, this);
					cv::setTrackbarPos("Lambda", wlsStream, 80);
					cv::setTrackbarPos("Sigma", wlsStream, 15);
				}

				void filter(const cv::Mat& disparity, const cv::Mat& left, cv::Mat& filteredDisparity)
				{
					mWlsFilter->setLambda(mLambda);
					mWlsFilter->setSigmaColor(mSigma);
					mWlsFilter->filter(disparity, left, filteredDisparity);
				}

			private:

				static void onTrackbarChangeLambda(int pos, void* ptr)
				{
					WlsFilter* f = static_cast<WlsFilter*>(ptr);
					f->mLambda = pos * 100.;
				}

				static void onTrackbarChangeSigma(int pos, void* ptr)
				{
					WlsFilter* f = static_cast<WlsFilter*>(ptr);
					f->mSigma = pos / 10.f;
				}

				float mLambda;
				float mSigma;
				cv::Ptr<cv::ximgproc::DisparityWLSFilter> mWlsFilter;
			};

			WlsFilter wlsFilter(8000, 1.5f);

			while (1)
			{
				auto inLeft = qLeft.get();
				auto inDisparity = qDisparity.get();
				
				disparityFrame = inDisparity->get<dai::ImgFrame>()->getCvFrame();
				rectifiedLeftFrame = inLeft->get<dai::ImgFrame>()->getCvFrame();
				cv::flip(rectifiedLeftFrame, rectifiedLeftFrame, 1); // flip horizontally

				float focal = disparityFrame.size().width / (2.f * tan(deg2rad(hfov / 2.0f)));
				float depthScaleFactor = baseline * focal;
				disparityFrame.convertTo(disparity16SFrame, CV_16SC1, 16); // filter expects 1 pixel = 16

				// apply the wls filter
				wlsFilter.filter(disparity16SFrame, rectifiedLeftFrame, filteredDispDepth16S);	

				cv::Mat filteredDisp;
				filteredDispDepth16S.convertTo(filteredDisp, CV_8UC1, 255.0 / (disp_levels - 1) / 16.0);
				cv::applyColorMap(filteredDisp, filteredDisp, cv::COLORMAP_JET);
				cv::imshow("wls colored disp", filteredDisp);

				// Compute Depth from disparity
				convertFilteredDisp16SToDepth(filteredDispDepth16S, depthScaleFactor);

				// resize by half for Slamtec suggested size of 320 width
				cv::resize(filteredDispDepth16S, halfFilteredDispDepth16S, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
				cv::imshow("raw depth", halfFilteredDispDepth16S);

				updateSlamtecBuffer(slamtecDepthFrame.data, halfFilteredDispDepth16S);

				cv::normalize(filteredDispDepth16S, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
				cv::equalizeHist(depthFrameColor, depthFrameColor);
				cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);
				cv::imshow(wlsFilter.wlsStream, depthFrameColor);
	
				int sensorId = 3; //TODO: Config your own SensorId

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
#ifdef ENABLE_SLAMTEC_PUBLISHING
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