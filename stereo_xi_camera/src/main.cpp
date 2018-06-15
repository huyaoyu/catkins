
/**
 * Test code for the stereo camera built on the xiC models produced by XIMEA.
 *
 * Author
 * ======
 *
 * Yaoyu Hu <yyhu_live@outlook.com>
 *
 * Date
 * ====
 *
 * Created: 2018-06-04
 *
 */

#include <iostream>
#include <string>
#include <sstream>

#include <stdio.h>
#include "xiApiPlusOcv.hpp"

// ========= Includes for ROS and OpenCV. ===================

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

// ============ Static global variables. ==========

const char* NODE_NAME = "xic_stereo";
const char* TOPIC_NAME_LEFT_IMAGE  = "xic_stereo/left/image_raw";
const char* TOPIC_NAME_RIGHT_IMAGE = "xic_stereo/right/image_raw";

const int TRIGGER_SOFTWARE       = 1;
const int EXPOSURE_MILLISEC_BASE = 1000;
const int N_IMAGES               = 0;

const std::string OUT_DIR = "/home/yyhu/ROS/p2/catkin/src/stereo_xi_camera/output";

int EXPOSURE_MILLISEC(int val)
{
	return val * EXPOSURE_MILLISEC_BASE;
}

char* XI_CAMERA_SN_1 = "CUCAU1814018";
char* XI_CAMERA_SN_0 = "CUCAU1814020";

const int CAM_IDX_0 = 0;
const int CAM_IDX_1 = 1;

// =================== Macros. ==========================

#define LOOP_CAMERAS_BEGIN \
	for( int loopIdx = 0; loopIdx < 2; loopIdx++ )\
	{

#define LOOP_CAMERAS_END \
	}

// ================ Local functions. =====================

int setup_camera_common(xiAPIplusCameraOcv& cam)
{
	int sta = 0;

	// Set exposure time.
//	cam.SetExposureTime( EXPOSURE_MILLISEC(100) );
	cam.SetAutoExposureAutoGainExposurePriority(1.0);
	cam.SetAutoExposureTopLimit( EXPOSURE_MILLISEC(100) );
	cam.EnableAutoExposureAutoGain();

	// Enable auto-whitebalance.
	cam.EnableWhiteBalanceAuto();

	// Image format.
	cam.SetImageDataFormat(XI_RGB24);

	// Sensor defects selector.
	cam.SetSensorDefectsCorrectionListSelector(XI_SENS_DEFFECTS_CORR_LIST_SEL_USER0);
	cam.EnableSensorDefectsCorrection();

	return sta;
}

// =============== main(). =========================

int main(int argc, char* argv[])
{
	int ret = 0;

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle;

	image_transport::ImageTransport imageTransport(nodeHandle);

	// ros::Publisher publishersImage[2] = {
	// 	nodeHandle.advertise<std_msgs::String>(TOPIC_NAME_LEFT_IMAGE, 1000),
	// 	nodeHandle.advertise<std_msgs::String>(TOPIC_NAME_RIGHT_IMAGE, 1000) };

	image_transport::Publisher publishersImage[2] = { 
		imageTransport.advertise(TOPIC_NAME_LEFT_IMAGE, 1), 
		imageTransport.advertise(TOPIC_NAME_RIGHT_IMAGE, 1) };

  	ros::Rate loop_rate(5);

	sensor_msgs::ImagePtr msgImage;

	xiAPIplusCameraOcv cams[2];

	try
	{
		// Open the first camera for tuning.
		cams[CAM_IDX_0].OpenBySN(XI_CAMERA_SN_0);
		cams[CAM_IDX_0].SetExposureTime( EXPOSURE_MILLISEC(100) );

		// Open the second camera for tuning.
		cams[CAM_IDX_1].OpenBySN(XI_CAMERA_SN_1);
		cams[CAM_IDX_1].SetExposureTime( EXPOSURE_MILLISEC(100) );

		// Configure common parameters.
		LOOP_CAMERAS_BEGIN
			setup_camera_common(cams[loopIdx]);
		LOOP_CAMERAS_END

		// Configure synchronization.
		// Set trigger mode on the first camera - as master.
		cams[CAM_IDX_0].SetTriggerSource(XI_TRG_SOFTWARE);
		cams[CAM_IDX_0].SetGPOSelector(XI_GPO_PORT1);
		cams[CAM_IDX_0].SetGPOMode(XI_GPO_EXPOSURE_ACTIVE);

		// Set trigger mode on the second camera - as slave.
		cams[CAM_IDX_1].SetGPISelector(XI_GPI_PORT1);
		cams[CAM_IDX_1].SetGPIMode(XI_GPI_TRIGGER);
		cams[CAM_IDX_1].SetTriggerSource(XI_TRG_EDGE_RISING);

		// Start acquisition.
		std::cout << "Start acquisition." << std::endl;

		LOOP_CAMERAS_BEGIN
			cams[loopIdx].StartAcquisition();
		LOOP_CAMERAS_END

		// Wait for a short period of time.
		cvWaitKey(500);

		// Obtain the images.
		XI_IMG_FORMAT format;
		Mat cv_mat_image;
		std::stringstream ss;

		ros::Time rosTimeStamp;

		int nImages = 0;

		while(ros::ok())
		{
			if ( 0 != N_IMAGES )
			{
				if ( nImages >= N_IMAGES )
				{
					break;
				}
			}

			std::cout << "nImages = " << nImages << std::endl;

			// Trigger.
			cams[CAM_IDX_0].SetTriggerSoftware(TRIGGER_SOFTWARE);

			rosTimeStamp = ros::Time::now();

			LOOP_CAMERAS_BEGIN
				ss.flush();
				ss.str("");
				ss.clear();

				ss << OUT_DIR << "/" << nImages << "_" << loopIdx << ".jpg";

				format = cams[loopIdx].GetImageDataFormat();
				cv_mat_image = cams[loopIdx].GetNextImageOcvMat();
				std::cout << "format = " << format << std::endl;
				if (format == XI_RAW16 || format == XI_MONO16)
				{
					normalize(cv_mat_image, cv_mat_image, 0, 65536, NORM_MINMAX, -1, Mat()); // 0 - 65536, 16 bit unsigned integer range
				}

				// Save the captured image to file system.
				// imwrite(ss.str(), cv_mat_image);

				std::cout << "Camera " << loopIdx << " captured image." << std::endl;

				// cvWaitKey(100);

				// Publish images.
				msgImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_mat_image).toImageMsg();
				msgImage->header.seq = nImages;
				msgImage->header.stamp = rosTimeStamp;

				publishersImage[loopIdx].publish(msgImage);
				ROS_INFO("%s", "Message published.");
			LOOP_CAMERAS_END

			ros::spinOnce();

			loop_rate.sleep();
			++nImages;
		}

		// Stop acquisition.
		LOOP_CAMERAS_BEGIN
			cams[loopIdx].StopAcquisition();
		LOOP_CAMERAS_END

		std::cout << "Begin waiting..." << std::endl;
		cvWaitKey(500);

		cams[CAM_IDX_1].Close();
		cams[CAM_IDX_0].Close();
		std::cout << "Done." << std::endl;
	}
	catch ( xiAPIplus_Exception& exp )
	{
		std::cout << "Error:" << std::endl;
		exp.PrintError();
		ret = -1;
	}

	return ret;
}
