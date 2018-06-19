
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
#include "StereoXiCamera.hpp"

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

const int N_IMAGES = 0;

const std::string OUT_DIR = "/home/yyhu/ROS/p2/catkin/src/stereo_xi_camera/output";

std::string XI_CAMERA_SN_1 = "CUCAU1814018";
std::string XI_CAMERA_SN_0 = "CUCAU1814020";

// =============== main(). =========================

int main(int argc, char* argv[])
{
	int ret = 0;

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle;

	image_transport::ImageTransport imageTransport(nodeHandle);

	image_transport::Publisher publishersImage[2] = { 
		imageTransport.advertise(TOPIC_NAME_LEFT_IMAGE, 1), 
		imageTransport.advertise(TOPIC_NAME_RIGHT_IMAGE, 1) };

  	ros::Rate loop_rate(2);

	sensor_msgs::ImagePtr msgImage;

	sxc::StereoXiCamera stereoXiCamera = sxc::StereoXiCamera(XI_CAMERA_SN_0, XI_CAMERA_SN_1);

	try
	{
		stereoXiCamera.open();

		// Start acquisition.
		std::cout << "Start acquisition." << std::endl;

		stereoXiCamera.start_acquisition();

		Mat cvImages[2];
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
			stereoXiCamera.software_trigger();

			// Get images.
			stereoXiCamera.get_images( cvImages[0], cvImages[1] );

			rosTimeStamp = ros::Time::now();

			LOOP_CAMERAS_BEGIN
				ss.flush();
				ss.str("");
				ss.clear();

				ss << OUT_DIR << "/" << nImages << "_" << loopIdx << ".jpg";

				// Save the captured image to file system.
				// imwrite(ss.str(), cvImages[loopIdx]);

				std::cout << "Camera " << loopIdx << " captured image." << std::endl;

				// Publish images.
				msgImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImages[loopIdx]).toImageMsg();
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
		stereoXiCamera.stop_acquisition();

		std::cout << "Begin waiting..." << std::endl;
		cvWaitKey(500);

		// Close.
		stereoXiCamera.close();
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
