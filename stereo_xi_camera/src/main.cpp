
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
 * Created:  2018-06-04
 * Modified: 2018-06-19
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

  	ros::Rate loop_rate(3);

	// The ROS message to be published.
	sensor_msgs::ImagePtr msgImage;

	// The object of stereo camera based on the XIMEA cameras.
	sxc::StereoXiCamera stereoXiCamera = sxc::StereoXiCamera(XI_CAMERA_SN_0, XI_CAMERA_SN_1);
	// Configure the stereo camera.
	stereoXiCamera.set_autogain_exposure_priority(1.0);
	stereoXiCamera.set_autoexposure_top_limit(200);
	stereoXiCamera.set_total_bandwidth(2400);
	stereoXiCamera.set_bandwidth_margin(10);

	// Run the ROS node.
	try
	{
		// Pre-open, open and configure the stereo camera.
		stereoXiCamera.open();

		// Start acquisition.
		ROS_INFO("%s", "Start acquisition.");
		stereoXiCamera.start_acquisition();

		// Temporary variables.
		Mat cvImages[2];        // OpenCV Mat array to hold the images.
		std::stringstream ss;   // String stream for outputing info.
		ros::Time rosTimeStamp; // ROS time stamp for the header of published ROS image messages.

		int nImages = 0; // Image counter.

		// Begin running ROS node.
		while(ros::ok())
		{
			if ( 0 != N_IMAGES )
			{
				if ( nImages >= N_IMAGES )
				{
					// Stop this ROS node.
					break;
				}
			}

			ROS_INFO("nImages = %d", nImages);

			// Trigger.
			stereoXiCamera.software_trigger();

			// Get images.
			stereoXiCamera.get_images( cvImages[0], cvImages[1] );

			// Prepare the time stamp for the header.
			rosTimeStamp = ros::Time::now();

			// Convert the image into ROS image message.
			LOOP_CAMERAS_BEGIN
				// Clear the temporary sting stream.
				ss.flush();	ss.str(""); ss.clear();
				ss << OUT_DIR << "/" << nImages << "_" << loopIdx << ".jpg";

				ROS_INFO( "%s", ss.str().c_str() );

				// Save the captured image to file system.
				// imwrite(ss.str(), cvImages[loopIdx]);
				ROS_INFO( "Camera %d captured image.", loopIdx );

				// Publish images.
				msgImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImages[loopIdx]).toImageMsg();
				msgImage->header.seq   = nImages;
				msgImage->header.stamp = rosTimeStamp;

				publishersImage[loopIdx].publish(msgImage);

				ROS_INFO("%s", "Message published.");
			LOOP_CAMERAS_END

			// ROS spin.
			ros::spinOnce();

			// Sleep().
			loop_rate.sleep();

			++nImages;
		}

		// Stop acquisition.
		stereoXiCamera.stop_acquisition();

		cvWaitKey(500);

		ROS_INFO("Stereo camera stopped.");

		// Close.
		stereoXiCamera.close();
		ROS_INFO("Stereo camera closed.");
	}
	catch ( xiAPIplus_Exception& exp )
	{
		ROS_ERROR("Error.");
		exp.PrintError();
		ret = -1;
	}

	return ret;
}
