#!/usr/bin/env python

from __future__ import print_function

import copy

import numpy as np

# ROS
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# ===================== Constants. ===========================

TOPIC_SUBSCRIBED = "/camera/image_raw"

TOPIC_IMAGE_RAW_LEFT  = "lenacv_camera/left/image_raw"
TOPIC_IMAGE_RAW_RIGHT = "lenacv_camera/right/image_raw"

TOPIC_CAMERA_INFO_LEFT  = "lenacv_camera/left/camera_info"
TOPIC_CAMERA_INFO_RIGHT = "lenacv_camera/right/camera_info"

FRAME_ID = "lenacv"

ROS_NODE_NAME = "listen_uvc"

# ===================== Functions. ============================

def get_default_camera_info_msg(imageSize = [1280, 720], frame_id = "FI"):
    tempCameraInfoMsg = CameraInfo()
    tempCameraInfoMsg.height = imageSize[1]
    tempCameraInfoMsg.width  = imageSize[0]
    tempCameraInfoMsg.distortion_model = ""
    tempCameraInfoMsg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    tempCameraInfoMsg.K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    tempCameraInfoMsg.R = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    tempCameraInfoMsg.P = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    tempCameraInfoMsg.binning_x = 0
    tempCameraInfoMsg.binning_y = 0
    tempCameraInfoMsg.roi.x_offset   = 0
    tempCameraInfoMsg.roi.y_offset   = 0
    tempCameraInfoMsg.roi.height     = 0
    tempCameraInfoMsg.roi.width      = 0
    tempCameraInfoMsg.roi.do_rectify = False
    tempCameraInfoMsg.header.frame_id = frame_id

    return tempCameraInfoMsg

# ==================== Classes. ===========================

class Separator:

    def __init__(self):
        self.publisherImageRaw = []
        self.publisherImageRaw.append( rospy.Publisher(TOPIC_IMAGE_RAW_LEFT,  Image, queue_size = 1) )
        self.publisherImageRaw.append( rospy.Publisher(TOPIC_IMAGE_RAW_RIGHT, Image, queue_size = 1) )

        self.publisherCameraInfo = []
        self.publisherCameraInfo.append( rospy.Publisher(TOPIC_CAMERA_INFO_LEFT,  CameraInfo, queue_size = 1) )
        self.publisherCameraInfo.append( rospy.Publisher(TOPIC_CAMERA_INFO_RIGHT, CameraInfo, queue_size = 1) )

        tempCameraInfoMsg = get_default_camera_info_msg([1280, 720], FRAME_ID)

        self.cameraInfoMsgs = [ tempCameraInfoMsg, copy.deepcopy(tempCameraInfoMsg) ]

        self.cvBridge = CvBridge()

        self.subsciber = rospy.Subscriber(TOPIC_SUBSCRIBED, Image, self.callback_subscriber)

        self.width  = 2560
        self.height = 720
        self.half   = self.width / 2

        self.cvImgs = []

        self.msgCount = 0

    def load_camera_info(self, filename, cameraInfo):
        """Load a yaml file as the camera info."""

        import yaml

        # Open the file.
        fp = open( filename, "r" )

        # Parse the yaml file.
        ci = yaml.safe_load(fp)

        # Copy the information into cameraInfo.
        cameraInfo.height = ci["image_height"]
        cameraInfo.width  = ci["image_width"]
        cameraInfo.distortion_model = ci["distortion_model"]
        cameraInfo.K = ci["camera_matrix"]["data"]
        cameraInfo.D = ci["distortion_coefficients"]["data"]
        cameraInfo.R = ci["rectification_matrix"]["data"]
        cameraInfo.P = ci["projection_matrix"]["data"]

    def callback_subscriber(self, data):
        # rospy.loginfo(rospy.get_caller_id() + " Message received.")

        self.cvImgs = []
        self.msgImages = []

        try:
            # Convert your ROS Image message to OpenCV2
            cvImg = self.cvBridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Separate the image into two images.
            self.cvImgs.append( cvImg[:, self.half:, :] )
            self.cvImgs.append( cvImg[:, :self.half, :] )

            timeStamp = rospy.Time.now()

            for i in range(2):
                try:
                    msgImage = self.cvBridge.cv2_to_imgmsg(self.cvImgs[i], "bgr8")
                    msgImage.header.seq = self.msgCount
                    msgImage.header.stamp = timeStamp

                    self.publisherImageRaw[i].publish( msgImage )

                    self.cameraInfoMsgs[i].header.seq = self.msgCount
                    self.cameraInfoMsgs[i].header.stamp = timeStamp
                    self.publisherCameraInfo[i].publish( self.cameraInfoMsgs[i] )
                except rospy.ROSException, ex:
                    # At the very first stages, ROS will complain about not initialized node.
                    rospy.loginfo(rospy.get_caller_id() + ex.message)

            self.msgCount += 1

            rospy.loginfo(rospy.get_caller_id() + " Data converted.")

# =================== main. ==========================

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, anonymous = True)
    
    S = Separator()

    S.load_camera_info(\
        "/home/yyhu/Documents/CMU/AirLab/Zeng/Lenacv/Calibration/calibrationdata_20180528/left.yaml", \
        S.cameraInfoMsgs[0])
        
    S.load_camera_info(\
        "/home/yyhu/Documents/CMU/AirLab/Zeng/Lenacv/Calibration/calibrationdata_20180528/right.yaml", \
        S.cameraInfoMsgs[1])

    rospy.spin()
