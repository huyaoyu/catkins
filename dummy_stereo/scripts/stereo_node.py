#!/usr/bin/env python
from __future__ import print_function

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import cv2

from cv_bridge import CvBridge, CvBridgeError

# PATH_IMAGES = [\
#     "/home/yaoyu/Documents/CMU/AirLab/Weikun/Data_20180501/left_dst_resized/left0000.jpg", \
#     "/home/yaoyu/Documents/CMU/AirLab/Weikun/Data_20180501/right_dst/right0000.jpg" \
# ]

PATH_IMAGES = [\
    "/home/yyhu/Documents/CMU/AirLab/stereo_camera/Data_20180501/left_dst_resized/left0000.jpg", \
    "/home/yyhu/Documents/CMU/AirLab/stereo_camera/Data_20180501/right_dst/right0000.jpg" \
]

class StereoNode(object):
    def __init__(self, name):
        self.name = name

        self.topicPrefix       = "/dummy_stereo"
        self.topicCameraNames  = ["/left", "/right"]
        self.topicImageRaw     = "/image_raw"
        self.topicCameraInfo   = "/camera_info"
        self.topicCameraImages = [\
            self.topicPrefix + self.topicCameraNames[0] + self.topicImageRaw,\
            self.topicPrefix + self.topicCameraNames[1] + self.topicImageRaw \
            ]
        self.topicCameraInfos = [\
            self.topicPrefix + self.topicCameraNames[0] + self.topicCameraInfo,\
            self.topicPrefix + self.topicCameraNames[1] + self.topicCameraInfo \
            ]

        self.cameraImages = []
        self.cameraInfos  = []

        self.cvImages = []
        self.cvImageHeight = 0
        self.cvImageWidth  = 0

        self.bridge = CvBridge()

        self.rosImageMessages = []
        self.rosCameraInfos   = []
        self.rosHeaderFrameId = "dummy_stereo"

        self.rate = 10
        self.topicQueueSize = 1

    def load_images(self, pathToImages):
        for i in range(2):
            p = pathToImages[i]
            self.cvImages.append( cv2.imread(p) )
            self.rosImageMessages.append( self.bridge.cv2_to_imgmsg( self.cvImages[i], "bgr8" ) )

        self.cvImageHeight = self.cvImages[0].shape[0]
        self.cvImageWidth  = self.cvImages[0].shape[1]

        for i in range(2):
            self.rosImageMessages[i].header.frame_id = self.rosHeaderFrameId

    def prepare_camera_infos(self):
        for i in range(2):
            self.rosCameraInfos.append( CameraInfo() )

        for i in range(2):
            self.rosCameraInfos[i].header.frame_id = self.rosHeaderFrameId

    def create_ROS_elements(self):
        self.cameraImages.append( rospy.Publisher( self.topicCameraImages[0], Image, queue_size = self.topicQueueSize ) )
        self.cameraImages.append( rospy.Publisher( self.topicCameraImages[1], Image, queue_size = self.topicQueueSize ) )

        self.cameraInfos.append( rospy.Publisher( self.topicCameraInfos[0], CameraInfo, queue_size = self.topicQueueSize ) )
        self.cameraInfos.append( rospy.Publisher( self.topicCameraInfos[1], CameraInfo, queue_size = self.topicQueueSize ) )

    def spin(self):
        rate = rospy.Rate(self.rate) # 10hz
        try:
            while not rospy.is_shutdown():
                for i in range(2):
                    self.rosImageMessages[i].header.stamp = rospy.Time.now()

                    self.cameraImages[i].publish( self.rosImageMessages[i] )

                    self.rosCameraInfos[i].header.stamp = rospy.Time.now()
                    self.rosCameraInfos[i].height = self.cvImageHeight
                    self.rosCameraInfos[i].width  = self.cvImageWidth

                    self.cameraInfos[i].publish( self.rosCameraInfos[i] )
                    rospy.loginfo("Topics published.")

                rate.sleep()
        except KeyboardInterrupt:
            print("Shutting down.")

if __name__ == '__main__':
    node = StereoNode("dummy_stereo_node")

    rospy.init_node(node.name, anonymous = True)

    node.load_images(PATH_IMAGES)
    node.prepare_camera_infos()
    node.create_ROS_elements()

    node.spin()
