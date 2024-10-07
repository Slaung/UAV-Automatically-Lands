#! /usr/bin/env python2

import rospy
import mavros
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf import transformations
import mavros_msgs.msg

class ObjectDetection:
    """ 
        Subscribe to the camera feed and detect the aruco marker in the scene
    """
    def __init__(self):
        
        video_topic = "roscam/cam/image_raw"
        image_subscriber = rospy.Subscriber(video_topic, Image, self.image_callback)
        self.bridge = CvBridge()

        # Initialize variables
        self.frame = np.zeros((240, 320, 3), np.uint8)
        self.frame_draw = self.frame
        
        # Setup rate
        self.rate = rospy.Rate(30)
        self.rate.sleep()

    def image_callback(self, data):
        """
            Save the image data everytime a new frame comes up
        """
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
