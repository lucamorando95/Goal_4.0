#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Imu, Image, CompressedImage
#from keras.layers.core import Dense, Dropout, Activation, Flatten
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2



def RGB_image_callback():
    camera_frame = None
    bridge = CvBridge()
    while camera_frame is None :
        try:
            # camera_frame = rospy.wait_for_message('/ardrone/bottom/image_raw', Image, timeout = 1)
            camera_frame = rospy.wait_for_message('/camera_vision_RGB_output/compressed', CompressedImage, timeout = 1)
        except:
             rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
        
	RGB_image_np = bridge.compressed_imgmsg_to_cv2(camera_frame, "bgr8")
	# np_arr = np.fromstring(camera_frame, np.uint8)
        # RGB_image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0: 
    return RGB_image_np
