#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from receive_image_listener import RGB_image_callback

VERBOSE=True

# class image_feature:

#     def __init__(self):
#         '''Initialize ros publisher, ros subscriber'''
        
#         # subscribed Topic THERMO IMAGES
#         # self.subscriber_THERMO = rospy.Subscriber("/camera_vision_output/compressed",
#         #     CompressedImage, self.Thermo_image_callback,  queue_size = 1)
        
# 	self.subscriber_RGB = rospy.Subscriber("/camera_vision_RGB_output/compressed",
#             CompressedImage, self.RGB_image_callback,  queue_size = 1)

#     def Thermo_image_callback(self, ros_data):
#         '''Callback function of subscribed topic. 
#         Here images get converted and features detected'''
#         if VERBOSE :
#             print 'received THERMO image  of type: "%s"' % ros_data.format

#         #### direct conversion to CV2 ####
#         np_arr = np.fromstring(ros_data.data, np.uint8)
#         thermo_image_np = cv2.imdecode(np_arr, cv2.IMREAD_ANYCOLOR)
#         #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

# 	cv2.imshow("Thermo image", thermo_image_np)
#         cv2.waitKey(1)
	

#     def RGB_image_callback(self, RGB_data):
#         '''Callback function of subscribed topic. 
#         Here images get converted and features detected'''
#         if VERBOSE :
#             print 'received RGB image of type: "%s"' % RGB_data.format

#         #### direct conversion to CV2 ####
#         np_arr = np.fromstring(RGB_data.data, np.uint8)
#         RGB_image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#         #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

# 	cv2.imshow("RGB image", RGB_image_np)
#         cv2.waitKey(1)

   
def obtain_THERMO_images():
    subscriber_THERMO = rospy.Subscriber("/camera_vision_output/compressed",
           CompressedImage, Thermo_image_callback,  queue_size = 1)
  
    #RGB_image = RGB_image_callback()
   

def obtain_RGB_images():
   
    subscriber_RGB = rospy.Subscriber("/camera_vision_RGB_output/compressed",
             CompressedImage, RGB_image_callback,  queue_size = 1)

def Thermo_image_callback(ros_data):
    '''Callback function of subscribed topic. 
    Here images get converted and features detected'''
    if VERBOSE :
        print 'received THERMO image  of type: "%s"' % ros_data.format    
    #### direct conversion to CV2 ####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    thermo_image_np = cv2.imdecode(np_arr, cv2.IMREAD_ANYCOLOR)
    #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:    
    cv2.imshow("Thermo image", thermo_image_np)
    cv2.waitKey(1)


def RGB_image_callback(RGB_data):
    '''Callback function of subscribed topic. 
    Here images get converted and features detected'''
    if VERBOSE :
        print 'received RGB image of type: "%s"' % RGB_data.format    
    #### direct conversion to CV2 ####
    np_arr = np.fromstring(RGB_data.data, np.uint8)
    RGB_image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:    
    cv2.imshow("RGB image", RGB_image_np)
    cv2.waitKey(1)


def main(args):
    '''Initializes and cleanup ros node'''
   
    rospy.init_node('receive_compressed_images', anonymous=True)
    condition = 100000000
    counter = 0
    #while (counter < condition):

    try:
        obtain_THERMO_images()
	obtain_RGB_images()
        rospy.spin()
       
    except KeyboardInterrupt:
        print "Shutting down ROS receive compressed images detector module"
        cv2.destroyAllWindows()
 
if __name__ == '__main__':
      main(sys.argv)