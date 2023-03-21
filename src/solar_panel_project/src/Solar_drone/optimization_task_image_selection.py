#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 15:18:34 2020

@author: lucamora
"""

import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image, CompressedImage
from ros_simulation_data import take_drone_RGB_clustered_image, take_drone_THERMO_clustered_image

VERBOSE = True

class RGB_image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
       
        # subscribed Topic
        
        self.subscriber_RGB = rospy.Subscriber("/dji_osdk_ros/OPT_RGB_IMG",
            CompressedImage, self.callback,  queue_size = 1)
        
      


        self.image_opt = []
        self.clustered_image_manual  = []
        self.counter = 0
        if VERBOSE :
            print "subscribed to /dji_osdk_ros/OPT_RGB_IMG"

    

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format
        
       
       #SUBSCRIBER TO THERMAL IMAGES 
        subscriber_THERMO = rospy.Subscriber("/dji_osdk_ros/OPT_THERMO_IMG",
            CompressedImage, self.callback_THERMO,  queue_size = 1)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.image_opt = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.clustered_image_manual = take_drone_RGB_clustered_image()
        
        
        # concatanate image Horizontally
        Hori = np.concatenate((  self.clustered_image_manual, self.image_opt), axis=1)
        
        cv2.imshow('1.  Manual Image   2.Optimized Image ', Hori)

          
        cv2.waitKey(1)
           


        self.counter = self.counter + 1 





# class  THERMO_image_feature:
#     def __init__(self):
#         '''Initialize ros publisher, ros subscriber'''
       
#         # subscribed Topic

        
#         self.subscriber_THERMO = rospy.Subscriber("/dji_osdk_ros/OPT_THERMO_IMG",
#             CompressedImage, self.callback_THERMO,  queue_size = 1)

#         self.counter = 0
#         if VERBOSE :
#             print "subscribed to /dji_osdk_ros/OPT_THERMO_IMG"



    def callback_THERMO(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received THERMO image of type: "%s"' % ros_data.format
        
       

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_opt = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        clustered_image_manual = take_drone_THERMO_clustered_image()
        
        
        # concatanate image Horizontally
        Hori = np.concatenate(( clustered_image_manual, image_opt), axis=1)
        
        cv2.imshow('1.  Manual Image   2.Optimized Image ', Hori)

          
        cv2.waitKey(1)
           


        self.counter = self.counter + 1 
            

def listener():
    '''Initializes and cleanup ros node'''
    ic = RGB_image_feature()
    # it = THERMO_image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:

        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()



if __name__ == '__main__':
    listener()