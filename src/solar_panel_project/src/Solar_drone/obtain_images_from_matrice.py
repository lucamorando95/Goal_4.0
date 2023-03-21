
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import sys

from scipy import ndimage
import cv2
import time
import os
import math

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


##################### SUBSCRIBE TO RGB IMAGES 
def subscribe_HSV():
    camera_frame = None
    while camera_frame is None :
        try:
            # camera_frame = rospy.wait_for_message('/ardrone/bottom/image_raw', Image, timeout = 1)
            camera_frame = rospy.wait_for_message('/OVTA_DEBUG_HSV_image', Image, timeout = 1)  # 
           
        except:
            rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough') 
          
    return cv_image


def subscribe_First_threshold_image():
    camera_frame = None
   
    while camera_frame is None :
        try:
            # camera_frame = rospy.wait_for_message('/ardrone/bottom/image_raw', Image, timeout = 1)
            camera_frame = rospy.wait_for_message('/OVTA_DEBUG_FIRST_THRESHOLD_image', Image, timeout = 1)
            
        except:
             rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')     
    return cv_image

def subscribe_cluster_image():
    camera_frame = None
   
    while camera_frame is None :
        try:
            # camera_frame = rospy.wait_for_message('/ardrone/bottom/image_raw', Image, timeout = 1)
            camera_frame = rospy.wait_for_message('/OVTA_DEBUG_CLUSTERING_image', Image, timeout = 1)
            
        except:
             rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')     
    return cv_image


########################### SUBSCRIBE TO THERMSL IMAGES
def subscribe_thermal_GRAY_image():
    camera_frame = None
   
    while camera_frame is None :
        try:
            # camera_frame = rospy.wait_for_message('/ardrone/bottom/image_raw', Image, timeout = 1)
            camera_frame = rospy.wait_for_message('/THERMAL_GRAY', Image, timeout = 1)
            
        except:
             rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')     
    return cv_image

def subscribe_thermal_threshold_image():
    camera_frame = None
   
    while camera_frame is None :
        try:
            # camera_frame = rospy.wait_for_message('/ardrone/bottom/image_raw', Image, timeout = 1)
            camera_frame = rospy.wait_for_message('/THERMAL_TH', Image, timeout = 1)
            
        except:
             rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')     
    return cv_image

def subscribe_thermal_distance_matrix_image():
    camera_frame = None
   
    while camera_frame is None :
        try:
            # camera_frame = rospy.wait_for_message('/ardrone/bottom/image_raw', Image, timeout = 1)
            camera_frame = rospy.wait_for_message('/THERMAL_DISTANCE', Image, timeout = 1)
            
        except:
             rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')     
    return cv_image


def listener():
    rospy.init_node('termo_frame_elaboration', anonymous=True)
    #Create Image Publisher
    count = 0
    condition = 100000000
    RGB = True
    Thermo = False
    r = rospy.Rate(10) # 10hz 
    while not rospy.is_shutdown():
        if RGB:
           HSV =subscribe_HSV()
       
       
           FT_image = subscribe_First_threshold_image()
           
           cluster_image = subscribe_cluster_image()
           
           cv2.imshow("HSV matrice", HSV)
           cv2.imshow("FT Matrice", FT_image)
           cv2.imshow("cluster Matrice", cluster_image)
           cv2.waitKey(1)
        
        #SHOW THERMAL IMAGES FOR DEBUGGING
        if Thermo:
           #thermal_gray =subscribe_thermal_GRAY_image()
           TH_image = subscribe_thermal_threshold_image()
           
           distance_Matrix_image = subscribe_thermal_distance_matrix_image()
           
           #cv2.imshow("GRAY THERMAL matrice", thermal_gray)
           cv2.imshow("FT THERMAL Matrice", TH_image)
           cv2.imshow("DISTANCE MATRIX Matrice", distance_Matrix_image)
           cv2.waitKey(2)
        
           

        print("SHOWING IMAGES")
        count = count + 1
        r.sleep()
       

   





if __name__ == '__main__':
    listener()
   # file1.close()
