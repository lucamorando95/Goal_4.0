#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point
import rospy
import roslib
import cv2
from cv_bridge import CvBridge, CvBridgeError
from scipy.ndimage import filters
import numpy as np
import logging
import threading
import matplotlib.pyplot as plt
__author__ = 'Luca Morando'
__version__ = '0.1'
__license__ = 'BSD'

# Python libs
import sys
import os
import time
import keyboard

# Lo script deve essere runnato come amministratore per prendere input da tastiera
VERBOSE = False


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        # subscribed RGB Topic
        self.image_subscriber = rospy.Subscriber("/OVTA_camera_vision_RGB_output/compressed",
                                           CompressedImage, self.RGB_image_callback,  queue_size=1)
                                   
        self.image_HSV_subscriber = rospy.Subscriber("/OVTA_DEBUG_HSV_image",
                                           Image, self.RGB_HSV_image_callback,  queue_size=1)
        
        self.image_FT_RGB_subscriber = rospy.Subscriber("/OVTA_DEBUG_FIRST_THRESHOLD_image",
                                           Image, self.RGB_FT_image_callback,  queue_size=1)
        
        # self.image_cluster_RGB_subscriber = rospy.Subscriber("/OVTA_DEBUG_HSV_image",
        #                                    Image, self.RGB_cluster_image_callback,  queue_size=1)






        self.record_obs_point1_sub = rospy.Subscriber("/RGB_control_point_1",
                                           Point, self.obs_point1_rec,  queue_size=1)
        self.record_obs_point2_sub = rospy.Subscriber("/RGB_control_point_2",
                                           Point, self.obs_point2_rec,  queue_size=1)

        self.flag_record_start = False
        self.flag_thread_start = False
        # Initialize List to store the data Points collected
        self.point1_list = []
        self.point2_list = []
        
        #IMAGE HSV
        self.image_elaborated = 0
        self.image_HSV = 0
        self.image_FT = 0

        self.i = 0
        self.num = 1

        if VERBOSE:
            print "subscribed to /OVTA_camera_vision_RGB_output/compressed"

    def plot_data(self):
       
        #### Function to plot the data ####
        #print("self.point1_list:", self.point1_list)
        

        t1 = np.arange(0.0, len(self.point1_list),1) #Creo array lunghezza point1_list relativo al numero di campioni raccolti da dipsorre su asse x con distamza 1
        
        f = plt.figure(1)
        plt.plot(t1, self.point1_list) #Plotta x ed y 
       
        plt.ylabel('meters')
        plt.xlabel('samples')

        plt.axvline(x=30, ymin=0, ymax=1, color='g', linestyle='--', label='y_value')
        plt.axvline(x=60, ymin=0, ymax=1, color='b', linestyle='--', label='x_value')
        plt.legend()
        #plt.xlim(0, 1)
        plt.ylim(-1, 1)
        f.show()
       
       #Plot Point 2
        t2 = np.arange(0.0, len(self.point2_list),1)
        g = plt.figure(2)
        plt.plot(t2, self.point2_list) #Plotta x ed y 
       
        plt.ylabel('meters')
        plt.xlabel('samples')

        plt.axvline(x=30, ymin=0, ymax=1, color='g', linestyle='--', label='y_value')
        plt.axvline(x=60, ymin=0, ymax=1, color='b', linestyle='--', label='x_value')
        plt.legend()
        #plt.xlim(0, 1)
        g.show() 

        #Save data acquired in a txt fileime
        # Create a directory to save data  an every tie an experiment is complete
        numm = self.num
        path = "/home/lucamora/catkin_ws/data_acquired_from_lab_experiments/exp_" + str(numm)
        
        
       
        

        try:
            os.mkdir(path)
        except OSError:
            print ("Creation of the directory %s failed" % path)
        else:
            print ("Successfully created the directory %s " % path)

        file1 = open(path + "/Point1.txt","w")
        
        for ii in range(0, len(self.point1_list)):
             file1.write(str( self.point1_list[ii][0]) + ","+ " " + str( self.point1_list[ii][1]) + "\n") 
             file1.flush()
        
        file1.close()
        file2 = open(path + "/Point2.txt","w")
        
        for ii in range(0, len(self.point2_list)):
             file2.write(str( self.point2_list[ii][0]) + ","+ " " + str( self.point2_list[ii][1]) + "\n") 
             file2.flush()

        self.num = self.num + 1

########################  THREAD FOR DETECTing KEYBOARD INPUT ############################
    def call_thread_key(self):
        if self.flag_thread_start == False:
            print("Start Key Pression Detection Thread")
            self.flag_thread_start = True
            t = threading.Thread(target=self.thread_detect_key_press_function(), args=(1,))
            t.start()
       
            

  
    def thread_detect_key_press_function(self):
        print("Thread Started")
        while True:
           
            if keyboard.is_pressed('s'):  # if key 's' is pressed 
                print('Starting  Record Observation Point1 and Point2 from camera images !')
                self.flag_record_start = True
            
            if keyboard.is_pressed('q')  and self.flag_record_start == True:
                self.flag_record_start = False
                self.flag_thread_start = False
                #self.plot_data()
                print('Stopping  Record Observation Point1 and Point2 from camera images !')
                
            if keyboard.is_pressed('c'):
                cv2.destroyAllWindows()
                break

    def display_images(self):
        cv2.imshow(' RGB elaborated Matrice', self.image_elaborated)
        
        
        cv2.imshow('HSV RGB Matrice', self.image_HSV) 
        

        cv2.imshow('FT RGB Matrice', self.image_FT)
        cv2.waitKey(1)

    ######################## ############################## ############################

    # ros_data è l'immagine compressa ricevuta
    def RGB_image_callback(self, ros_data):
        if VERBOSE:
            print 'received image of type: "%s"' % ros_data.format

    #### direct conversion to CV2 per compressed images####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image_elaborated = image_np

        self.display_images()
        #cv2.imshow('RGB_stream_of_the_elaborated_image_from_Matrice', image_np)
       # cv2.waitKey(2)

        # Start a thread to detect if a key is pressed 

    # If it is required to republish a compressed image in python
    #### Create CompressedIamge ####
   # msg = CompressedImage()
    # msg.header.stamp = rospy.Time.now()
    # msg.format = "jpeg"
    # msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    # # Publish new image
    # self.image_pub.publish(msg)

    # def RGB_HSV_image_callback(self, ros_data):
    #     camera_frame = None
    #     bridge = CvBridge()
    #     cv_image = bridge.imgmsg_to_cv2(ros_data, desired_encoding='passthrough')     
    #     cv2.imshow('HSV_MATRICE_STREAM', ros_data)
    #     cv2.waitKey(2)
    
    def RGB_HSV_image_callback(self, ros_data):
        if VERBOSE:
            print 'received image of type: "%s"' % ros_data.format

    #### direct conversion to CV2 ####
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_data, desired_encoding='bgr8')  
        # np_arr = np.fromstring(ros_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image_HSV = cv_image
        #cv2.imshow('HSV RGB Matrice', image_np)
        #cv2.waitKey(2)


    def RGB_FT_image_callback(self, ros_data):
        if VERBOSE:
            print 'received image of type: "%s"' % ros_data.format

    #### direct conversion to CV2 ####
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_data, desired_encoding='bgr8')  
        # np_arr = np.fromstring(ros_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image_FT = cv_image
        # cv2.imshow('FT RGB Matrice', image_np)
        # cv2.waitKey(2)

    
    # def RGB_cluster_image_callback(self, ros_data):
    #     if VERBOSE:
    #         print 'received image of type: "%s"' % ros_data.format

    # #### direct conversion to CV2 ####
    #     np_arr = np.fromstring(ros_data.data, np.uint8)
    #     image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    #     cv2.imshow('cluster RGB Matrice', image_np)
    #     cv2.waitKey(2)



   


        
    def obs_point1_rec(self, msg1):
        if  self.flag_record_start == False:
             self.point1_list = []
             self.point2_list = []
             return
        
       
        
        # Take point 1 coordinate from the message 
        x1 = msg1.x
        y1 = msg1.y
        print("x1: {} y1: {}".format(x1, y1))
        self.point1_list.append([x1,y1])  
         
        
            
    def obs_point2_rec(self, msg2):
        if  self.flag_record_start == False:
             self.point1_list = []
             self.point2_list = []
             return
        # Take point 1 coordinate from the message 
        x2 = msg2.x
        y2 = msg2.y
        print("x2: {} y2: {}".format(x2, y2))
        self.point2_list.append([x2,y2])     








def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('RGB_image_view_stream', anonymous=True)
    try:
       
        ic.call_thread_key()

        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS RGB_image_view_stream detector module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
