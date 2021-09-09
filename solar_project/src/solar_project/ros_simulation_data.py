#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 15:18:34 2020

@author: lucamora
"""


import rospy 
import time
from geometry_msgs.msg import Twist,PoseWithCovariance, Quaternion, Point, Pose, Vector3, Vector3Stamped, PoseStamped, PoseWithCovarianceStamped,PointStamped
from std_msgs.msg import String, Header, Bool, Float32, Int32
from std_msgs.msg import Empty 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, CompressedImage
from solar_project.msg import HSV, THERMO, KEY_PRESSED   #Modificare e iniserire in dji_ws
from dji_osdk_ros.msg import VOPosition
#from keras.layers.core import Dense, Dropout, Activation, Flatten
import matplotlib.pyplot as plt
import numpy as np
import time
import random
import math
import pdb
import cv2
import sys

from cv_bridge import CvBridge, CvBridgeError


def takeEnvObservations(): #Function which takes information from the environment in gazebo 
       
        #Take pose information 
        poseData = None
        count_for_exit_while = 1
        while poseData is None :
            try:
                poseData = rospy.wait_for_message('/dji_osdk_ros/local_position', PointStamped, timeout = 1)
            except:
                rospy.loginfo('Unable to reach the drone /dji_osdk_ros/local_position topic. Try to connect again')
                if count_for_exit_while > 100:
                   break
                count_for_exit_while = count_for_exit_while +1

        # altiutde_data = None
        # count_for_exit_while2 = 1
        # while altiutde_data is None :
        #     try:
        #         altitude_data = rospy.wait_for_message('/dji_osdk_ros/local_position', VOPosition, timeout = 1)
        #     except:
        #         rospy.loginfo('Unable to reach the drone /dji_osdk_ros/local_position topic. Try to connect again')
        #         if count_for_exit_while2 > 100:
        #            break
        #         count_for_exit_while2 = count_for_exit_while2 +1
            
        #     altitude = -1*altitude_data.z + 100


        # imuData = None 
        # while imuData is None :
        #     try:
        #         imuData = rospy.wait_for_message('/ardrone/imu', Imu, timeout = 1)
        #     except:
        #         rospy.loginfo('Unable to reach the drone Imu topic. Try to connect again')
        
        # velData = None
        # while velData is None:
        #   try:
        #       velData = rospy.wait_for_message('/fix_velocity', Vector3Stamped, timeout=1)
        #   except:
        #       rospy.loginfo("Unable to reach the drone Imu topic. Try to connect again")
        # altitudeVelDrone = None
       
        # while altitudeVelDrone is None:
        #   try:
        #       altitudeVelDrone = rospy.wait_for_message('/drone/cmd_vel', Twist, timeout=5)
                
        #   except:
        #       rospy.loginfo("Unable to reach the drone velocity topic. Try to connect again")      
              
    
        # return poseData, imuData, velData, altitudeVelDrone
        return poseData



def take_drone_camera_frame():
    camera_frame = None
    count_for_exit_while = 1
    while camera_frame is None :
        try:
            camera_frame = rospy.wait_for_message('/dji_osdk_ros/main_camera_images', Image, timeout = 5)
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='passthrough')
            cv_image = cv_image[0:cv_image.shape[0], cv_image.shape[1]/2: cv_image.shape[1]]        
            
           
           
        except:
            rospy.loginfo('Unable to reach the drone camera topic. Try to connect again')
            if count_for_exit_while > 100:
                break
            count_for_exit_while = count_for_exit_while +1
       
    return cv_image




def take_drone_camera_RGB_frame():
    camera_frame = None
    count_for_exit_while = 1
   
    while camera_frame is None :
     
        try:
            
            camera_frame = rospy.wait_for_message('/dji_osdk_ros/main_camera_images', Image, timeout = 1)
            
            bridge = CvBridge()
        
            cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='bgr8')
            cv_image = cv_image[0:cv_image.shape[0], 0:cv_image.shape[1]/2] 
               
           
        except:
            rospy.loginfo('Unable to reach the drone Main camera images topic. Try to connect again') 
        
        if count_for_exit_while > 100:
              break
        count_for_exit_while = count_for_exit_while +1
    return cv_image
               
       
           
##### ONLY in SIMULATION
def take_drone_camera_RGB_frame_simulation():
    camera_frame = None
    count_for_exit_while = 1
  
    while camera_frame is None :
       
       
        try:
            camera_frame = rospy.wait_for_message('/gimbal_upward_RGB_camera/image_raw', Image, timeout = 10)
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='bgr8')
            
        except:
            rospy.loginfo('Unable to reach the drone camera topic. Try to connect again') 
        
        if count_for_exit_while > 100:
              break
        count_for_exit_while = count_for_exit_while +1

        return cv_image    


def take_drone_camera_frame_simulation():
    camera_frame = None
    count_for_exit_while = 1
  
    while camera_frame is None :
       
       
        try:
            camera_frame = rospy.wait_for_message('/gimbal_bottom_camera/image_raw', Image, timeout = 10)
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(camera_frame, desired_encoding='bgr8')
            
        except:
            rospy.loginfo('Unable to reach the drone camera topic. Try to connect again') 
        
        if count_for_exit_while > 100:
              break
        count_for_exit_while = count_for_exit_while +1

        return cv_image    










    


def receive_estimated_control_point_P1():
    Point_1_B = None 
    count_for_exit_while = 1
    while Point_1_B is None :
       
        try:
            Point_1_B = rospy.wait_for_message('/P1_estimated_control_point', Point, timeout = 1)
        except:
            rospy.loginfo('Unable to reach the drone Estimated Point P1 topic. Try to connect again')
            if count_for_exit_while > 2:
                
                break
            count_for_exit_while = count_for_exit_while +1
        
    
    return Point_1_B

def receive_estimated_control_point_P2():
    Point_2_B = None 
    count_for_exit_while = 1
    while Point_2_B is None :
         
        try:
            Point_2_B = rospy.wait_for_message('/P2_estimated_control_point', Point, timeout = 1)
        except:
            rospy.loginfo('Unable to reach the drone Estimated Point P2 topic. Try to connect again')
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    
    return Point_2_B


#########################################################################################################
############################    Subscribe  to Gazebo simulation topic for RGB and thermale estimated position 
def sub_gazebo_sim_thermal_control_point_P1():
    Point_1_sim = None 
    count_for_exit_while = 1
    while Point_1_sim is None :
         
        try:
            Point_1_sim = rospy.wait_for_message('/Thermo_control_point_1', Point, timeout = 1)
        except:
            rospy.loginfo('Unable to reach the drone Estimated Point P2 topic. Try to connect again')
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    
    return Point_1_sim


def sub_gazebo_sim_thermal_control_point_P2():
    Point_2_sim = None 
    count_for_exit_while = 1
    while Point_2_sim is None :
         
        try:
            Point_2_sim = rospy.wait_for_message('/Thermo_control_point_2', Point, timeout = 1)
        except:
            rospy.loginfo('Unable to reach the drone Estimated Point P2 topic. Try to connect again')
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    
    return Point_2_sim



def receive_flag_KF_init():
    data = None
    count_for_exit_while = 1
    while data is None :
         
        try:
            data = rospy.wait_for_message('/dji_osdk_ros/KF_init', Bool, timeout = 10)
        except:
            rospy.loginfo('Unable to reach the -- KF_init --  topic. Try to connect again')
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    
    return data.data



def receive_flag_user_control():
    data = None
    count_for_exit_while = 1
    while data is None :
         
        try:
            data = rospy.wait_for_message('/dji_osdk_ros/user_joy_control_obtained', Bool, timeout = 10)
        except:
            rospy.loginfo('Unable to reach the --/dji_osdk_ros/user_joy_control_obtained --  topic. Try to connect again')
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    
    return data.data


############### Receive flag inNav ###################
def receive_inNav_flag():
    data = None
    count_for_exit_while = 1
    while data is None :
         
        try:
            data = rospy.wait_for_message('/dji_osdk_ros/inNav_flag', Bool, timeout = 10)
        except:
            rospy.loginfo('Unable to reach the --/dji_osdk_ros/inNav_flag --  topic. Try to connect again')
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    
    return data.data






################# OPTIMIZATION TASK##########################
#Relative to Optimization Alg
def wait_for_opt_task():
    """
    Wait true flag from solar_flight control.
    It means that the drone is in correct positiion and the parameters optimization should start.
    """
    data = None
    count_for_exit_while = 1
    while data is None :
         
        try:
            data = rospy.wait_for_message('/dji_osdk_ros/optimization_start', Bool, timeout = 10)
        except:
            rospy.loginfo('Unable to reach the -- optimization_start --  topic. Try to connect again')
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    
    return data.data



def receive_HSV_parameters():
    """
    RGB script shoudl receive the optimized parameters 
    """
    data = None
    count_for_exit_while = 1
    while data is None :
         
        try:
            data = rospy.wait_for_message('/dji_osdk_ros/HSV_optimized_param', HSV, timeout = 1)
        except:
            rospy.loginfo('Unable to reach the -- HSV_optimized_param --  topic. Try to connect again')
            if count_for_exit_while > 1:
               break
            count_for_exit_while = count_for_exit_while +1
    if data is None:
        hsv_param = [0,0,0]
    else:
        hsv_param =  data.data
    return hsv_param





def receive_Thermo_opt_parameters():
    """
    Thermo script shoudl receive the optimized parameters 
    """
    data = None
    count_for_exit_while = 1
    while data is None :
         
        try:
            data = rospy.wait_for_message('/dji_osdk_ros/Thermo_optimized_param', THERMO, timeout = 10)
        except:
            rospy.loginfo('Unable to reach the -- Thermo_optimized_param --  topic. Try to connect again')
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    if data is None:
        thermo_param = [0,0,0,0,0]
    else:
        thermo_param = data.data
    
    return thermo_param





def receive_optimization_flag():
    """
    Receive Flag relative to complete optimization.
    Script RGB and THermo detection receive this flag only when the optimization is completed.
    The next action is the possibility to the user to choice the desired set of parameters.
    """
    
    data = None
    count_for_exit_while = 1
    while data is None :
         
        try:
            data = rospy.wait_for_message('/dji_osdk_ros/optimization_completed', Bool, timeout = 1)
            
        except:
            rospy.loginfo('Unable to reach the -- optimization_completed --  topic. Try to connect again')
           
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    
    return data.data


def receive_user_control_flag():
    """"
    Receive flag which is true when the drone is under user control 
    """
    data = None
    count_for_exit_while = 1
    while data is None :
         
        try:
            data = rospy.wait_for_message('/dji_osdk_ros/drone_is_under_user_control', Bool, timeout = 10)
        except:
            rospy.loginfo('Unable to reach the -- drone_is_under_user_control --  topic. Try to connect again')
            if count_for_exit_while > 2:
               break
            count_for_exit_while = count_for_exit_while +1
    
    return data.data





def publish_optimization_completed(flag):
    """
    Publish to flight control the flag relative to the completed optimization.
    This flag is sent only when the user has seen the result of the final optimization and the drone 
    is ready to go
    """
    pub = rospy.Publisher('/dji_osdk_ros/parameter_opt_completed', Bool, queue_size=1)
    msg = Bool()
    msg.data = flag
    pub.publish(msg)


#####################################################

def publish_optimization_flag(flag):
    """
    Publish a flag that indicates when the RGB detection script should start to 
    listen the HSV_optimized_param topic 
    """
    pub = rospy.Publisher('/dji_osdk_ros/optimization_completed', Bool, queue_size=1)
    msg = Bool()
    msg.data = flag
    pub.publish(msg)



#Publish HSV parameters to python_script
def publish_HSV_parameters(H, S, V): 
    pub =  rospy.Publisher('/dji_osdk_ros/HSV_optimized_param', HSV, queue_size=1)  
    message = HSV()
    message.data = [H, S, V]
    pub.publish(message)



#Publish Thermal Param
def publish_Thermo_parameters(args):
    '''
    Publish Optimization Parameters relative to the Theramal Optimization
    '''
    th_0 = args[0]
    th_1_l = args[1]
    th_1_h = args[2]
    th_2 = args[3]
    th_3 = args[4]
    th_4 = args[5]


    pub =  rospy.Publisher('/dji_osdk_ros/Thermo_optimized_param', THERMO, queue_size=1)  
    message = HSV()
    message.data = [th_0, th_1_l, th_1_h, th_2, th_3, th_4]
    pub.publish(message)




###########################################################################


def publish_desired_velocity(vel):
    #publish desired velocity to the drone 
    pub =  rospy.Publisher('/dji_osdk_ros/user_desired_velocity', Float32, latch=True, queue_size=1)  
    message = Float32()
    message.data = vel
    pub.publish(message)

def publish_desired_velocity_y(vel):
    #publish desired velocity to the drone 
    pub =  rospy.Publisher('/dji_osdk_ros/user_desired_velocity_y', Float32, latch=True, queue_size=1)  
    message = Float32()
    message.data = vel
    pub.publish(message)


def publish_key_pressed(key):
   
    pub =  rospy.Publisher('/dji_osdk_ros/key_pressed', KEY_PRESSED, latch=True, queue_size=1)  
    message = KEY_PRESSED()
    message.data = key
    pub.publish(message)


def publish_opt_image(image):
    '''
    Publish Optimized image to PC Dell in order to make the user able to select the image on line
    '''

    image_pub = rospy.Publisher("/dji_osdk_ros/OPT_RGB_IMG",
            CompressedImage)
    
    
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
    # Publish new image
    image_pub.publish(msg)

   
def publish_output_THERMO_image(image):
    '''
    Publish Optimized image to PC Dell in order to make the user able to select the image on line
    '''

    image_pub = rospy.Publisher("/dji_osdk_ros/OPT_THERMO_IMG",
            CompressedImage)
    
    
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
    # Publish new image
    image_pub.publish(msg)



#################################################à



def publish_altitude_offset(offset):
    pub = rospy.Publisher('/dji_osdk_ros/altitude_offset', Float32, queue_size=1)
    msg = Float32()
    msg.data = offset
    pub.publish(msg)



def publish_navigation_Thermo_point(x1, y1, x2, y2):
    pub = rospy.Publisher('Thermo_control_point_1', Point, queue_size=1)
    pub2 = rospy.Publisher('Thermo_control_point_2', Point, queue_size=1)
   
    #Punto 1
    msg = Point()
    msg.x= x1
    msg.y = y1
    msg.z = 0.0
    pub.publish(msg)
  
    #Punto 2
    msg1 = Point()
    msg1.x= x2
    msg1.y = y2
    msg1.z = 0.0
    pub2.publish(msg1)

  
  
def  publish_navigation_Thermo_point2(x1, y1, x2, y2):
    pub = rospy.Publisher('Thermo_control_point2_1', Point, queue_size=1)
    pub2 = rospy.Publisher('Thermo_control_point2_2', Point, queue_size=1)
   
    #Punto 1
    msg = Point()
    msg.x= x1
    msg.y = y1
    msg.z = 0.0
    pub.publish(msg)
  
    #Punto 2
    msg1 = Point()
    msg1.x= x2
    msg1.y = y2
    msg1.z = 0.0
    pub2.publish(msg1)
    
    
    
    
def publish_navigation_RGB_point(x1_RGB, y1_RGB, x2_RGB, y2_RGB):
    pub_RGB1 = rospy.Publisher('RGB_control_point_1', Point, queue_size=1)
    pub2_RGB2 = rospy.Publisher('RGB_control_point_2', Point, queue_size=1)
   
    #Punto 1
    msg_RGB = Point()
    msg_RGB.x= x1_RGB
    msg_RGB.y = y1_RGB
    msg_RGB.z = 0.0
    pub_RGB1.publish(msg_RGB)
  
    #Punto 2
    msg1_RGB = Point()
    msg1_RGB.x= x2_RGB
    msg1_RGB.y = y2_RGB
    msg1_RGB.z = 0.0
    pub2_RGB2.publish(msg1_RGB)
    
    
    
#Publish compressed images   
def publish_output_image(clustered_image):
    Camera_elaborated_thermo_frame_pub = rospy.Publisher('camera_vision_output', Image, queue_size=1)
    #Camera_elaborated_thermo_frame_pub = rospy.Publisher("/camera_vision_output/compressed",CompressedImage, queue_size=1)
    #### Create CompressedIamge ####
    # msg = CompressedImage()
    # msg.header.stamp = rospy.Time.now()
    # msg.format = "jpeg"
    # msg.data = np.array(cv2.imencode('.jpg', clustered_image)[1]).tostring()
   
    #msg = Image()
    msg = CvBridge().cv2_to_imgmsg(clustered_image,"rgb8")
   
    #Publish The image 
    Camera_elaborated_thermo_frame_pub.publish(msg)


    
def publish_output_RGB_image(clustered_RGB_image):
    camera_elaborated_frame_pub = rospy.Publisher('camera_vision_RGB_output', Image, queue_size=1)
    #camera_elaborated_frame_pub = rospy.Publisher('camera_vision_RGB_output/compressed', CompressedImage, queue_size=1)
     #### Create CompressedIamge ####
    # msg = CompressedImage()
    # msg.header.stamp = rospy.Time.now()
    # msg.format = "jpeg"
    # msg.data = np.array(cv2.imencode('.jpg', clustered_RGB_image)[1]).tostring()

    #msg = Image()
    msg_frame_RGB = CvBridge().cv2_to_imgmsg(clustered_RGB_image,"rgb8")
    #msg.data = msg_frame_RGB
    #Publish The image 
    camera_elaborated_frame_pub.publish(msg_frame_RGB)


######ONçLY FOR DEBUGGING #################################
def publish_img_hsv(img):
    camera_hsv_frame_pub = rospy.Publisher('OVTA_DEBUG_HSV_image', Image, queue_size=1)
   # msg = Image()
    msg_frame_HSV = CvBridge().cv2_to_imgmsg(img,"bgr8")
    #msg.data = msg_frame
    #Publish The image 
    camera_hsv_frame_pub.publish(msg_frame_HSV)

def publish_img_first_threshold(img):
    camera_first_frame_pub = rospy.Publisher('OVTA_DEBUG_FIRST_THRESHOLD_image', Image, queue_size=1)
   # msg = Image()
    msg_frame_first = CvBridge().cv2_to_imgmsg(img, '8UC1')
    #msg.data = msg_frame
    #Publish The image 
    camera_first_frame_pub.publish(msg_frame_first)

def publish_img_clustering_mask(img):
    camera_cluster_mask_pub = rospy.Publisher('OVTA_DEBUG_CLUSTERING_image', Image, queue_size=1)
   # msg = Image()
    msg_frame_mask = CvBridge().cv2_to_imgmsg(img, '8UC3')
    #msg.data = msg_frame
    #Publish The image 
    camera_cluster_mask_pub.publish(msg_frame_mask)





    ##################à PUBLISH IMAGE FOR DEBUGGING frome THERMAL 
def pub_gray_image(img):
    
    camera_gray_frame_pub = rospy.Publisher('OVTA_THERMAL_GRAY', Image, queue_size=1)
   # msg = Image()
    msg_frame_gray = CvBridge().cv2_to_imgmsg(img,"32FC1")
    #msg.data = msg_frame
    #Publish The image 
    camera_gray_frame_pub.publish(msg_frame_gray)
    
def pub_th_image(img):
    camera_th_frame_pub = rospy.Publisher('OVTA_THERMAL_TH', Image, queue_size=1)
   # msg = Image()
    msg_frame_th = CvBridge().cv2_to_imgmsg(img,"32FC1")
    #msg.data = msg_frame
    #Publish The image 
    camera_th_frame_pub.publish(msg_frame_th)
    
    
def pub_distance_image(img):
    camera_distance_matr_frame_pub = rospy.Publisher('OVTA_THERMAL_DISTANCE', Image, queue_size=1)
   # msg = Image()
    msg_distance_frame = CvBridge().cv2_to_imgmsg(img,"64FC1")
    #msg.data = msg_frame
    #Publish The image 
    camera_distance_matr_frame_pub.publish(msg_distance_frame)
    
  


  #Services Script
def publish_obtain_user_control_flag(flag):
    """
    Publish a flag received by the Fligth controller only when the user press a keyboard input.
     The drone will start or stop its navigation depending when it is received.
    """

    pub = rospy.Publisher('/dji_osdk_ros/user_keyboard_input', Bool, queue_size=1)
    msg = Bool()
    msg.data = flag
    pub.publish(msg)

    
