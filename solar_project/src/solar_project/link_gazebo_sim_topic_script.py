
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import sys

import cv2
import time
import os
import math
from random import  uniform




import rospy
from ros_simulation_data import sub_gazebo_sim_thermal_control_point_P1
from ros_simulation_data import sub_gazebo_sim_thermal_control_point_P2
from ros_simulation_data import publish_navigation_Thermo_point



def listener():
    rospy.init_node('termo_frame_elaboration', anonymous=True)
    #Create Image Publisher
    count = 0
    condition = 100000000
    print("SOno qui")
    r = rospy.Rate(10) # 10hz 
    while not rospy.is_shutdown():

       
        
       #Subscribe to RGB and Thermal Control Point from Gazebo sim 
        # P1 = sub_gazebo_sim_thermal_control_point_P1()
        # P2 = sub_gazebo_sim_thermal_control_point_P2()
        P1_x = 0.0
        P1_y = -0.1#niform(0, .01)
        P2_x = 0.6
        P2_y =-0.3#uniform(0, .01)
       #Publish Control Point taken from Gazebo simulator via the topic obtained with ROS via internet
        publish_navigation_Thermo_point(P1_x, P1_y, P2_x, P2_y)
      

        count = count + 1
        r.sleep()
       

   





if __name__ == '__main__':
    listener()
   # file1.close()
