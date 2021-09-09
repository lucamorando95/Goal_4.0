#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 15:09:43 2020

@author: lucamora
"""


""" 
Script that recognize if a key is pressed and it send a message to the drone,
in particular to the cpp script requiring the drone to stop or to start along the PV arrays
"""


import matplotlib
import operator

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import rospy 

import numpy as np

from scipy import ndimage
import cv2
import time
import os
import math
import networkx 
from networkx.algorithms.components.connected import connected_components
import PIL.Image

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread

from ros_simulation_data import publish_desired_velocity

class KEY_pressed:
    def __init__(self):
        self.key_pressed = 0
        self.counter_key = 0
        self.counter_key_old = 0
        self.flag = False
        self.user_control = False

def listen_keyboard_input(key, condition):
    while (key.counter_key < condition):
        
        try:
            key.key_pressed = float(raw_input("[SELECT DESIRED VELOCITY] Select the desired velocity of the UAV: ----> "))
        except:
            print("[SELECT DESIRED VELOCITY] PLEASE SELECT A DECIMAL NUMBER")

        if ( key.key_pressed == ''):
            print("[SELECT DESIRED VELOCITY] PLEASE PRESS A NUMBER: {}".format(key.key_pressed))
            continue

        
        if ( key.key_pressed > 4.0):
            print("[SELECT DESIRED VELOCITY] PLEASE SELECT A VELOCITY LOWER THAN 4 M/S: {}".format(key.key_pressed))
            continue

        if ( key.key_pressed < 0.0):
            print("[SELECT DESIRED VELOCITY] PLEASE SELECT A POSITIVE VELOCITY: {}".format(key.key_pressed))
            continue
                
        publish_desired_velocity(key.key_pressed)
                     
         
        
        key.counter_key = key.counter_key + 1
        key.flag = True
    return 




def listener():
    rospy.init_node('Select_desired_velocity', anonymous=True)
    
    key = KEY_pressed()
   
    counter_key_old = 0
    count_end = 0
    condition = 10000000000000000
    start = 0
    end = 0
   
    #Listen Keyboard input 
    count = 0
    flag = False
    while(count < condition):
        if count == 1:
             #create Thread to listen Keyboard input
            try:
                thread = Thread(target = listen_keyboard_input, args = (key, condition))
                thread.daemon = True
                thread.start()
                #thread.join()
            except:
                print("Impossible to create the thread to listen the user input")

        start = time.time()

        #receive Flag = True if the drone is under user control 
       
        # print("############################################",  key.user_control  )

        time.sleep(0.5)
      
        count = count + 1

   




if __name__ == '__main__':
    listener()
  