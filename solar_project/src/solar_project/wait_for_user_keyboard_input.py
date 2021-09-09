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

from ros_simulation_data import publish_obtain_user_control_flag, receive_user_control_flag, publish_key_pressed

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
            key.key_pressed = raw_input("[WAIT USER KEYBOARD INPUT] Press key S to STOP the drone or the correspondant array number to start the drone!")
        except:
            print("[WAIT USER KEYBOARD INPUT] PLEASE PRESS KEY S TO TAKE CONTROL OR A NUMBER TO RELEASE CONTROL: {}".format(key.key_pressed))

        if ( key.key_pressed == ''):
            print("[WAIT USER KEYBOARD INPUT] PLEASE PRESS KEY S TO TAKE CONTROL OR A NUMBER TO RELEASE CONTROL: {}".format(key.key_pressed))
            continue


        key.user_control = receive_user_control_flag()
      
        if (key.user_control == True):
            key.key_pressed = int(key.key_pressed)
            if  (key.key_pressed >= 0 and key.key_pressed <= 100):
                print("[WAIT USER KEYBOARD INPUT] KEY IS A NUMBER, DRONE TAKE CONTROL : {} ".format( key.key_pressed))
                
                publish_key_pressed(key.key_pressed)
                     
            else:
                print("[WAIT USER KEYBOARD INPUT] PLEASE PRESS THE ARRAY NUMBER TO START THE NAVIGATION TASK : {}".format( key.key_pressed) )
        
        key.counter_key = key.counter_key + 1
        key.flag = True
    return 




def listener():
    rospy.init_node('termo_frame_elaboration', anonymous=True)
    
    key = KEY_pressed()
   
    counter_key_old = 0
    count_end = 0
    condition = 10000000000000000
    start = 0
    end = 0
    #create Thread to listen Keyboard input
    try:
        thread = Thread(target = listen_keyboard_input, args = (key, condition))
        thread.start()
        #thread.join()
    except:
        print("Impossible to create the thread to listen the user input")

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


        if key.counter_key > counter_key_old:
           #If the key is pressed publish a flag on a topic 
            flag = True
            if key.flag == True:
                end = time.time()
                key.flag = False
            #print("TIME: {}".format(start- end ))
            if start- end > 2:
                counter_key_old = key.counter_key
        else:
            flag = False
        
        if ( key.key_pressed == ''):
            print("SONO QUI 2")
            thread.kill()
            
        publish_obtain_user_control_flag(flag)
        
      
        count = count + 1

   




if __name__ == '__main__':
    listener()
  