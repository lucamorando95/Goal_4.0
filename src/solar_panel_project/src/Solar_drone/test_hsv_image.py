#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 12:05:23 2020

@author: lucamora
"""





 
import matplotlib
import operator

import matplotlib.pyplot as plt
import matplotlib.image as mpimg


import numpy as np
import sys
from PIL import Image
from scipy import ndimage
import cv2
import time
import os
import math


def load_RGB_image():
     name = '/home/lucamora/catkin_ws/test1.png'
     image = cv2.imread(name) 
     
     #cluster_image = image_pre_processing(image,photo_number, th3_value, th4_value, rotation_degree)
     
     return image
     




if __name__ == '__main__':
   image = load_RGB_image()
   cv2.imshow('image', image)
   cv2.waitKey(1000000)
    
    