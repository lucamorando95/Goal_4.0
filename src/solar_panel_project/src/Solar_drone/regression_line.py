#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun  8 22:33:50 2020

@author: lucamora
"""
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import os
import math
from scipy import ndimage
# from PIL import Image, ImageDraw
#from sklearn import cluster
#rom sklearn.datasets.samples_generator import make_blobs
#from sklearn.preprocessing import StandardScaler
import time

th_theta = 60
def regression_line(image, theta_mean):
    #Per calcolare la retta di regressione nell'immagine devo considerare l'asse y nella direzione dell'asse x e l'asse x lungo le righe
    #------->y
    #|
    # |
    # |
    # x
    shape = image.shape
    # print('shape: ', shape)
    row = shape[0]
    col = shape[1]
    col_start = 0
    #Find mean values x_mean and y_mean
    x_coo = []
    y_coo = []
    
    #Search x coo of pixels different from zero
    if theta_mean > th_theta:
       for ii in range(0, row):
          for jj in range(col_start, col):
              if not (image[ii][jj] == 0):
                  x_coo.append(ii)
                  y_coo.append(jj)
    else:
        #Inverto gli assi se rette sono orizzontali, cioe con angoli vicini ai 0 gradi. 
        for ii in range(0, row):
          for jj in range(col_start, col):
              if not (image[ii][jj] == 0):
                  y_coo.append(ii)
                  x_coo.append(jj)
                  
    x_mean, y_mean = evaluate_mean(x_coo, y_coo)   
    print('x_mean: ', x_mean)
    print('y_mean: ', y_mean)
    #Evaluate Parameters slope b1 and intercept b0 --> y = b1x + b0
    b0, b1, y_down =  evaluate_parameters(x_coo, y_coo, x_mean, y_mean, row, col, col_start, theta_mean)
    
    # #Draw Line on image 
    # cv2.line(image, (y_down, row), (b0, 0), (255,0,0), 2)
    # cv2.imshow("Image", image)
   
    return image, b0, b1, y_down
    # regression_image = 

def evaluate_parameters(x_coo, y_coo, x_mean, y_mean, row, col, col_start, theta_mean): 
    #b1 = sum(x_i - x_bar)(y_i - y_bar)/ sum(x_i - x_bar)^2
    num = 0
    den = 0
    # if (theta_mean > th_theta): 
    for ii in range(0, len(x_coo)):
       num += (x_coo[ii] - x_mean)*(y_coo[ii] - y_mean)
       den += pow((x_coo[ii] - x_mean), 2)
      
    #slopes
       b1 = num/den
    
    #b0 intercept: b0 = y_mean - b1*x_mean
    #b0 intercept in upper border
   
       b0 = y_mean - b1*x_mean
   
    
       y_down = b1*row + b0
    
    # else:
    #     for ii in range(0, len(x_coo)):
       
    #         # num += (y_coo[ii] - y_mean)*(x_coo[ii] - x_mean)
    #         # den += pow((y_coo[ii] - y_mean), 2)
    #        num += (x_coo[ii] - x_mean)*(y_coo[ii] - y_mean)
    #        den += pow((x_coo[ii] - x_mean), 2)
    # #slopes
    #     b1 = num/den
    
    # #b0 intercept: b0 = y_mean - b1*x_mean
    # #b0 intercept in upper border
   
    # # b0 = y_mean - b1*x_mean
    #     b0 = y_mean - b1*x_mean #--> #in questa maniera b0 e b1 sono le intersezioni con i bordi laterali
    
    #     y_down = b1*row + b0
    
    # print('y_down: ', y_down)
    # print('b1: ', b1)
    # print('b0: ', b0)
    return int(b0), b1, int(y_down)


    
def evaluate_mean(x_coo, y_coo):  
    numx = 0
    numy = 0    
    for ii in range(0, len(x_coo)):
        numx += x_coo[ii]
        numy += y_coo[ii]
    x_mean = numx/len(x_coo)
    y_mean = numy/len(y_coo)
    
    return x_mean, y_mean
        
    ####Per le immagini ruotate di 90 gradi è necessario invertire le righe con le colonne, quindi cambiare gli assi
#inoltre cv2 line bisogna posizionare asse y in corrispondenza delle righe e x in corrsipondenza delle colonne quando l'immagine è a 90 gradi
    #Bisogna capire che se la retta di regressione non ha intersezione con le colonne allora avra intersezione con la colonna zero e la colonna massima in corripondnza delle righe dei
#bordi di destrra e di sinistra 