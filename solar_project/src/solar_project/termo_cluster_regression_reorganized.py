#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 15:09:43 2020

@author: lucamora
"""


###In questo script l'immagine originale è scalata. su one drive c'è quello originale 

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
#from sklearn import cluster


from ros_simulation_data import take_drone_camera_frame, takeEnvObservations, receive_estimated_control_point_P1, receive_estimated_control_point_P2, take_drone_camera_frame_simulation
from ros_simulation_data import publish_navigation_Thermo_point
from ros_simulation_data import publish_navigation_Thermo_point2
from ros_simulation_data import publish_output_image, publish_output_THERMO_image
from ros_simulation_data import pub_gray_image, pub_th_image, pub_distance_image
from ros_simulation_data import wait_for_opt_task, receive_Thermo_opt_parameters, receive_optimization_flag

from Thermo_image_detection import Image_THERMO_elaboration



class drone(object):
    #def __init__(self, pose):
    def __init__(self):
        #Store position data
        self.x = 0 #pose.point.x
        self.y = 0 # pose.point.y
        self.z = 15 #pose.point.z


#############  TAke drone status information ###################
def takedronedata():
    # poseData, imuData, velData, altitudeVelDrone = takeEnvObservations()
    #poseData = takeEnvObservations()
    drone_obj = drone()#poseData)
    return drone_obj




def image_pre_processing(THERMO,image, line_versors_old,drone_obj):
    flag_final_frame_analyzed = False
    ########## Rotation and resize of image ###########
    pub_debug_img = True
   

    
    # THERMO.th_0 = 125.547 #125.547 #Reshape intensity pixels values 
    # THERMO.th_1 = [120, 180] #[120, 180] #mask
    # THERMO.th_2 = 9 #14 #distance 
    # THERMO.th_3 =  5000 #5500 #4500 # 2000 area
    # THERMO.th_4 = 0.05  #epsilon aproxPolydp 
    
    THERMO.index_th1 = 0
    THERMO.index_th2 = 0
    THERMO.index_th3 = 0
    THERMO.index_th4 = 0
    inf = THERMO.th_1[0]
    sup =  THERMO.th_1[1]

    THERMO.original_image_shape = [0.0,0.0]

     #GRAYSCALE
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    #Evaluate mean of all image pixel intensities
    intensity_mean = 0
    intensity_mean = np.mean(gray)
    #print("intensity_mean: ", intensity_mean)
   
    THERMO.original_image_shape[0] = gray.shape[0]
    THERMO.original_image_shape[1] = gray.shape[1]
    
    const_int_value = [ [ int(THERMO.th_0 - intensity_mean) for y in range( gray.shape[1] ) ] for x in range( gray.shape[0] ) ]
    
   
    const_int_value = np.array(const_int_value)
   # gray = const_int_value + gray
    gray=gray.astype(np.float32)
    
    if pub_debug_img:
       pub_gray_image(gray)
    # cv2.imshow('threshold', gray)
    # cv2.waitKey(1)
    
    intensity_mean = np.mean(gray)
      
    
       
    percent = 70 #70
    #Resize Image 
    width = int(image.shape[1] * percent/ 100)
    height = int(image.shape[0] * percent/ 100)
    dim = (width, height)

    frame = cv2.resize(gray, dim, interpolation =cv2.INTER_AREA)
    image = cv2.resize(image, dim, interpolation =cv2.INTER_AREA)
    
   

    blur1 = cv2.bilateralFilter(frame,3,75,75)
    ret,thresh_blur1 = cv2.threshold(blur1,inf,255,cv2.THRESH_TOZERO)
    ret,thresh_blur2 = cv2.threshold(thresh_blur1,sup,255,cv2.THRESH_TOZERO_INV)

   

    
    
    if pub_debug_img:
       pub_th_image(thresh_blur2)
    
    distance_matrix = ndimage.distance_transform_edt(thresh_blur2) 
    
    # imgplot = plt.imshow(distance_matrix,cmap='gray', vmin = 1, vmax = 255)
    # plt.show()
   
    ret,thresh= cv2.threshold(distance_matrix,THERMO.th_2,255,cv2.THRESH_BINARY)
    # cv2.imshow('distance', thresh)
    # cv2.waitKey(1)
    if pub_debug_img:
       pub_distance_image(thresh)
    # imgplot = plt.imshow(thresh,cmap='gray', vmin = 1, vmax = 255)
    # plt.show()
    thresh = cv2.convertScaleAbs(thresh)
    _ ,contours, _= cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    

   
    THERMO.first_cluster_search(contours, thresh, image)
    
    if len(THERMO.start_point) == 1:
        cluster = [[0]]
    else:
        #CLUSTERING LINES 
        cluster , theta_mean = THERMO.clustering_lines(thresh.shape[0], thresh.shape[1])  
    


    ### Create a mask for each cluster, Draw Contours and fit line for each detected shape in the cluster 
    line_versors, mask = THERMO.create_cluster_mask(cluster, image, thresh, drone_obj)

    _ , mean_distance_of_line_from_center_line, line_points = THERMO.voting_technique(line_versors, line_versors_old, thresh)

     #Take the line which presents the minimum distance of all of its point respect the frame central line 
    
    if (len(mean_distance_of_line_from_center_line) > 0): #and math.isnan(mean_distance_of_line_from_center_line[0]) == False ):
    # if (len(cartesian_distance_point_from_image_center) > 0):
        #Ordering vector from lowest to highest value
          
        min_line = min(mean_distance_of_line_from_center_line)

         #Take the index of the first line 
        try:
            index = mean_distance_of_line_from_center_line.index(min_line)
            #print("index: ", index)
        except:
            index = 0 #Prende la rpima linea se non trova l'index desiderato
        
        # print("line_points: ", line_points)
        # print("index: ", index)
        # print("min_line: ", min_line)
        # print("line_points: ", line_points)
        # print("index: ", index)
        line_considered = line_points[index]
        print("###############  mean line distance from center image: ",    mean_distance_of_line_from_center_line[index][0])

        THERMO.line_point_exportation_in_drone_body_frame(line_considered[0],line_considered[1],line_considered[2],line_considered[3], (thresh.shape[0]/2), (thresh.shape[1]/2), drone_obj, first = True)  #Definsico locazione del body frame del drone rispetto l'image plane (esattamente al centro )
        flag_final_frame_analyzed = True
        THERMO.counter_frame_analyzed = THERMO.counter_frame_analyzed + 1

        # imgplot = plt.imshow(image,cmap='gray', vmin = 0, vmax = 255)
        # plt.show()
    return  image, line_versors
         


def evaluate_altitude_offset(RGB, flag_width):
    if (flag_width == True):
        offset = RGB.desired_mask_width - RGB.mask_width
        #print("offset: ", offset)


def load_image(photo_number):
     photo_number = 540

     #name = '/home/lucamora/image_analysis/Volo_01/DJI_0' + str(photo_number) + '.jpg'
     name = '/home/dji/Desktop/test1.png'
     image = cv2.imread(name) 
     
     
     #tiff.imsave('/home/lucamora/Desktop/new.tiff', image)
     #cluster_image = image_pre_processing(image,photo_number,inf, sup, th2_value, th3_value, th4_value, rotation_degree)
     
     return image


def listen_keyboard_input(THERMO, dir_opt):
    
       
    Image2 = cv2.imread(dir_opt)
    for ii in range(0, 50):
        publish_output_THERMO_image(Image2)
        time.sleep(0.2)

    print("Please Select a Picture to continue: 1 - LEFT, 2 - RIGHT")
    print("Press Enter to continue after the picture selection")
    THERMO.selection = input("Select the desired set of Parameters")
    plt.close('all')
    return



def show_images(THERMO, dir_manual, dir_opt, drone_obj, clustered_image):

   #Create a thread to take input from keyboard
    try:
        thread = Thread(target = listen_keyboard_input, args = (THERMO, dir_opt,))
        thread.start()
        #thread.join()
    except:
        print("Impossible to create the thread to listen the user input")

    
 

    print("WAIT UNTIL A SELECTION IS MADE")
    while(THERMO.selection == 0):
        publish_output_image(clustered_image)
        time.sleep(0.1)




def listener():
    rospy.init_node('termo_frame_elaboration', anonymous=True)
    #Create Image Publisher
    Camera_elaborated_frame_pub = rospy.Publisher('camera_vision_output', Image, queue_size=10)
    
    #Save clustered image to give user ability to select it
    user_clustered_image_path = '/home/dji/DATA/dji_ws/user_image_selection_after_opt'

    #Inizializzo la classe di Thermal Detection 
    THERMO = Image_THERMO_elaboration()
   
    THERMO.th_0 = rospy.get_param("/THERMO_params/th_0")
    THERMO.th_1 = rospy.get_param("/THERMO_params/th_1")
    THERMO.th_2 = rospy.get_param("/THERMO_params/th_2")
    THERMO.th_3 = rospy.get_param("/THERMO_params/th_3")
    THERMO.th_4 = rospy.get_param("/THERMO_params/th_4")
    THERMO.parameters_optimization_enabled = rospy.get_param("/THERMO_params/parameters_optimization_enabled")
    simulation_mode_enabled = rospy.get_param("/THERMO_params/simulation_mode_enabled")

    condition = 10000
    count = 0
    line_versors_old = []
    
    ext = '.png'
    filename_original = user_clustered_image_path + '/default_thermo_param_img' + ext
    
    image_received_flag = False
    THERMO_width_selected_flag = False
    opt_flag = True
    while(count < condition):
        #Save all informations in drone struct 
        start1 = time.time()
        drone_obj = takedronedata()
        
        # print('end1: ', end1)
        #Take image frame
        
     
        try:
            if simulation_mode_enabled == True:
                image = take_drone_camera_frame_simulation()
            else:
                image = take_drone_camera_frame()
            image_received_flag = True
          
        except: 
            print("[THERMO INFO] Impossible to obtain RGB images")
            image_received_flag = False
        #image = load_image(2)
       
        # imgplot = plt.imshow(image,cmap='gray', vmin = 0, vmax = 255)
        # plt.show()  
       
        clustered_image, line_versors = image_pre_processing(THERMO, image, line_versors_old, drone_obj)
        # line_versors = []
        # clustered_image = []
       
        
        if THERMO.parameters_optimization_enabled == True:
           
            #wait flag from flight controller which start the optimization task.
             # The flag is sent only when the drone is in wait mode
            start_optimization_task = wait_for_opt_task()
            #start_optimization_task = True
            if start_optimization_task == True:
               
                opt_flag = receive_optimization_flag()


                #Use Manual inserted Parameters while waiting the optimization
                THERMO.th_0 = rospy.get_param("/THERMO_params/th_0")
                THERMO.th_1 = rospy.get_param("/THERMO_params/th_1")
                THERMO.th_2 = rospy.get_param("/THERMO_params/th_2")
                THERMO.th_3 = rospy.get_param("/THERMO_params/th_3")
                THERMO.th_4 = rospy.get_param("/THERMO_params/th_4")
                
                THERMO.XT2_FOV = rospy.get_param("/THERMO_params/XT2_FOV")
                #Upload both images to make the user able to choice one set of paramters 
            
                filename_opt = user_clustered_image_path + '/Optimized_parameters_img/THERMO_opt_img' + '_1' + ext
                if (os.path.exists(filename_opt) == True):
                   
                    print(filename_original)
                    cv2.imwrite(filename_original, clustered_image)
                 
                    if opt_flag == True:

                        time.sleep(1)
                        THERMO.optimized_param = receive_Thermo_opt_parameters()
                        if (THERMO.optimized_param == [0,0,0]):
                            #metodo di default per caricare i parametri inseriti dall'utente inizialmente
                            THERMO.optimized_param[0] =  THERMO.th_0
                            THERMO.optimized_param[1] =  THERMO.th_1[0]
                            THERMO.optimized_param[2] =  THERMO.th_1[1]
                            THERMO.optimized_param[3] =  THERMO.th_2
                            THERMO.optimized_param[4] =  THERMO.th_3
                            THERMO.optimized_param[5] =  THERMO.th_4 
                        THERMO.optimized_param = np.asarray(THERMO.optimized_param)   

                        show_images(THERMO, filename_original, filename_opt, drone_obj, clustered_image)
                        if THERMO.selection == 1:
                            #User has selected the maunual Parameters 
                            THERMO.th_0 = rospy.get_param("/THERMO_params/th_0")
                            THERMO.th_1 = rospy.get_param("/THERMO_params/th_1")
                            THERMO.th_2 = rospy.get_param("/THERMO_params/th_2")
                            THERMO.th_3 = rospy.get_param("/THERMO_params/th_3")
                            THERMO.th_4 = rospy.get_param("/THERMO_params/th_4")
    
                            THERMO.parameters_optimization_enabled = False
    
                        elif THERMO.selection == 2:
                            #User has selected the Optimized Parameters 
                            THERMO.th_0  = THERMO.optimized_param[0]
                            th_1_low = THERMO.optimized_param[1]
                            th_1_high =  THERMO.optimized_param[2]
                            THERMO.th_2  = THERMO.optimized_param[3]
                            THERMO.th_3  = THERMO.optimized_param[4]
                            THERMO.th_4  = THERMO.optimized_param[5]
    
                            print("PUBLISH THERMAL PARAMETERS --> th1_l: {}, th1_h:{}, distance:{}".format( th_1_low, th_1_high, THERMO.th_2))
                            start_optimization_task = False 
                        else:
                            #Selection not valid --> use Manual Pramaters
                            print("Selection Not valid---> Keep Manual Param")
    
                        #Wait User Input 
                        opt_flag = False
        else:
            THERMO.th_0 = rospy.get_param("/THERMO_params/th_0")
            THERMO.th_1 = rospy.get_param("/THERMO_params/th_1")
            THERMO.th_2 = rospy.get_param("/THERMO_params/th_2")
            THERMO.th_3 = rospy.get_param("/THERMO_params/th_3")
            THERMO.th_4 = rospy.get_param("/THERMO_params/th_4")


        #Add estimated KF line before publishing the image 
        #if THERMO.counter_frame_analyzed > 0:
            # P1 = receive_estimated_control_point_P1()   
            # P2 = receive_estimated_control_point_P2()   
            # if THERMO_width_selected_flag == False:
            #    THERMO.desired_mask_width = THERMO.mask_width  #-----> TOP: SALVARE QUELLO DOPO OTTIMIZZAZIONE 
            #    THERMO_width_selected_flag = True
            # evaluate_altitude_offset(THERMO, THERMO_width_selected_flag)
        #    clustered_image = draw_estimated_line(P1, P2, clustered_image, drone_obj)

         #Write on txt file 
        # file1.write(str(counter_frame_received ) + "," + str(counter_frame_analyzed)+ "\n") 
        # file1.flush()
        
        # imgplot = plt.imshow(clustered_image,cmap='gray', vmin = 0, vmax = 255)
        # plt.show()  
        #Publish output frame in rostopic
        
        
        
     
        
        if image_received_flag == True:
            publish_output_image(clustered_image)
            line_versors_old = line_versors
            count = count + 1
        else:
            print("Impossible to output RGB images")
            count = count + 1
            if count > 10:
                break
     

        end2= time.time() - start1
        # file2.write(str(end2 ) + "\n") 
        # file2.flush()
         
       
        rospy.loginfo('MAIN LOOP: Loop time: %f ', 1/end2)
        #cv2.imshow('image', clustered_image)
        #cv2.waitKey(1)
    # imgplot = plt.imshow(image,cmap='gray', vmin = 0, vmax = 255)
    # plt.show()     
    #cluster_image = image_pre_processing(image)
        THERMO.empty_lists()
        line_versors_old = line_versors
        count = count + 1
        # start2 = time.time()
        # rospy.Rate(0.2).sleep()  # 1 Hz
        # end3= time.time() - start2
        # print('#####################end3: ', end3)
      
   # cv2.destroyAllWindows()    
  


if __name__ == '__main__':
    listener()
  
