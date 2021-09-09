#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import sys

import cv2
import time
import os
import math
import argparse
import imutils
import itertools
from threading import Thread


import networkx 
from networkx.algorithms.components.connected import connected_components
import os.path

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from ros_simulation_data import take_drone_camera_RGB_frame, takeEnvObservations, receive_estimated_control_point_P1, receive_estimated_control_point_P2,take_drone_camera_RGB_frame_simulation
from ros_simulation_data import publish_navigation_RGB_point
from ros_simulation_data import publish_output_image,  publish_opt_image,  publish_img_hsv, publish_img_first_threshold
from ros_simulation_data import publish_output_RGB_image, receive_flag_user_control, publish_altitude_offset, receive_flag_KF_init ,receive_HSV_parameters
from ros_simulation_data import wait_for_opt_task, receive_optimization_flag, publish_optimization_completed, receive_inNav_flag


from Image_RGB_detection import Image_RGB_elaboration



number = 0 #Global Varaible

class drone(object):
    #def __init__(self, pose, altitude):
    def __init__(self):
        #Store position data
        self.x = 0 #pose.point.x
        self.y = 0 # pose.point.y
        self.z = 15 # #pose.point.z


#############  TAke drone status information ###################
def takedronedata():
    # poseData, imuData, velData, altitudeVelDrone = takeEnvObservations()
    #poseData = takeEnvObservations()
    drone_obj = drone()#poseData)
    return drone_obj





def image_pre_processing(RGB, image, line_versors_old, drone_obj):
    
    pub_debug_img = True
    RGB.th3 = 1500
    RGB.th4 = 0.01  
    RGB.th5 =  20000 #51200
    
    # if RGB.parameters_optimization == True and len(RGB.optimized_param) > 0:
    #     RGB.optimized_param = np.asarray(RGB.optimized_param)
    #     print("RGB.optimized_param: ", RGB.optimized_param[0]) 
    #     H_MIN = RGB.optimized_param[0]
    #     S_MIN = RGB.optimized_param[1]
    #     V_MIN = RGB.optimized_param[2]
    #     RGB.optimized_param_old = RGB.optimized_param

    # else:
    #     H_MIN = rospy.get_param("/RGB_params/H_MIN")
    #     S_MIN = rospy.get_param("/RGB_params/S_MIN")
    #     V_MIN = rospy.get_param("/RGB_params/V_MIN")
    
    H_MAX = rospy.get_param("/RGB_params/H_MAX")
    S_MAX = rospy.get_param("/RGB_params/S_MAX")
    V_MAX = rospy.get_param("/RGB_params/V_MAX")


    RGB.resize_percent = 60
    image = resize_image(image, RGB.resize_percent)
    
    
    #image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) 
    
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
  
    # cv2.imshow('First Threshold', img_hsv)
    # cv2.waitKey(1)
    # imgplot = plt.imshow(img_hsv)
    # plt.show()

    lower_blue = np.array([RGB.H_MIN, RGB.S_MIN, RGB.V_MIN], np.uint8)  #Valori pannello in lab [80,0,40]
    upper_blue = np.array([H_MAX,S_MAX,V_MAX], np.uint8) #[160,100,220]

     # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(img_hsv, lower_blue, upper_blue)
    blur1 = cv2.bilateralFilter(mask,15,75,75)
    

    if pub_debug_img:
        publish_img_hsv(img_hsv)
        publish_img_first_threshold(mask)
    # imgplot = plt.imshow(mask,cmap='gray', vmin = 0, vmax = 255)
    # plt.show()

    thresh = cv2.convertScaleAbs(blur1)
    # cv2.imshow(' Threshold', thresh)
    # cv2.waitKey(1)
    # imgplot = plt
    
    
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    #Secondo set di parametri 
    new_region = []
    final_countours = []
    clusters = []
    same_mask = []
    flag_difference = False
    line_versors = []
    line_versors_final = []
    RGB.first_cluster_search(contours, thresh, image)
    
    if len(RGB.start_point) == 1:
        cluster = [[0]]
    else:
        #CLUSTERING LINES 
        cluster , theta_mean = RGB.clustering_lines(thresh.shape[0], thresh.shape[1])  
    
    
    ### Create a mask for each cluster, Draw Contours and fit line for each detected shape in the cluster 
    line_versors, mask, image = RGB.create_cluster_mask(cluster, image, thresh, drone_obj)
    
    
   
    
   

    _, mean_distance_of_line_from_center_line, line_points = RGB.voting_technique(line_versors, line_versors_old, thresh)
    
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
        
        # print("min_line: ", min_line)
        # print("line_points: ", line_points)
        # print("index: ", index)
        line_considered = line_points[index]
        print("[RGB DETECTION] mean line distance from center image: ",    mean_distance_of_line_from_center_line[index][0])

        # print("########################line_considered: ", line_considered)
         ############# DEBUGGING TXT FILES ##########
        # print("######################## mean_distance_of_line_from_center_line: ", mean_distance_of_line_from_center_line[index])
        
         ########## Exportation del punto (x,y) della retta e del punto Pv = (x + vx, y + vy) nel body frame del drone (considerando Ar drone 2.0)
        RGB.line_point_exportation_in_drone_body_frame(line_considered[0],line_considered[1],line_considered[2],line_considered[3], (thresh.shape[0]/2), (thresh.shape[1]/2), drone_obj, first = True)  #Definsico locazione del body frame del drone rispetto l'image plane (esattamente al centro )
        RGB.flag_final_frame_analyzed = True
        RGB.counter_frame_analyzed = RGB.counter_frame_analyzed + 1
    #Op.write_txt_file(file1, Op2, step) 
    # cv2.imshow('image', image)
    # cv2.waitKey(1)  
    return image, line_versors      






def load_RGB_image(photo_number):
     #name = '/home/lucamora/image_analysis/Volo_01_RGB/DJI_00' + str(photo_number) + '.jpg'
     name = '/home/dji/DATA/dji_ws/src/solar_project/src/solar_project/test.jpeg'
     image = cv2.imread(name) 
     
     #cluster_image = image_pre_processing(image,photo_number, th3_value, th4_value, rotation_degree)
     
     return image



def resize_image(image, percentage):
    #Resize Image
    width = int(image.shape[1] * percentage/ 100)
    height = int(image.shape[0] * percentage/ 100)
    dim = (width, height)
    
    image_res = cv2.resize(image, dim, interpolation =cv2.INTER_AREA)

    return image_res 
    

def evaluate_altitude_offset(RGB):
    print("RGB.desired_mask_width : ", RGB.desired_mask_width )
    print("RGB.mask_width : ", RGB.mask_width)
    RGB.mask_width_array.append(RGB.mask_width)
    value = RGB.desired_mask_width; #Default nel caso non venisse rilevato niente
    if (len( RGB.mask_width_array) > 20 and RGB.desired_mask_width > 0):
        value = np.mean(RGB.mask_width_array)
        RGB.mask_width_array.pop(0)
        print("MEAN OFFSET VALUE: ", value)
    offset = (RGB.desired_mask_width -value)
    if ( RGB.desired_mask_width == 0):
        print("ERROR!!! Heigth respect images not available ")
        offset = 0
    return offset 


def listen_keyboard_input(RGB, dir_opt):
    
    #dir_opt = '/home/dji/DATA/dji_ws/user_image_selection_after_opt/Optimized_parameters_img/RGB_opt_img.jpg'
    Image2 = cv2.imread(dir_opt)
    
    for ii in range(0, 50):
        publish_opt_image(Image2)
        time.sleep(0.2)
   
    print("[RGB DETECTION] Please Select a Picture to continue: 1 - LEFT, 2 - RIGHT")
    print("[RGB DETECTION] Press Enter to continue after the picture selection")
    RGB.selection = input("Select the desired set of Parameters")
    print("[RGB DETECTION]  arg.selection: ", RGB.selection)
    plt.close('all')
    return

    
def show_images(RGB, dir_manual, dir_opt, drone_obj, clustered_image):

   #Create a thread to take input from keyboard
    try:
        thread = Thread(target = listen_keyboard_input, args = (RGB, dir_opt, ))
        thread.start()
        #thread.join()
    except:
        print("[RGB DETECTION] Impossible to create the thread to listen the user input")

    
    print("[RGB DETECTION] WAIT UNTIL A SELECTION IS MADE")
    
    while(RGB.selection == 0):
        #Mentre l'user fa la scelta del sert di parametri da utilizzare, 
        #il while continua a pubblicare l'immagine clusterizzata con i parametri precedenti
        publish_output_RGB_image(clustered_image)
        time.sleep(0.1)
    
     









def listener():
    rospy.init_node('RGB_frame_elaboration', anonymous=True)
    #Create Image Publisher
    Camera_elaborated_frame_pub = rospy.Publisher('camera_vision_output_RGB', Image, queue_size=10)
    
    RGB = Image_RGB_elaboration()
   
    #OPen TXT file to save pixels 
    outFile1= open("/home/dji/DATA/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/pixel_width.txt","w+")
    #outFile2 = open("/home/dji/DATA/dji_ws/simulation_data/Test_simulazione_Paper/counter_frame_received.txt", "w+")
    #Save clustered image to give user ability to select it
    user_clustered_image_path = '/home/dji/DATA/dji_ws/user_image_selection_after_opt'

  
       
    condition = 10000
    count = 0
    counter_initialization_pixel = 0
    line_versors_old = []
    RGB_width_selected_flag = True
    Matrice = False #True if The matrice is connected in DJI sim
    RGB.parameters_optimization = True ##True if The param opt is available
    start_optimization_task = False #flag linked to RGB RGB.parameters_optimization. If RGB.parameters_optimization == True then start_optimization_task = True
    
    #Import ROSParameters
    Matrice = rospy.get_param("/RGB_params/Matrice")
    RGB.XT2_FOV = rospy.get_param("/RGB_params/XT2_FOV")

    RGB.parameters_optimization = rospy.get_param("/RGB_params/parameters_optimization")
    simulation_mode_enabled = rospy.get_param("/RGB_params/simulation_mode_enabled")
    #Path to save manual image 
    
    ext = '.png'
    filename_original = user_clustered_image_path + '/manual_parameters_img/default_param_img' + ext
    
    counter_frame_received = 0
    image_received_flag = False
    opt_flag = True
    start_navigation_task_flag = False  #True when the user has selected an image, permits the drone to stat navigation task
   
    while(count < condition):
        #Save all informations in drone struct 
        start1 = time.time()
        drone_obj = takedronedata()
        
        # print('end1: ', end1)
        #Take image frame
     
        try:
            if simulation_mode_enabled == True:
                image = take_drone_camera_RGB_frame_simulation()
            else:
                image = take_drone_camera_RGB_frame()
            image_received_flag = True
          
        except: 
            print("[RGB INFO] Impossible to obtain RGB images")
            image_received_flag = False
        
        #image = load_RGB_image(24)
       
        clustered_image, line_versors = image_pre_processing(RGB, image, line_versors_old, drone_obj)
        
      
        offset = 0.0

        ###########------------------------->> ALTITUDE CONTROL IN REAL MATRICE ----------------------------
        #Add estimated KF line before publishing the image 
        if Matrice == True:
         
            # P1 = receive_estimated_control_point_P1()
            # P2 = receive_estimated_control_point_P2()
            data = receive_flag_user_control()
            if (data == True):
                RGB.desired_mask_width = RGB.mask_width  #-----> TOP: SALVARE QUELLO DOPO OTTIMIZZAZIONE 
                RGB_width_selected_flag = True
                print("[RGB DETECTION] data: ", data) 

                #Add central line in clustered images
                start_point = (clustered_image.shape[1]/2, 0)
                end_point = (clustered_image.shape[1]/2, clustered_image.shape[0])
                cv2.line(image,start_point,end_point,255,2) 
                clustered_image = cv2.line(clustered_image, start_point, end_point, [0,0,255], 2)
                 
            else:
                offset = evaluate_altitude_offset(RGB)
                print("offset: ", offset)
                publish_altitude_offset(offset)
        
        # elif(RGB_width_selected_flag == False):
        #     RGB.desired_mask_width = RGB.mask_width  #-----> TOP: SALVARE QUELLO DOPO OTTIMIZZAZIONE 
        #     print("Recording Desired Mask Width: ", RGB.desired_mask_width)
        #     if (RGB.counter_frame_analyzed > 25):
        #         RGB_width_selected_flag = True
        # else:
        #     if (RGB_width_selected_flag == True):
        #         offset = evaluate_altitude_offset(RGB)    
        #         publish_altitude_offset(offset)
        #         print("Publish Pixel OFFSET:  ", offset)

        #Reinitialize width detection for each new panel Array:

        if Matrice == True:
            KF_init_flag = receive_flag_KF_init()
           
            if  KF_init_flag == True:
                RGB.desired_mask_width = RGB.mask_width 
                counter_initialization_pixel = counter_initialization_pixel + 1
                RGB_width_selected_flag = False
                RGB.mask_width_array = []
                print("[RGB DETECTION] ---------> [INFO] REINITIALIZED DESIRED MASK WIDTH: {} ".format(RGB.desired_mask_width))
            else:
                offset = evaluate_altitude_offset(RGB)    
                publish_altitude_offset(offset)
                print("[RGB DETECTION] Desired RGB pixel width: ", RGB.desired_mask_width)
                print("[RGB DETECTION] Publish Pixel OFFSET:  ", offset)
   
#######################   Parameters Optimization ############
       
        if RGB.parameters_optimization == True:
            start_optimization_task = wait_for_opt_task()
            
            if start_optimization_task == True:
                # RGB.optimized_param = receive_HSV_parameters()
                # RGB.optimized_param = np.asarray(RGB.optimized_param)
                opt_flag= receive_optimization_flag()
                
                #Use Manual inserted Parameters while waiting the optimization
                if len(RGB.optimized_param) > 0:
                    RGB.H_MIN = RGB.optimized_param[0]
                    RGB.S_MIN = RGB.optimized_param[1]
                    RGB.V_MIN = RGB.optimized_param[2]
                else:

                    RGB.H_MIN = rospy.get_param("/RGB_params/H_MIN")
                    RGB.S_MIN = rospy.get_param("/RGB_params/S_MIN")
                    RGB.V_MIN = rospy.get_param("/RGB_params/V_MIN")
                #Upload both images to make the user able to choice one set of paramters 
              
                filename_opt = user_clustered_image_path + '/Optimized_parameters_img/RGB_opt_img' + ext
                if (os.path.exists(filename_opt) == True):
                   
                    print(filename_original)
                    cv2.imwrite(filename_original, clustered_image)
                   
                if  opt_flag == True:
                    #wait some seconds to ob tain the real parametres from the optimization script 
                    time.sleep(1)
                    RGB.optimized_param = receive_HSV_parameters()
                    if (RGB.optimized_param == [0,0,0]):
                        #metodo di default per caricare i parametri inseriti dall'utente inizialmente
                        RGB.optimized_param[0] =  RGB.H_MIN
                        RGB.optimized_param[1] =  RGB.S_MIN
                        RGB.optimized_param[2] =  RGB.V_MIN
                    RGB.optimized_param = np.asarray(RGB.optimized_param)
                    #Upload both images to make the user able to choice one set of paramters 
            
                    show_images(RGB, filename_original, filename_opt, drone_obj, clustered_image)

                    if RGB.selection == 1:
                        #User has selected the maunual Parameters 
                        RGB.H_MIN = rospy.get_param("/RGB_params/H_MIN")
                        RGB.S_MIN = rospy.get_param("/RGB_params/S_MIN")
                        RGB.V_MIN = rospy.get_param("/RGB_params/V_MIN")
                        RGB.parameters_optimization = False
                    elif RGB.selection == 2:
                        #User has selected the Optimized Parameters 
                        RGB.H_MIN = RGB.optimized_param[0]
                        RGB.S_MIN = RGB.optimized_param[1]
                        RGB.V_MIN = RGB.optimized_param[2]
                        print("[RGB DETECTION] Parameters chosen, H: {}, S: {}, V: {}".format(RGB.H_MIN, RGB.S_MIN, RGB.V_MIN))
                        start_optimization_task = False 
                    else:
                        #Selection not valid --> use Manual Pramaters
                        print("[RGB DETECTION] Selection Not valid---> Keep Manual Param")
                    
                    start_navigation_task_flag = True
                    #Delete files from folder
                 
                    # os.remove("/home/dji/DATA/dji_ws/user_image_selection_after_opt/Optimized_parameters_img/RGB_opt_img.png")
                    # os.remove("/home/dji/DATA/dji_ws/user_image_selection_after_opt/Optimized_parameters_img/THERMO_opt_img_1.png")
                    #Wait User Input 
                    opt_flag = False
                    
        
        else: 
            RGB.H_MIN = rospy.get_param("/RGB_params/H_MIN")
            RGB.S_MIN = rospy.get_param("/RGB_params/S_MIN")
            RGB.V_MIN = rospy.get_param("/RGB_params/V_MIN")



        # if  opt_flag == True:
        #     #Upload both images to make the user able to choice one set of paramters 
        #     filename_opt = user_clustered_image_path + '/opt_img' + ext
        #     show_images(RGB, filename_original, filename_opt)
        #     if RGB.selection == 1:
        #         #User has selected the maunual Parameters 
        #         RGB.H_MIN = rospy.get_param("/RGB_params/H_MIN")
        #         RGB.S_MIN = rospy.get_param("/RGB_params/S_MIN")
        #         RGB.V_MIN = rospy.get_param("/RGB_params/V_MIN")
        #         RGB.parameters_optimization = False
                
        #     elif RGB.selection == 2:
        #         #User has selected the Optimized Parameters 
        #         RGB.H_MIN = RGB.optimized_param[0]
        #         RGB.S_MIN = RGB.optimized_param[1]
        #         RGB.V_MIN = RGB.optimized_param[2]
        #         print("Parameters chosen, H: {}, S: {}, V: {}".format(RGB.H_MIN, RGB.S_MIN, RGB.V_MIN))
        #         start_optimization_task = False 
        #     else:
        #         #Selection not valid --> use Manual Pramaters
        #         print("Selection Not valid---> Keep Manual Param")
        #     #Wait User Input 
        #     opt_flag = False       
        
                
        #Publish to solar flighth control that the optimization has been completed and the drone should satrt to navigate along the solar array        
        publish_optimization_completed(start_navigation_task_flag)
            
        outFile1.write(str(RGB.desired_mask_width) + "," + str(RGB.mask_width) + "\n")
        outFile1.flush()
        
        # if (receive_inNav_flag() == True):
        #     outFile2.write(str(counter_frame_received) + "\n")
        #     outFile2.flush()
        #     counter_frame_received = counter_frame_received + 1
        

        #    clustered_image = draw_estimated_line(P1, P2, clustered_image, drone_obj)

         #Write on txt file 
        # file1.write(str(counter_frame_received ) + "," + str(counter_frame_analyzed)+ "\n") 
        # file1.flush()
        
        
        #Publish output frame in rostopic
        # imgplot = plt.imshow(clustered_image,cmap='gray', vmin = 0, vmax = 255)
        # plt.show()  

      

       
        if image_received_flag == True:
            publish_output_RGB_image(clustered_image)
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
        RGB.empty_lists()
        RGB.flag_final_frame_analyzed = False
        
       



       
        # start2 = time.time()
        # rospy.Rate(0.2).sleep()  # 1 Hz
        # end3= time.time() - start2
        # print('#####################end3: ', end3)
      
   # cv2.destroyAllWindows()    
  


if __name__ == '__main__':
    listener()
