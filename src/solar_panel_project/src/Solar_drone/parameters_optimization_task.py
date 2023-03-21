#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np
from scipy.optimize import minimize, LinearConstraint, fmin_l_bfgs_b, basinhopping, brute

import operator

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import sys
from PIL import Image
from scipy import ndimage
import cv2
import time
import os
import math
import argparse
import imutils
import itertools


import networkx 
from networkx.algorithms.components.connected import connected_components

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from parameters_optimization.shape_detector import ShapeDetector
from parameters_optimization.RGB_class_optimization import Optimizer
from parameters_optimization.Image_RGB_detection import Image_RGB_elaboration
from parameters_optimization.Thermo_image_detection import Image_THERMO_elaboration
from parameters_optimization.Thermo_image_detection import Formatter as Formatter

from ros_simulation_data import take_drone_camera_RGB_frame, take_drone_camera_frame, takeEnvObservations
from ros_simulation_data import  receive_flag_user_control
from ros_simulation_data import wait_for_opt_task, publish_HSV_parameters, publish_optimization_flag, publish_Thermo_parameters


"""
Lo script contiene l'algoritmo di ottimizzazione necessario per 
ottimizzare i 3 parametri low relativi al range di HSV.
"""

number = 44
ext = '.JPG'
#path = '/home/lucamora/image_analysis/Volo_01_RGB/DJI_00'
path = '/home/lucamora/Desktop/Optimization_dataset/RGB/DJI_00'


class drone():
    def __init__(self):
        #Store position data
        self.x = 0.0
        self.y = 0.0
        self.z =  25.0




class RandomDisplacementBounds(object):
    """random displacement with bounds"""
    def __init__(self, xmin, xmax, stepsize):
        self.xmin = xmin
        self.xmax = xmax
        self.stepsize = stepsize

    def __call__(self, x):
        """take a random step but ensure the new position is within the bounds"""
        while True:
            # this could be done in a much more clever way, but it will work for example purposes
            
            if np.random.random() < 0.5:
                x= x + self.stepsize #np.random.uniform(-self.stepsize, self.stepsize, np.shape(x))
              
            else:
                x = x - self.stepsize
              
           
            if np.all(x< self.xmax) and np.all(x > self.xmin):
                break
           
        return x




def takedronedata():
    # poseData, imuData, velData, altitudeVelDrone = takeEnvObservations()
    
    drone_obj = drone()
    return drone_obj




def resize_image(image, percentage):
    #Resize Image
    width = int(image.shape[1] * percentage/ 100)
    height = int(image.shape[0] * percentage/ 100)
    dim = (width, height)
    
    image_res = cv2.resize(image, dim, interpolation =cv2.INTER_AREA)

    return image_res 






def image_pre_processing(Op, sd, RGB, image, drone_obj):

    RGB.th3 = 1500
    RGB.th4 = 0.01  
    RGB.th5 =  20000 #51200
    step = Op.step
    
    #Upload default parameters 
    H_MIN_default = rospy.get_param("/RGB_params/H_MIN")
    S_MIN_default = rospy.get_param("/RGB_params/S_MIN")
    V_MIN_default = rospy.get_param("/RGB_params/V_MIN")
    H_MAX_default = rospy.get_param("/RGB_params/H_MAX")
    S_MAX_default = rospy.get_param("/RGB_params/S_MAX")
    V_MAX_default = rospy.get_param("/RGB_params/V_MAX")

    #publish default parameters while advence in optimization 
    publish_HSV_parameters(H_MIN_default, S_MIN_default, V_MIN_default)

    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

    lower_blue = np.array([Op.h_low, Op.s_low, Op.v_low], np.uint8)  #Valori pannello in lab [80,0,40]
    upper_blue = np.array([H_MAX_default,S_MAX_default,V_MAX_default], np.uint8)#[255,180,220] #[160,100,220]

     # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(img_hsv, lower_blue, upper_blue)
    blur1 = cv2.bilateralFilter(mask,15,75,75)
    
    thresh = cv2.convertScaleAbs(blur1)
    # cv2.imshow('First Threshold', thresh)
    # cv2.waitKey(1)
    
    
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
    if (Op.test == True):
        corr_mask, line_versors, mask,line_versors_for_opt,  final_image = RGB.create_cluster_mask(Op, sd, cluster, image, thresh, drone_obj)
        Op.corr_mask = corr_mask
    else:
        line_versors, mask,line_versors_for_opt,  final_image = RGB.create_cluster_mask(Op, sd, cluster, image, thresh, drone_obj)
        
    # fig, ax = plt.subplots()
    # imgplot = plt.imshow(final_image,cmap='gray', vmin = 1, vmax = 255)
    # ax.format_coord = Formatter(imgplot)
    # plt.show()
    
    #Nello script con due istanze:
    #---> per h,s,v di img1 
    #--> calcolo cost_function per img1
    #---> chiamo iterativamente hsv per img2
    #---> per ciascuna iterazione hsv per img2 calcolo costo 
    #---> sommo il costo di ciascuna iterazne hsv img2 con il costo calcilato per hsv img1
    #--> aggiungo pesato il coefficiente di correlazione 
    
    
    #_-------------------- COMPUTE COST FUNCTION -------------
    #Op.compute_cost_function(sd) #without correlation ---> Salva in Op.cost
    Op.compute_cost_RGB_function(sd, line_versors_for_opt)

    # point_line_distance_actual, mean_distance_of_line_from_center_line, line_points = RGB.voting_technique(line_versors, line_versors_old, thresh)
    # #### EValuate mean distance pf the line from the center line 
    # mean_distance_of_line_from_center_line.append(np.mean(point_line_distance_actual)) 
    
    # # a = math.sqrt(pow(x-thresh.shape[0]/2,2 ) + pow(y-thresh.shape[1]/2,2))
    # # cartesian_distance_point_from_image_center.append(a)
    
    
    # try:      
    #    cv2.line(image,(thresh.shape[1]-1,righty),(0,lefty),255,2)   
    # except:
    #     pass
    # # print('cartesian_distance_point_from_image_center: ', cartesian_distance_point_from_image_center)
   
    
    # if (len(mean_distance_of_line_from_center_line) > 0):
    # # if (len(cartesian_distance_point_from_image_center) > 0):
    #     #Ordering vector from lowest to highest value
          
    #     min_line = min(mean_distance_of_line_from_center_line)

    #      #Take the index of the first line 
    #     try:
    #         index = mean_distance_of_line_from_center_line.index(min_line)
    #     except:
    #         index = 1 #Prende la rpima linea se non trova l'index desiderato
        
        
    #     line_considered = line_points[index]
    #     # print("########################line_considered: ", line_considered)
    #      ############# DEBUGGING TXT FILES ##########
    #     # print("######################## mean_distance_of_line_from_center_line: ", mean_distance_of_line_from_center_line[index])
        
    #      ########## Exportation del punto (x,y) della retta e del punto Pv = (x + vx, y + vy) nel body frame del drone (considerando Ar drone 2.0)
    #     RGB.line_point_exportation_in_drone_body_frame(line_considered[0],line_considered[1],line_considered[2],line_considered[3], (thresh.shape[0]/2), (thresh.shape[1]/2), drone_obj)  #Definsico locazione del body frame del drone rispetto l'image plane (esattamente al centro )
    #     flag_final_frame_analyzed = True


    #Call image detectector per hsv img 2
    #image2 = image2_pre_processing(mask.copy(), Op, Op2, drone_obj, step)

    sd.coo_list = []
    sd.area_list = []
    
    #Op.write_txt_file(file1, Op2, step)   
    return image      




def load_image(photo_number, path, ext):
     #name = path + str(photo_number) + ext
     name = path + str(photo_number) + ext
     #name = '/home/lucamora/Desktop/parameters_optimization/Panels_effects/evening_sunlight' + '.png'
     image = cv2.imread(name) 
     
     #cluster_image = image_pre_processing(image,photo_number, th3_value, th4_value, rotation_degree)
     
     return image






def create_boundaries(var):
    n_var = len(var)
    possible_values =np.array(range(0,n_var))
    x_min = np.array(range(0,n_var))
    x_max = np.array(range(0,n_var))
    bnds = np.array(range(0,n_var), dtype=object)

    for ii in range(0, len(possible_values)):
        possible_values[ii] = 255
        x_min[ii] = 0
        x_max[ii] = 255
        bnds[ii] = (0,255)
    print("bnds: ", bnds)
    return x_min.tolist(), x_max.tolist(), bnds


def find_initial_values(options, init_grid, var):
    """
    FUNCTION TO INITIALIZE OPTIMIZATION ALG:
    Permits to subdivide the range of parameters in an initiali grid.
    Given all the permutation it is evaluated the cost function in each cell of the grid.
    The algorithm is initialized in the cell where the initialization cost is higher.

    INPUT: 
    optsions: class and value to give as input to cluster
    n_var: number of variable to optimize between HSV
    init_grid: number of ceoll for each range. Example 25 --> 255/25
    var: variable to optimize between 'H', 'S', 'V'
    """
    Op = options[0]
    sd = options[1]
    RGB = options[2]
    image = options[3]
    drone_obj = options[4]
    RGB_opt_flag = options[5]

    n_var = len(var)
    
    value = 255/init_grid #Value correspondant in HSV to the index
    a = [range(0,value)] # Creo un vetrtore di lunghezza pari a init_grid

    c = list(itertools.product(a[0], repeat = n_var)) #creo lista delle possibili combinazioni di indici ex: (1,1,1), (1,1,2), (1,1,3).. celle da verificare
    
    #cost_array = np.empty((init_grid**n_var, 1), dtype=object) #Per ciascuna cella della griglia eseguo la funzione di costo e la salvo nell'array
    cost_array = []
    parameters = []
    #Find the Obj value of the function in each grid
    #Scorro tutte le combinazioni in c per la prima variabile
    for ii in range(0, len(c)):
        #cambio variabile
        for jj in range(0, len(var)):
            if var[jj] == 'h':
                Op.h_low= c[ii][jj]*init_grid #assegno valore di h moltiplicato per l'indice di divisione
            elif var[jj] == 's':
                Op.s_low = c[ii][jj]*init_grid #assegno valore di s moltiplicato per l'indice di divisione
            else:
                Op.v_low = c[ii][jj]*init_grid #assegno valore di v moltiplicato per l'indice di divisione

       
        print("H: {}, S: {}, V: {}".format(Op.h_low, Op.s_low, Op.v_low))   

        # Op2.h_low  = 0
        # Op2.s_low = 0  
        clustered_image = image_pre_processing(Op, sd, RGB, image, drone_obj)
        
        if Op.cost >= 1.0:
            Op.cost = 0.0
        cost_array.append(Op.cost)
        #cost_array[ii][0] = Op2.cost 
    #print(cost_array)
    #Search the maximum cost value and its position index in the array
    max_value = max(cost_array)
    index = cost_array.index(max_value)
    #max_value  = np.amax(cost_array)
    #index = np.where(cost_array == max_value)
    #print("index: ", index)
    #Itero sulla lunghezza degli indici selezionati come massimi (nel caso ce ne fossero piu di 1 a parimerito)
    #for ii in range(0, len(index)):
        
    parameters.append(c[index]) 
    
    parameters = np.array(parameters)
    initial_guess = []
    #Moltiplico parameters per initgrid
    for ii in range(0, len(parameters)):
        for jj in range(0, len(parameters[ii])):
            parameters[ii][jj] =  parameters[ii][jj]*init_grid
        initial_guess.append( parameters[ii])
    # print("parameters: ", parameters)
    print("initial_guess: ", initial_guess)
    #Moltiplicare parameters per value per avere il valore del aprametro
    #creare initial_guess = [,,]

    return initial_guess



def Obj_func(x, Op, sd, RGB, image, drone_obj, var):
    if len(var) == 1:
        Op.h_low = x[0]
        print("H: {}".format(Op.h_low)) 
    elif len(var) == 2:
        Op.h_low = x[0]
        Op.s_low = x[1]
        print("H: {}, S: {}".format(Op.h_low, Op.s_low)) 
    else:
        Op.h_low = x[0]
        Op.s_low = x[1]
        Op.v_low = x[2]
        print("H: {}, S: {}, V: {}".format(Op.h_low, Op.s_low, Op.v_low)) 
    
    
    clustered_image = image_pre_processing(Op, sd, RGB, image, drone_obj)
    publish_optimization_flag(flag = False)
    s = Op.cost
    
    print("OBJ FUNCTION: {} ".format(-1*s)) #sum(Op2.final_cost_list)))
    return -1*s




def  RGB_optimization(args):
    
    RGB = args[0] 
    sd = args[1]
    Op = args[2]
    drone_obj = args[3]
    var = args[4]
    init_grid = args[5]
    n_better_solutions = args[6]
    RGB_image = args[7]
    RGB_opt_flag = args[8]
    #path = '/home/lucamora/image_analysis/Volo_01_RGB/DJI_00'
    #image = load_image(number,path, ext)
    
    
    RGB.resize_percent = 70
    image = resize_image(RGB_image, RGB.resize_percent)
   
    Op.step = 5
    
    

    #Fix variable outside the Optimization
    Op.h_low = 90
    Op.s_low = 70
    Op.v_low = 15

    
    
    #cons = ({'type': 'eq', 'fun': lambda x:  x[0] - 2 * x[1] + 2})
    possible_values = np.array(range(0, 255, Op.step))
    
    xmin, xmax, bnds = create_boundaries(var)
    # xmin = [0, 0]
    # xmax = [255, 255]
    
    # bx0 = (0, 255)
    # bx1 =  (0, 255)
    # bx2 =  (0, 255)
    # bnds = (bx0, bx1)
    # bnds = [((0, n) for n in possible_values )]
    print("xmin: {}, xmax: {}, bnds: {} ".format(xmin, xmax, bnds) )
   
    
    #Initiliaze OPtimization Algorithm and variables
    args = (Op, sd, RGB, image, drone_obj,RGB_opt_flag)
    initial_guess_list = find_initial_values(args, init_grid, var) 
    
    
    initial_guess =initial_guess_list[0] 
    print("initial_guess: ", initial_guess)
    #initial_guess=  [60, 60]  #np.random.randint(0,255/Op.step, 1) 
    # #---> Variabile riferita ad Op2.h_low
    take_step = RandomDisplacementBounds(xmin, xmax, Op.step)
    minimizer_kwargs = dict(method="L-BFGS-B", bounds=bnds, args=(Op, sd, RGB, image, drone_obj,var,))
    res = basinhopping(Obj_func, x0=initial_guess, minimizer_kwargs=minimizer_kwargs,  take_step=take_step)
    
    print("res: ", res)
    RGB_opt_flag = True
    # print("global minimum: x = [%.4f, %.4f, %.4f], f(x0) = %.4f" % (res.x[0], res.x[1], res.x[2],
    #                                                       res.fun))
    Op.test = True
    if (Op.test == True):
        Op.h_low = res.x[0]
        Op.s_low = res.x[1]
        Op.v_low = res.x[2]
        clustered_image = image_pre_processing(Op,sd, RGB, image, drone_obj)
        imgplot = plt.imshow(clustered_image,cmap='gray', vmin = 0, vmax = 255)
        plt.show()
        
        # imgplot = plt.imshow(Op.corr_mask,cmap='gray', vmin = 0, vmax = 255)
        # plt.show()
        save_path = '/home/lucamora/catkin_ws/user_image_dir/'
        filename = save_path + 'RGB_opt_img' + '.png'
        print(filename)
        cv2.imwrite(filename, clustered_image)

        

    #constraints=cons)
    
    # #Nella objective_function
    clustered_image = image_pre_processing(Op, sd, RGB, image, drone_obj)
    return clustered_image, res, RGB_opt_flag











#######################################################################################################################################################################
######################################## THERMAL CALIBRATION #####################################################################################à

def image_THERMO_pre_processing(Op_thermo, sd, THERMO, image_or, drone_obj, RGB_corr_template):
    ########## Rotation and resize of image ###########

    # if ( Op_thermo.correlation == True):
    #     template = Op_thermo.load_template(path_template)

   
    
    th_0_default = rospy.get_param("/THERMO_params/th_0")
    th_1_default = rospy.get_param("/THERMO_params/th_1")
    th_2_default = rospy.get_param("/THERMO_params/th_2")
    th_3_default = rospy.get_param("/THERMO_params/th_3")
    th_4_default = rospy.get_param("/THERMO_params/th_4")

    th_1_low_default = th_1_default[0]
    th_1_high_default = th_1_default[1]
  
    #Paramters not optimized
    THERMO.th_0 = th_0_default
    THERMO.th_3 = th_3_default
    THERMO.th_4 = th_4_default
###############################################
    
    THERMO.resize_percent = 70
    image = resize_image(image_or, THERMO.resize_percent)
    width = int(image.shape[1] *  THERMO.resize_percent/ 100)
    height = int(image.shape[0] *  THERMO.resize_percent/ 100)
    dim = (width, height)

    #publish default parameters while advence in optimization 
    args = (th_0_default, th_1_low_default, th_1_high_default, th_2_default, th_3_default, th_4_default)
    publish_Thermo_parameters(args)
    publish_optimization_flag(flag = False)

    #GRAYSCALE
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
   

    #Evaluate mean of all image pixel intensities
    intensity_mean = 0
    intensity_mean = np.mean(gray)
   # print("intensity_mean: ", intensity_mean)
   
    THERMO.original_image_shape[0] = gray.shape[0]
    THERMO.original_image_shape[1] = gray.shape[1]
    
    const_int_value = [ [ int(THERMO.th_0 - intensity_mean) for y in range( gray.shape[1] ) ] for x in range( gray.shape[0] ) ]
    
   
    const_int_value = np.array(const_int_value)
    gray = const_int_value + gray
    gray=gray.astype(np.float32)
    
    intensity_mean = np.mean(gray)
      
    blur1 = cv2.bilateralFilter(gray,3,75,75)
       
   
    if (Op_thermo.th_1_high <= Op_thermo.th_1_low):
        Op_thermo.cost = 0.0
        empty_lists(THERMO)
        return 
    
    print("TH1_low: {}, TH1_high: {}, D: {}".format( Op_thermo.th_1_low, Op_thermo.th_1_high, Op_thermo.th_2))
    
    ret,thresh_blur1 = cv2.threshold(blur1,Op_thermo.th_1_low,255,cv2.THRESH_TOZERO)
    ret,thresh_blur2 = cv2.threshold(thresh_blur1,Op_thermo.th_1_high ,255,cv2.THRESH_TOZERO_INV)
    
    # cv2.imshow('First Threshold', thresh_blur2)
    # cv2.waitKey(1)
    
    distance_matrix = ndimage.distance_transform_edt(thresh_blur2) 
    
    ret,thresh= cv2.threshold(distance_matrix,Op_thermo.th_2,255,cv2.THRESH_BINARY)
    # cv2.imshow('distance', thresh)
    # cv2.waitKey(1)
    
    # fig, ax = plt.subplots()
    # imgplot = plt.imshow(thresh,cmap='gray', vmin = 1, vmax = 255)
    # ax.format_coord = Formatter(imgplot)
    # plt.show
    thresh = cv2.convertScaleAbs(thresh)
    _ ,contours, _= cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    

    THERMO.first_cluster_search(contours, thresh, image)
    
    if len(THERMO.start_point) == 1:
        cluster = [[0]]
    else:
        #CLUSTERING LINES 
        cluster , theta_mean = THERMO.clustering_lines(thresh.shape[0], thresh.shape[1])  
    
     ### Create a mask for each cluster, Draw Contours and fit line for each detected shape in the cluster 
    line_versors, mask, line_versors_for_opt, image, corr_mask = THERMO.create_cluster_mask(cluster, image, thresh, drone_obj, sd)
    
    # if (Op_thermo.test == True):
    #     fig, ax = plt.subplots()
    #     imgplot = plt.imshow(image,cmap='gray', vmin = 1, vmax = 255)
    #     ax.format_coord = Formatter(imgplot)
    #     plt.show()
    
    
    # cv2.imshow('image', image)
    # cv2.waitKey(1)
    
    Op_thermo.template = RGB_corr_template
    Op_thermo.compute_cross_correlation(Op_thermo.template, corr_mask, cluster)
    #Compute COst FUnction
    
    Op_thermo.compute_cost_function(sd,line_versors_for_opt )
    
           #  Op.h_range_index.append(Op.h_range[Op.h_low]
    sd.coo_list = []
    sd.area_list = []

          
    # 
    # 
    # 
    #SVUOTO LISTE 
    empty_lists(THERMO)
    #Op.write_THERMO_txt_file(file1, step_TH1)
    return  image
         


def empty_lists(THERMO):
    THERMO.point_line_distance_actual = []
    THERMO.same_cluster = []
    THERMO.same_cluster_similar_direction = []
    THERMO.sum_list = []
    THERMO.new_list = []
    THERMO.count1 = 0 
    #General variables inside first cluster search
    THERMO.lefty_array = []
    THERMO.rigthy_array = []
    THERMO.start_point = []
    THERMO.end_point = []
    THERMO.points = []
    THERMO.angular_coefficient = []
    THERMO.actual_points = []
    THERMO.angular_line_versors = []
    THERMO.countours_considered = []
    THERMO.pixels_iniside_area = []
    THERMO.region_pixels_size_coo = [] # coordinate of min col min row pixel, of max col min row pixel and max col, min row pixel
    
    
    THERMO.counter_cnt = 0
    THERMO.count = 0

       






def create_thermo_boundaries(var):
    ''' 
    Create Boundaries for Thermal variables to optimize 
    '''
    n_var = len(var)
    possible_values =np.array(range(0,n_var))
    x_min = np.array(range(0,n_var))
    x_max = np.array(range(0,n_var))
    bnds = np.array(range(0,n_var), dtype=object)
    
    if (var[-1] == 'distance'):
        for ii in range(0, len(possible_values)):
            if (ii < len(var)-1):
                #Create Boundaries for th1_lo and th1_high
                possible_values[ii] = 255
                x_min[ii] = 0
                x_max[ii] = 255
                bnds[ii] = (0,255)
            else:
                possible_values[ii] = 20
                x_min[ii] = 2
                x_max[ii] = 20
                bnds[ii] = (2,20)
    else:
        for ii in range(0, len(possible_values)):
            #Create Boundaries for th1_lo and th1_high
            possible_values[ii] = 255
            x_min[ii] = 0
            x_max[ii] = 255
            bnds[ii] = (0,255)#(0,255)


   
    return x_min.tolist(), x_max.tolist(), bnds


def find_initial_THERMO_values(options, init_THERMO_grid, init_THERMO_grid_distance,  distance_max, var):
    """
    FUNCTION TO INITIALIZE OPTIMIZATION ALG:
    Permits to subdivide the range of parameters in an initiali grid.
    Given all the permutation it is evaluated the cost function in each cell of the grid.
    The algorithm is initialized in the cell where the initialization cost is higher.

    INPUT: 
    optsions: class and value to give as input to cluster
    n_var: number of variable to optimize between HSV
    init_grid: number of ceoll for each range. Example 25 --> 255/25
    var: variable to optimize between th1_low, th1_high, distance
    """

    #args of the function 
    Op_thermo = options[0]
    sd = options[1]
    THERMO = options[2]
    image = options[3]
    drone_obj = options[4]
    RGB_corr_template = options[5] 
    

    n_var = len(var)
    value = 255/init_THERMO_grid #Value correspondant in HSV to the index
    value_distance = distance_max/init_THERMO_grid_distance
    
    # value = 3
    # value_distance = 1
    # n_var = 3
    # Creo un vetrtore di lunghezza pari a init_grid
    a = [range(0,value)] 
    a1 = range(0,value_distance)
    print("a1: ", a1)

    #Define all the possible index combnation 
    if n_var == 3: #---> Nel caso sia presente anche la distanza
        c = list(itertools.product(a[0],  repeat = 2)) #com
        c_list = []
        for ii in range(0, len(c)):
            for jj in range(0, len(a1)):
                c_list_inter = []
                c_list_inter.append(c[ii][0])
                c_list_inter.append(c[ii][1])
                c_list_inter.append(a1[jj])
                #print("c_list_inter: ", c_list_inter)
                c_list.append(c_list_inter)
       
    else:
        #creo lista delle possibili combinazioni di indici ex: (1,1,1), (1,1,2), (1,1,3).. celle da verificare
        c = list(itertools.product(a[0], repeat = n_var)) 
        c_list = []
        for ii in range(0, len(c)):
            c_list.append(list(c[ii]))
    
    #print("c_list: ", c_list)
    # #cost_array = np.empty((init_grid**n_var, 1), dtype=object) #Per ciascuna cella della griglia eseguo la funzione di costo e la salvo nell'array
    cost_array = []
    parameters = []
    #Find the Obj value of the function in each grid
    #Scorro tutte le combinazioni in c per la prima variabile
    for ii in range(0, len(c_list)):
        #cambio variabile
        for jj in range(0, len(var)):
            if var[jj] == 'th1_l':
                Op_thermo.th_1_low= c_list[ii][jj]*init_THERMO_grid #assegno valore di h moltiplicato per l'indice di divisione
            elif var[jj] == 'th1_h':
                Op_thermo.th_1_high = c_list[ii][jj]*init_THERMO_grid #assegno valore di s moltiplicato per l'indice di divisione
            else:
                Op_thermo.th_2 = c_list[ii][jj]*init_THERMO_grid_distance #assegno valore di v moltiplicato per l'indice di divisione
               
        #Clacolo il valore di cost chiamando l'algoritmo di detetction
        print("TH1_l: {}, TH1_H: {}, TH2: {}".format(Op_thermo.th_1_low, Op_thermo.th_1_high, Op_thermo.th_2))   
   
        clustered_image = image_THERMO_pre_processing(Op_thermo, sd, THERMO, image, drone_obj, RGB_corr_template)
        
        cost_array.append(Op_thermo.cost)
        #cost_array[ii][0] = Op2.cost 
    #print(cost_array)
   
    #Search the maximum cost value and its position index in the array
    max_value = max(cost_array)
    index = cost_array.index(max_value) #--> trovo la griglia corrispondente agli indici relativi al valore dove la funzione di costi ha un picco
    
    
    parameters.append(c_list[index]) 
    
    parameters = np.array(parameters)
    initial_guess = []
    #Moltiplico parameters per initgrid --> Permette di avere il valore delle variabili e non l'indice.
    #Per esempio se init_grid = 50 --> index = 2 * 50 = 100 valore di th1 (low o high)
    for ii in range(0, len(parameters)):
        for jj in range(0, len(parameters[ii])):
            if (jj < 2):
                parameters[ii][jj] =  parameters[ii][jj]*init_THERMO_grid
            else:
                parameters[ii][jj] =  parameters[ii][jj]*init_THERMO_grid_distance

        initial_guess.append( parameters[ii])
    # print("parameters: ", parameters)
    print("initial_guess: ", initial_guess)
    #Moltiplicare parameters per value per avere il valore del aprametro
    #creare initial_guess = [,,]
    return initial_guess



def THERMO_Obj_func(x, Op_thermo, sd, THERMO, image, drone_obj, var, RGB_corr_template):
    """
    Function relative to the computation of the objective fun.
    The array var define the type of variables to optimize,
    chosen from th1_low, th1_max, distance.
    """
    if len(var) == 1:
        Op_thermo.th_1_low = int(x[0])
        Op_thermo.Optimization_parameters_array.append([Op_thermo.th_1_low])
    elif len(var) == 2:
        Op_thermo.th_1_low = int(x[0])
        Op_thermo.th_1_high= int(x[1])
        Op_thermo.Optimization_parameters_array.append([Op_thermo.th_1_low,  Op_thermo.th_1_high])
       
    else:
       Op_thermo.th_1_low = int(x[0])
       Op_thermo.th_1_high = int(x[1])
       Op_thermo.th_2 = int(x[2])
       Op_thermo.Optimization_parameters_array.append([Op_thermo.th_1_low,  Op_thermo.th_1_high,  Op_thermo.th_2])
 
    #print("TH1_l: {}, TH1_H: {}, TH2: {}".format(Op_thermo.th_1_low, Op_thermo.th_1_high, Op_thermo.th_2))   
    clustered_image = image_THERMO_pre_processing(Op_thermo, sd, THERMO, image, drone_obj, RGB_corr_template)
    
    publish_optimization_flag(flag = False)
    s = Op_thermo.cost
    #Append the value in an arry to find the best three solutions
    Op_thermo.Optimization_cost_final_array.append(s)
   
    print("OBJ FUNCTION: {} ".format(s)) #sum(Op2.final_cost_list)))
    return -1*s



def Thermal_optimization(args):
    """
    Function where the optimization structure is implemented 
    """
    THERMO = args[0] 
    sd_thermo = args[1]
    Op_thermo = args[2]
    drone_obj = args[3]
    var = args[4]
    init_THERMO_grid = args[5]
    init_THERMO_grid_distance = args[6]
    RGB_corr_template = args[7]
    n_better_solutions = args[8]
    image_thermal = args[9]
    Thermo_opt_flag = args[10]
     
    photo_number = 1
    ext = '.png'
    # photo_number = 522
    # ext = '.jpg'
    # path = '/home/lucamora/image_analysis/Volo_01/DJI_0'
    # image_thermal = load_image(photo_number,path, ext)
    
   
    
    #Define the Basin Hopper STep and the value peak of the gaussian in the cost function.
    #The eak of the gaussian is centered in the mean area of the created mask
    Op_thermo.step = 5 #Step for Basin Hopper alg during Opt
    Op_thermo.Thermo_gaussian_mean_cost_value = 0.05 #Cambiarlo per cambiare funzione di costo
    Op_thermo.Thermo_std = 0.2
    

    Op_thermo.th_0 = 125.647
    Op_thermo.th_1_low = 120 # da ottimizzare
    Op_thermo.th_1_high = 180
    Op_thermo.th_2 = 9
    distance_max = 20

    #Number of better solutio to found 
    n_better_solutions = 3
    #variables ---> 'th1_l', 'th1_h', 'distance' --> to optimiza

    xmin, xmax, bnds = create_thermo_boundaries(var)
    print("xmin: {}, xmax: {}, bnds: {} ".format(xmin, xmax, bnds) )
    
    #arg to pass to the obj function 
    args = (Op_thermo, sd_thermo ,THERMO, image_thermal, drone_obj,RGB_corr_template,)
    initial_guess_list = find_initial_THERMO_values(args, init_THERMO_grid, init_THERMO_grid_distance, distance_max, var) 
    
    initial_guess =initial_guess_list[0] 
    print("initial_guess: ", initial_guess)
    #initial_guess=  [60, 60]  #np.random.randint(0,255/Op.step, 1) 
    # #---> Variabile riferita ad Op2.h_low
    take_step = RandomDisplacementBounds(xmin, xmax, Op_thermo.step)
    minimizer_kwargs = dict(method="L-BFGS-B", bounds=bnds, args=(Op_thermo, sd_thermo, THERMO, image_thermal, drone_obj,var,RGB_corr_template, ))
    res = basinhopping(THERMO_Obj_func, x0=initial_guess, minimizer_kwargs=minimizer_kwargs,  take_step=take_step)
    
    print("res: ", res)
    Thermo_opt_flag = True
    # print("global minimum: x = [%.4f, %.4f, %.4f], f(x0) = %.4f" % (res.x[0], res.x[1], res.x[2],
    #                                                       res.fun))

    #Search the FIrst Three better solutions 
    final_solution_parameters, final_cost_solutions, message = find_better_solutions(Op_thermo, n_better_solutions, res)
    if message == 'Success':
        print("final_solution_parameters: ", final_solution_parameters)
        print("final_cost_solutions: ", final_cost_solutions)
        
        Op_thermo.test = True
        if  (Op_thermo.test == True):
            for ii in range(0, len(final_cost_solutions)):
                if (len(var) == 1):
                    Op_thermo.th_1_low = final_solution_parameters[ii][0]
                elif (len(var) == 2):
                    Op_thermo.th_1_low = final_solution_parameters[ii][0]
                    Op_thermo.th_1_high = final_solution_parameters[ii][1]
                else: 
                    Op_thermo.th_1_low = final_solution_parameters[ii][0]
                    Op_thermo.th_1_high = final_solution_parameters[ii][1]
                    Op_thermo.th_2 = final_solution_parameters[ii][2]
                
                clustered_image = image_THERMO_pre_processing(Op_thermo, sd_thermo, THERMO, image_thermal, drone_obj, RGB_corr_template)
               
                fig, ax = plt.subplots()
                imgplot = plt.imshow(clustered_image,cmap='gray', vmin = 1, vmax = 255)
                ax.format_coord = Formatter(imgplot)
                plt.show()
                save_path = '/home/lucamora/catkin_ws/user_image_dir/'
                filename = save_path + 'THERMO_opt_img'+ "_" + str(ii) + ext
                print(filename)
                cv2.imwrite(filename, clustered_image)
    else:
        Op_thermo.test = True
        if  (Op_thermo.test == True):
            
            if (len(var) == 1):
                Op_thermo.th_1_low = res.x[0]
            elif (len(var) == 2):
                Op_thermo.th_1_low = res.x[0]
                Op_thermo.th_1_high = res.x[1]
            else: 
                Op_thermo.th_1_low = res.x[0]
                Op_thermo.th_1_high = res.x[1]
                Op_thermo.th_2 = res.x[2]
            
            clustered_image = image_THERMO_pre_processing(Op_thermo, sd_thermo, THERMO, image_thermal, drone_obj, RGB_corr_template)
            
            fig, ax = plt.subplots()
            imgplot = plt.imshow(clustered_image,cmap='gray', vmin = 1, vmax = 255)
            ax.format_coord = Formatter(imgplot)
            plt.show()
            save_path = '/home/lucamora/catkin_ws/user_image_dir/'
            filename = save_path + 'THERMO_opt_img'+ "_" + '1' + ext
            print(filename)
            cv2.imwrite(filename, clustered_image)

    #Send True in optimization Complete 
    publish_optimization_flag(flag = True)

    # #Nella objective_function
    # clustered_image = image_pre_processing(Op, Op2, sd, RGB, image, drone_obj)
    return clustered_image
    
def remove_values_from_list(the_list, val):
   return [value for value in the_list if value != val]

def  find_better_solutions(Op, n_better_solutions, res):
    # Op.Optimization_cost_final_array = (Op.Optimization_cost_final_array))
    #Op.Optimization_parameters_array = np.array((Op.Optimization_cost_final_array))
    
    # print("Op.Optimization_cost_final_array : ",Op.Optimization_cost_final_array )
    # print("Op.Optimization_parameters_array : ",Op.Optimization_parameters_array )
    final_cost_solutions = []
    final_solution_parameters = []
    
    for ii in range(0,n_better_solutions):
        #FInd the max value in the solution array
        max_value = max(Op.Optimization_cost_final_array)
        index_of_maximum = np.where(Op.Optimization_cost_final_array == max_value)
        if (len(index_of_maximum[0]) > 1):
            new_index_of_maximum = index_of_maximum[0][0]
        else: 
            final_cost_solutions = res.fun
            final_solution_parameters = res
            message = 'Failed'
            break
        final_solution_parameters.append( Op.Optimization_parameters_array[new_index_of_maximum])
        final_cost_solutions.append(Op.Optimization_cost_final_array[new_index_of_maximum])
        
        
        
        #for jj in range(0, len(index_of_maximum[0])):
            # print("index_of_maximum[0][jj]: ", index_of_maximum[0][jj])
            # print("Op.Optimization_cost_final_array: ", Op.Optimization_cost_f
            # inal_array)
        element_1 = Op.Optimization_cost_final_array[index_of_maximum[0][0]]
        element_2 = Op.Optimization_parameters_array[index_of_maximum[0][0]]
        # print("element_1: ", element_1)
        # print("element_2: ", element_2)

        #Eliminate all the elements in array equal to index_of_maximum
        Op.Optimization_cost_final_array = remove_values_from_list(Op.Optimization_cost_final_array, element_1)
        Op.Optimization_parameters_array = remove_values_from_list(Op.Optimization_parameters_array, element_2)
        # Op.Optimization_cost_final_array.remove(Op.Optimization_cost_final_array[index_of_maximum[0][0]])
        # Op.Optimization_parameters_array.remove(Op.Optimization_parameters_array[index_of_maximum[0][0]])
        # print("Op.Optimization_cost_final_array: ", Op.Optimization_cost_final_array)
        # print(" Op.Optimization_parameters_array: ",  Op.Optimization_parameters_array)
        index_of_maximum = []
        message = 'Failed'
        if (len(Op.Optimization_cost_final_array ) < n_better_solutions - ii ):
            break
        # Op.Optimization_cost_final_array.pop(index)
        # Op.Optimization_parameters_array.pop(index)

    return final_solution_parameters, final_cost_solutions, message


 
    
def  main_loop():
    rospy.init_node('Camera_Param_Opt', anonymous=True)   
    #Create Image Publisher
    elaborated_img_after_opt = rospy.Publisher('img_elaborated_after_opt', Image, queue_size=10)   
   
    drone_obj = takedronedata()
    
    RGB_image = take_drone_camera_RGB_frame()
    THERMO_image = take_drone_camera_frame()

    #Initialize required classes for THERMO OPT
    sd_thermo = ShapeDetector()
    Op_thermo= Optimizer(THERMO_image.shape)
    THERMO = Image_THERMO_elaboration()

    #Initialize required classes for RGB opt
    sd = ShapeDetector()
    RGB = Image_RGB_elaboration()
    Op = Optimizer(RGB_image.shape)
    
    #True if the Correlation is available from the opt image 
    Op_thermo.correlation = True
    
    #Initialize Hyper-Parameters for the Obj Function evaluation 
    Op_thermo.Thermo_gaussian_mean_cost_value = 0.02
    Op_thermo.sd_thermo = 0.2

    Op.RGB_gaussian_mean_cost_value = 0.7
    Op.RGB_std = 0.3

    #Variables to Optimize RGB
    var_RGB = ['h', 's', 'v']
    
     #Variables to Optimize THERMO
    var_THERMO = ['th1_l', 'th1_h', 'distance']
    
    #Initial grid RGB 
    RGB_init_grid = 25

    #Initial grid Thermo
    Thermo_init_grid = 25
    Thermo_init_distance_grid = 3

    #Number of better solutio to found 
    n_better_solutions = 3
    

    H_MIN_default = rospy.get_param("/RGB_params/H_MIN")
    S_MIN_default = rospy.get_param("/RGB_params/S_MIN")
    V_MIN_default = rospy.get_param("/RGB_params/V_MIN")


    condition = 100000
    count = 0
    start_optimization_task = False
    RGB_opt_flag = False
    Thermo_opt_flag = False
    flag = False
    while(count < condition):
        #Take_RGB_camera_frame 
        RGB_image = take_drone_camera_RGB_frame()
        THERMO_image = take_drone_camera_frame()

        #Wait signal from flight controller to start Optimization task
        start_optimization_task = wait_for_opt_task()

        if start_optimization_task == True and (RGB_opt_flag == False or Thermo_opt_flag == False):
            #Do RGB Optimization 
            RGB_args = (RGB, sd, Op, drone_obj, var_RGB, RGB_init_grid, n_better_solutions, RGB_image, RGB_opt_flag)
            #RGB_clustered_image, res, RGB_opt_flag = RGB_optimization(RGB_args)
            
            #Do Thermal Optimization
            path_template = '/home/lucamora/Desktop/parameters_optimization/template.jpg'  #ELiminare a regime
            Op.corr_mask = Op_thermo.load_template(path_template)
            Thermo_args = (THERMO, sd, Op_thermo, drone_obj, var_THERMO, Thermo_init_grid, Thermo_init_distance_grid, Op.corr_mask, n_better_solutions, THERMO_image, Thermo_opt_flag)
            Thermo_clustered_image, res_thermo, Thermo_opt_flag = Thermal_optimization(Thermo_args)

            #Publish Optimized Parameters 
            publish_HSV_parameters(Op.h_low, Op.s_low, Op.v_low)
            print("PUBLISH HSV PARAMETERS --> H: {}, S:{}, V:{}".format(Op.h_low, Op.s_low,  Op.v_low))

            args = ( THERMO.th_0 , Op_thermo.th_1_low, Op_thermo.th_1_high, Op_thermo.th_2, THERMO.th_3,  THERMO.th_4)
            publish_Thermo_parameters(args)
            print("PUBLISH THERMAL PARAMETERS --> th1_l: {}, th1_h:{}, distance:{}".format(Op_thermo.th_1_low, Op_thermo.th_1_high,  Op_thermo.th_2))

        elif (start_optimization_task == True and RGB_opt_flag == True and Thermo_opt_flag == True):
            publish_HSV_parameters(Op.h_low, Op.s_low, Op.v_low)
            publish_optimization_flag(flag = True)
            print("PUBLISH HSV PARAMETERS --> H: {}, S:{}, V:{}".format(Op.h_low, Op.s_low,  Op.v_low))
        else:
            print("Waiting To reach the desired position to start the Parameter Optimization")
            publish_HSV_parameters(H_MIN_default, S_MIN_default, V_MIN_default)



        

if __name__ == '__main__':
   main_loop()
   #RGB_clustered_image = RGB_optimization()
   #THERMO_clustered_image = Thermal_optimization()

