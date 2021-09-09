#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 13 15:09:43 2020

@author: lucamora
"""


###In questo script l'immagine originale è scalata. su one drive c'è quello originale 

#import matplotlib
import operator
import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

#import matplotlib.pyplot as plt
#import matplotlib.image as mpimg
import rospy # if does not find the package inn shell : --> unset PYTHONPATH

import numpy as np

from scipy import ndimage
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') #Necessario prima di caricare cv2 in python3, commentare se uso python 2
import cv2
import time
import os
import math
import networkx 
from networkx.algorithms.components.connected import connected_components

#import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from sklearn import cluster


from ros_simulation_data import take_drone_camera_frame, takeEnvObservations
from ros_simulation_data import publish_navigation_Thermo_point
from ros_simulation_data import publish_navigation_Thermo_point2
from ros_simulation_data import publish_output_image


#Txt Files for debugging
# file1 = open("mean_line_distance.txt","w") 
# file21 = open("[PYTHON]_selected_control_point11.txt","w") 
# file22 = open("[PYTHON]_selected_control_point12.txt","w") 
# file31 = open("[PYTHON]_selected_control_point21.txt","w") 
# file32 = open("[PYTHON]_selected_control_point22.txt","w") 

# file3 = open("[PYTHON]Point1_X_control_body_frame.txt","w") 
# file4= open("[PYTHON]Poin1_Y_control_body_frame.txt","w") 
# file5 = open("[PYTHON]Point2_X_control_body_frame.txt","w") 
# file6= open("[PYTHON]Poin2_Y_control_body_frame.txt","w") 


    #Txt Files for debugging
#file1 = open("sim_data_complete/frame_received_and_analyzed_thermo.txt","w") 
#file2 = open("sim_data_complete/time_to_analyze_frame_thermo.txt","w") 

###############################################################################
#Parametri grandezza originale th2 = 9, th3 = 2000 th4 = 0.05
#Parametri grandezza scalata 70% th2 = 7, th3 = 1400 th4 = 0.05

#Parametri variano anche in base alll'altitudine --> Se due metri occorre ingrandirw area
#th2 = 9, th3 = 2000 th4 = 0.05

#Paramtri per altitudine 5m (molto simile realtà) immagine non scalata --> th2 = 11, th3 = 5500 
#Paramtri per altitudine 5m (molto simile realtà) immagine scalata a 70 --> th2 = 9, th3 = 5000 


##########################################################################################

th_0 = 125.547 #Reshape intensity pixels values 
th1 = [120, 180] #mask
# th1 = [20, 100]  #video
#th1 = [20, 90]
th2 = 9 #14 #distance 
th3 =  5000 #5500 #4500 # 2000 area
th4 = 0.05  #epsilon aproxPolydp 

index_th1 = 0
index_th2 = 0
index_th3 = 0
index_th4 = 0

original_image_shape = [0.0,0.0]

class drone(object):
   # def __init__(self, pose):
     def __init__(self):
        #Store position data
        # self.x = pose.pose.pose.position.x
        # self.y =  pose.pose.pose.position.y
        # self.z =  pose.pose.pose.position.z
        self.x =  0
        self.y =  0
        self.z = 3

# x = test(21, None)
# assert x.a == 21

#############  TAke drone status information ###################
def takedronedata():
    # poseData, imuData, velData, altitudeVelDrone = takeEnvObservations()
    #poseData = takeEnvObservations()
    #drone_obj = drone(poseData)
    drone_obj = drone()
    return drone_obj

def magnitude(point1, point2):
    vectorx = point1[0] - point2[0]
    vectory = point1[1] - point2[1]
    
    return math.sqrt(vectorx*vectorx + vectory*vectory)

def evaluate_mean_on_regression_line_distance(distance_point_line):
    N = len(distance_point_line)
    summ = 0
    for kk in range(0, N):
        summ += distance_point_line[kk][0]
    
    mean = summ/N
    return mean 
    

def rotate_image(image, angle):
    """
    Rotates an OpenCV 2 / NumPy image about it's centre by the given angle
    (in degrees). The returned image will be large enough to hold the entire
    new image, with a black background
    """

    # Get the image size
    # No that's not an error - NumPy stores image matricies backwards
    image_size = (image.shape[1], image.shape[0])
    image_center = tuple(np.array(image_size) / 2)

    # Convert the OpenCV 3x2 rotation matrix to 3x3
    rot_mat = np.vstack(
        [cv2.getRotationMatrix2D(image_center, angle, 1.0), [0, 0, 1]]
    )

    rot_mat_notranslate = np.matrix(rot_mat[0:2, 0:2])

    # Shorthand for below calcs
    image_w2 = image_size[0] * 0.5
    image_h2 = image_size[1] * 0.5

    # Obtain the rotated coordinates of the image corners
    rotated_coords = [
        (np.array([-image_w2,  image_h2]) * rot_mat_notranslate).A[0],
        (np.array([ image_w2,  image_h2]) * rot_mat_notranslate).A[0],
        (np.array([-image_w2, -image_h2]) * rot_mat_notranslate).A[0],
        (np.array([ image_w2, -image_h2]) * rot_mat_notranslate).A[0]
    ]

    # Find the size of the new image
    x_coords = [pt[0] for pt in rotated_coords]
    x_pos = [x for x in x_coords if x > 0]
    x_neg = [x for x in x_coords if x < 0]

    y_coords = [pt[1] for pt in rotated_coords]
    y_pos = [y for y in y_coords if y > 0]
    y_neg = [y for y in y_coords if y < 0]

    right_bound = max(x_pos)
    left_bound = min(x_neg)
    top_bound = max(y_pos)
    bot_bound = min(y_neg)

    new_w = int(abs(right_bound - left_bound))
    new_h = int(abs(top_bound - bot_bound))

    # We require a translation matrix to keep the image centred
    trans_mat = np.matrix([
        [1, 0, int(new_w * 0.5 - image_w2)],
        [0, 1, int(new_h * 0.5 - image_h2)],
        [0, 0, 1]
    ])

    # Compute the tranform for the combined rotation and translation
    affine_mat = (np.matrix(trans_mat) * np.matrix(rot_mat))[0:2, :]

    # Apply the transform
    result = cv2.warpAffine(
        image,
        affine_mat,
        (new_w, new_h),
        flags=cv2.INTER_LINEAR
    )

    return result


def largest_rotated_rect(w, h, angle):
    """
    Given a rectangle of size wxh that has been rotated by 'angle' (in
    radians), computes the width and height of the largest possible
    axis-aligned rectangle within the rotated rectangle.

    Original JS code by 'Andri' and Magnus Hoff from Stack Overflow

    Converted to Python by Aaron Snoswell
    """

    quadrant = int(math.floor(angle / (math.pi / 2))) & 3
    sign_alpha = angle if ((quadrant & 1) == 0) else math.pi - angle
    alpha = (sign_alpha % math.pi + math.pi) % math.pi

    bb_w = w * math.cos(alpha) + h * math.sin(alpha)
    bb_h = w * math.sin(alpha) + h * math.cos(alpha)

    gamma = math.atan2(bb_w, bb_w) if (w < h) else math.atan2(bb_w, bb_w)

    delta = math.pi - alpha - gamma

    length = h if (w < h) else w

    d = length * math.cos(alpha)
    a = d * math.sin(alpha) / math.sin(delta)

    y = a * math.cos(gamma)
    x = y * math.tan(gamma)

    return (
        bb_w - 2 * x,
        bb_h - 2 * y
    )


def crop_around_center(image, width, height):
    """
    Given a NumPy / OpenCV 2 image, crops it to the given width and height,
    around it's centre point
    """

    image_size = (image.shape[1], image.shape[0])
    image_center = (int(image_size[0] * 0.5), int(image_size[1] * 0.5))

    if(width > image_size[0]):
        width = image_size[0]

    if(height > image_size[1]):
        height = image_size[1]

    x1 = int(image_center[0] - width * 0.5)
    x2 = int(image_center[0] + width * 0.5)
    y1 = int(image_center[1] - height * 0.5)
    y2 = int(image_center[1] + height * 0.5)

    return image[y1:y2, x1:x2]


def to_graph(l):
    G = networkx.Graph()
    for part in l:
        # each sublist is a bunch of nodes
        G.add_nodes_from(part)
        # it also imlies a number of edges:
        G.add_edges_from(to_edges(part))
    return G

def to_edges(l):
    """ 
        treat `l` as a Graph and returns it's edges 
        to_edges(['a','b','c','d']) -> [(a,b), (b,c),(c,d)]
    """
    it = iter(l)
    last = next(it)

    for current in it:
        yield last, current
        last = current    






######## Esporto punti linea espressi in bottom iage frame in drone body frame   
def line_point_exportation_in_drone_body_frame(u1,v1,u2,v2, v0, u0, drone_obj, first):
   #x1, y1: coordinate punto P per il quale passa la retta trovata da fitline
   #x2, y2: coordinate punto P1 per il quale passa la retta dato dall'offset dei versori 
   #u0: coordinate del body frame rispetto l'image plane della bottom camera  
   #v0: coordinate del body frame rispetto l'image plane della bottom camera 
   
    #k/f : 0.00146 coefficiente di trasformazione pixel in metri calcolata sperimentalmente per òa bottom camera 
   const = 0.00146
   gain_y= 0.5
   
   
   #Point 1 
   #xb1 = -(const) * ((y1 * original_image_shape[1])/v0 -  original_image_shape[1])* drone_obj.z
   xb1 = -(const) * ((v1  -  v0 )* drone_obj.z)
   #yb1 =  -(const) * (x1 - u0)* drone_obj.z
 
   
   #yb1 =  -(const) * ((x1 * original_image_shape[0])/u0 - original_image_shape[0] )* drone_obj.z
   yb1 = -1* gain_y * (const) * ((u1  - u0 ) * drone_obj.z)
   #Point2 
   xb2 = -(const) * ((v2 - v0 )* drone_obj.z)
   yb2 = -1 * gain_y * (const) * ((u2 - u0  )* drone_obj.z)
   
   # file3.write(str(xb1) + '\n') 
   # # file3.write('%f\n'%xb1) 
   # file4.write(str(yb1) + '\n') 
   # # file5.write(str(xb2) + "\n") 
   # # file6.write(str(yb2) + "\n") 
   
   # file3.flush()
   # file4.flush()
   
   print('v0: ', v0)
   print('u0: ', u0)
   print('xb1: {0} v1: {1}'.format(xb1, v1))
   print('yb1: {0} u1: {1}'.format(yb1, u1))
   print('xb2: {0} v2: {1}'.format(xb2, v2))
   print('yb2: {0} u2: {1}'.format(yb2, u2))
    
   print('######### PANEL LINE RECOGNIZED, FOLLOWING')
   
   if (first == True):
         # Publish over the air The message with control points 
         publish_navigation_Thermo_point(xb1, yb1, xb2, yb2)
 
   else:
         publish_navigation_Thermo_point2(xb1, yb1, xb2, yb2)
         
   
def clustering_lines(start_point, end_point, points, row, col, angular_coefficient, angular_line_versors):
   
    point_line_distance_actual = []
   
    same_cluster = []
    same_cluster_similar_direction = []
    sum_list = []
    new_list = []
    count1 = 0
    
    # t3_start = process_time()  
    
    for ii in range(0, len(start_point)):
       
        vx = angular_line_versors[ii][0][0]
        vy = angular_line_versors[ii][0][1]
        # print('x: ',start_point[ii][0][0])
        # print('y: ',start_point[ii][0][1])
        # line_start = [start_point[ii][0][0], start_point[ii][0][1]]
        # line_end = [end_point[ii][0][0], end_point[ii][0][1]]
        # #Considero coordinate lungo asse x di linea di start e di end --> colonne matrice
        # line_ii_start_x = line_start[0]
        # line_ii_end_x = line_end[0]
        
        same_cluster1 = []
        same_cluster1.append(ii)
      
        # print('m: ',m)
        # print('b: ',b)
        # print('c: ',c)
        
        
        #I calcoli qui vengono effettuati in pixel coordinates
        m = -end_point[ii][0][1] +start_point[ii][0][1]   #-y2 + y1 ---> (y_end + y_start)  in pixel corrdinates 
        b = (end_point[ii][0][0] - start_point[ii][0][0])
        c = -1*(-end_point[ii][0][1]* start_point[ii][0][0] + end_point[ii][0][0] * start_point[ii][0][1])
        # m = 1/(-end_point[ii][0][0] +start_point[ii][0][0])
        # b = (end_point[ii][0][1] - start_point[ii][0][1])
        # c = -end_point[ii][0][0]* start_point[ii][0][1] + end_point[ii][0][1] * start_point[ii][0][0]
        # scale_factor = end_point[ii][0][0] - start_point[ii][0][0]
        # print('m: ',m)
        # print('b: ',b)
        # print('c: ',c)
        # print('scale_factor: ',scale_factor)
        
      
        #Considero la retta ii e calcolo la distanzadella retta ii da tutti i punti che compongono le restanti rette jj
        for jj in range(0,len(start_point)):
            if start_point[jj][1][0] == ii:  #considero il valore di count
               continue
            else:
               # print('ii: {0} jj: {1}'.format(ii, jj))
               vx_jj = angular_line_versors[jj][0][0]
               vy_jj = angular_line_versors[jj][0][1]
               #siccome versori, gia allineati agli assi posso sommare le componenti dei descrittori delle due rette per ottenere la magnitudine della risultante V
               if abs(vx) - abs(vx_jj) < 0.02 and abs(vy) - abs(vy_jj) < 0.02:
                    Vx = abs(vx) - abs(vx_jj)
                    Vy =abs(vy) - abs(vy_jj)
               else:
                    Vx = vx - vx_jj 
                    Vy = vy - vy_jj
               V = math.sqrt(Vx*Vx + Vy*Vy)
                
               
               
               # line_jj_start_x = start_point[jj][0][0]
               # line_jj_end_x = end_point[jj][0][0]
               
               # print('ii: {0} jj: {1}'.format(ii, jj))
               # print('row_ii_start: {0}  row_jj_start: {1}  row_ii_end: {2} row_jj_end: {3}'.format(line_ii_start_x,line_jj_start_x, line_ii_end_x, line_jj_end_x))
               # print('col_ii_start: {0}  col_jj_start: {1}  col_ii_end: {2} col_jj_end {3}'.format(start_point[ii][0][1],start_point[jj][0][1], end_point[ii][0][1], end_point[jj][0][1]))

              
               # # print('sono qui')
               # # print('ii: {0} jj: {1}'.format(ii, jj))
              
               # print('vx: {0}  vy: {1}  vx_jj: {2} vy_jj: {3}'.format(vx, vy, vx_jj, vy_jj))
               # print('V: ', V)
              
               
               #per verificare l'intersezione dovrei fare un check sui valori di start ed end delle line sull'asse x, quindi sulle colonne
               #Se due linee hanno i valori di start ed end minori o maggiori allora si intersecano
               for mm in range(0, len(points)):
                   if (points[mm][1][0] == jj):
                      
                       x = points[mm][0][0]
                       y = points[mm][0][1]
                       # print('x: ', x)
                       # print('y: ', y)
                       # if (jj == 8 and ii == 1):
                       #     time.sleep(1000)
               #Distanza punto retta 
                       # point_line_distance.append([[np.abs(m * x + b*y + c)/math.sqrt(m*m + b*b)],[points[jj][1][0]]])
                       point_line_distance_actual.append(np.abs(m * x + b*y + c)/math.sqrt(m*m + b*b))
              
               #Select lines based on their minimum distance and angular values  
               #LA distanza minima deve essere prossima allo zero e il valore di V maggiore di 1, in quanto corrisponde alla risultante della somma dei vettori vx1 con vx2 vy1 vy2
               if (V < 0.2):
                    #Check for cluster which have same direction ofthe regression line
                   same_cluster_similar_direction.append([ii])
                   same_cluster_similar_direction.append([jj])
                   if (min(point_line_distance_actual) < 20):   #(intersection_flag == True): 
                        same_cluster.append([ii, jj])
                        # print('-------> same_cluster: ', same_cluster)
                        same_cluster1.append(jj)
            
               point_line_distance_actual = []
               count1 = count1 + 1
               
               
        if ( (sum(same_cluster1) in sum_list) == True or len(same_cluster1) == 1):
            continue
      
        sum_list.append(sum(same_cluster1))
        # print('sum_list: ',sum_list)
        new_list.append(same_cluster1)
    #     print('-------> new_list: ', new_list)
    # print('-------> same_cluster: ', same_cluster)
    
    #############Commentare nel caso si volessse riconoscere meno pannelli ma con una precisione maggiore
    if len(same_cluster) == 0:
        #Check for cluster which have same direction ofthe regression line
        #Insert in same cluater the cluster with similar direction without intersection
        #---> Utilizzo principale nella fine della vela
        new_list = same_cluster_similar_direction
        # print('same_cluster: ', same_cluster)
    
   
    if len(same_cluster) > 0:
        #Verifico se riconosco pannelli che non vengono clutserizzati
        #ma che la retta di regressione possiede stessa inclinazione ma nessuna intersezione
        for ii in range(0, len(same_cluster_similar_direction)):
            for jj in range(0, len(same_cluster)):
                for kk in range(0, len(same_cluster[jj])):
                    if same_cluster[jj][kk] == same_cluster_similar_direction[ii]:
                        continue
                    else:
                        new_list.append(same_cluster_similar_direction[ii])
    ####################                
                    
    G = to_graph(new_list)
    s = connected_components(G)
    new_list = []
    for ss in s:
        new_list.append(list(ss))
        # print (new_list)
   
    
    
    same_cluster = []
    same_cluster = new_list
     
    theta = []
    theta_mean = []
    #### Find angle theta for each selected cluster 
    for ii in range(0, len(same_cluster)):
        for jj in range(0, len(same_cluster[ii])):
            
            vx = angular_line_versors[same_cluster[ii][jj]][0][0]
            vy = angular_line_versors[same_cluster[ii][jj]][0][1]
            a = math.atan(abs(vy)/abs(vx))
            theta.append(a * 180/np.pi)
        theta_mean.append(np.mean(theta))
        theta = []
          
        
    # #Eliminate outliers from same cluster in order to avoid 
        
    # print('theta_mean: ',theta_mean )
    return same_cluster, theta_mean


def image_pre_processing(image, line_versors_old,drone_obj,  counter_frame_analyzed, counter_frame_received):
    flag_final_frame_analyzed = False
    ########## Rotation and resize of image ###########
    
     #Counting frame received for statistical analysis
    if counter_frame_analyzed > 1:
         counter_frame_received = counter_frame_received + 1
         
         
    # image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    inf = th1[0]
    sup = th1[1]
    th2_value = th2 
    th3_value = th3
    th4_value = th4
    
     #GRAYSCALE
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # imgplot = plt.imshow(gray,cmap='gray', vmin = 0, vmax = 255)
    # plt.show() 
    #Evaluate mean of all image pixel intensities
    intensity_mean = 0
    intensity_mean = np.mean(gray)
   
    original_image_shape[0] = gray.shape[0]
    original_image_shape[1] = gray.shape[1]
    
    # print('intensity_mean: ', intensity_mean)
   
    # plt.hist(gray.ravel(),256,[0,256]); plt.show()
    const_int_value = [ [ int(th_0 - intensity_mean) for y in range( gray.shape[1] ) ] for x in range( gray.shape[0] ) ]
    
    # im1arr = np.asarray(gray)
    const_int_value = np.array(const_int_value)
    gray = const_int_value + gray
    gray=gray.astype(np.float32)
   
    intensity_mean = np.mean(gray)
    # print('intensity_mean: ', intensity_mean)
    # plt.hist(gray.ravel(),256,[0,256]); plt.show()
    # imgplot = plt.imshow(gray,cmap='gray', vmin = 0, vmax = 255)
    # plt.show() 
   
   

   
    percent =70 #70
    #Resize Image 
    width = int(image.shape[1] * percent/ 100)
    height = int(image.shape[0] * percent/ 100)
    dim = (width, height)
    
    
    
    
    
    frame = cv2.resize(gray, dim, interpolation =cv2.INTER_AREA)
    image = cv2.resize(image, dim, interpolation =cv2.INTER_AREA)
    # imgplot = plt.imshow(image1,cmap='gray', vmin = 1, vmax = 255)
    # plt.show()
   
    # imgplot = plt.imshow(image,cmap='gray', vmin = 0, vmax = 255)
    # plt.show() 
    
   
    # plt.hist(gray.ravel(),256,[0,256]); plt.show()
    # time.sleep(1000)
    # imgplot = plt.imshow(gray,cmap='gray', vmin = 1, vmax = 255)
    # plt.show()
    
    
    # print('width:  {0}, height: {1}'.format(width, height))
    blur1 = cv2.bilateralFilter(frame,3,75,75)
    ret,thresh_blur1 = cv2.threshold(blur1,inf,255,cv2.THRESH_TOZERO)
    ret,thresh_blur2 = cv2.threshold(thresh_blur1,sup,255,cv2.THRESH_TOZERO_INV)
    # imgplot = plt.imshow(thresh_blur2,cmap='gray', vmin = 1, vmax = 255)
    # plt.show()
    distance_matrix = ndimage.distance_transform_edt(thresh_blur2) 
   
    # imgplot = plt.imshow(distance_matrix,cmap='gray', vmin = 1, vmax = 255)
    # plt.show()
   
    ret,thresh= cv2.threshold(distance_matrix,th2_value,255,cv2.THRESH_BINARY)
    # imgplot = plt.imshow(thresh,cmap='gray', vmin = 1, vmax = 255)
    # plt.show()
    thresh = cv2.convertScaleAbs(thresh)
    _ ,contours, _= cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
   
    lefty_array = []
    rigthy_array = []
    start_point = []
    end_point = []
    points = []
    angular_coefficient = []
    actual_points = []
    angular_line_versors = []
    countours_considered = []
    pixels_iniside_area = []
    region_pixels_size_coo = [] # coordinate of min col min row pixel, of max col min row pixel and max col, min row pixel
    
    counter_cnt = 0
    count = 0
    
    
    ############## INDIVIDUZIONE DEI DIVERSI CLUSTER E DELLE RISPETTIVE RETTE DI REGRESSIONE ############
    for cnt in contours: 
        if cv2.contourArea(cnt) < 500000 and cv2.contourArea(cnt) > th3_value/2:
           # print('th2_value:  {0}, th3_value: {1}'.format(th2_value, th3_value))
           epsilon = th4_value*cv2.arcLength(cnt,True)   #0.1 is 10% of arc length
           out_table = cv2.approxPolyDP(cnt, epsilon, True)
         
           # cv2.drawContours(image, [out_table], -1, (0, 255, 0), 3)
           #ritorna i vettori normalizzati lungo x e y e un punto xo yo sulla retta
           [vx,vy,x,y] = cv2.fitLine(out_table,cv2.DIST_L2,0,0.01,0.01)
          
       #intersezione della retta con asse y della figura nel punto x = 0 
           lefty= int((-x*vy/vx)+y)
           # print('lefty:',lefty)
       #intersezione della retta con asse y della figura nel punto x = thresh.shape[1]
           righty = int(((thresh.shape[1]-x)*vy/vx)+y)
       #evaluate line angular coefficient:
           if righty - lefty == 0:
               righty= righty + 1
           #angular_coefficient.append([[(thresh.shape[1]-1)/(righty - lefty)],[count]])
               
           # cv2.line(image,(thresh.shape[1]-1,righty),(0,lefty),255,2) 
           # cv2.line(image,(righty,thresh.shape[1]-1),(lefty,0),255,2) 
           # imgplot = plt.imshow(image,cmap='gray', vmin = 0, vmax = 255)
           # plt.show()   
           #Save 
           # vx_array.append(abs(vx))
           # vy_array.append(abs(vy))
           # stop1 = False
           # stop = False
           
           #######################
           #Evaluate all points inside this line: --> [righe, col]
           #start_point [col asse x  (da 0  a 630 lungo i bordi orizzontali), riga asse x (da 0 a 510 lungo il bordo laterale, zero si trova in alto), [numero della retta]
           for ii in range(0, thresh.shape[1]):
               if (((ii-x)*vy/vx)+y >= 0 and ((ii-x)*vy/vx)+y <= thresh.shape[0]):
                       points.append([[ii, int(((ii-x)*vy/vx)+y)],[count]])
                       actual_points.append([ii, int(((ii-x)*vy/vx)+y)])
                       # print('points: ',  [ii, int(((ii-x)*vy/vx)+y)], [count])
                       # if count == 8:
                       #     time.sleep(1000)
          
           #Puo succedere che se la retta è molto verticale non vemgono trovati valori interi di y (righe) per ii che varia lungo asse x (colonne)
           #Se succede cerco valori di x al variare di y:
           if (len(actual_points) == 0):
               for ii in range(0, thresh.shape[0]):
                   if ((y - ii)*(1/(vy/vx))+x >= 0 and ((y - ii)*(1/(vy/vx))+x  <= thresh.shape[1])): #Cerco un valore di x quindi scorro su bordo orizzontale
                          points.append([[int((y - ii)*(1/(vy/vx))+x), ii],[count]])
                          actual_points.append([int((y - ii)*(1/(vy/vx))+x), ii])
               
                          
           # print('actual_points[0]: ', actual_points[0])
           
           start_point.append([actual_points[0],[count]]) #Di tutti i punti che appartengono alla retta considero il primo incontrato e l'ultimo sia per x che y
               
           end_point.append([actual_points[-1],[count]])
           # print('start_point: ', [actual_points[0],[count]])
           # print('[end_point: ', [actual_points[-1],[count],[count]])                                     
           angular_line_versors.append([[vx, vy], [count]])                
            
          ############################# 
          #Per conoscere i pixel dentro il contorno creo una maschera con valore 1 per i pixel dentro il cotorno
           lst_points = []

          # Create a mask image that contains the contour filled in
           img = np.zeros_like(image)
        
           cv2.drawContours(img, [out_table],-1, color=255, thickness=-1)

           # Access the image pixels and create a 1D numpy array then add to list
           pts = np.where(img == 255) ### ---> ritorna una lista di tre array dove il primo sono le righe, il secondo le colonne dei pixel considerati
          
           lst_points.append([[pts[0]],[pts[1]], [count]])  ## righe, colonne dei pixel dentro la maschera e contatore della regione (quindi della retta ) considerata
           
           #NB: I PIXEL QUA SONO ORDINATI COME RIGHE E COLONNE, QUINDI Y X 
           #Search pixel to describe region
           
           # min_row = min(pts[0]) #asse y
           # col_related_min_row = pts[1][np.where(pts[0] == min(pts[0]))[0][0]]  #asse x
           
           # max_col = max(pts[1])
           # row_related_max_col = pts[0][np.where(pts[1] == max(pts[1]))[0][0]]
           
           # max_row = max(pts[0])
           # col_related_max_row = pts[1][np.where(pts[0] == max(pts[0]))[0][0]]
           # #Queste coordinate mi serviranno per capire quale cluster si trova sopra l'altro e per unire la max row di quello sopra con la min row di quello sotto in modo da avere un box finale
           
           # # print('max_row ', max_row)
           # # print('col_related_max_row ', col_related_max_row)
           # region_pixels_size_coo.append([[min_row, col_related_min_row], [ row_related_max_col, max_col],  [max_row, col_related_max_row], [count]])
           # # print('region_pixels_size_coo ', region_pixels_size_coo)
           #[minore riga, colonna corrispondente], [riga corrispondente, max colonna ][max riga, colonna corrispondente]
                                           
           pixels_iniside_area.append([lst_points])
           # print('pixels_iniside_area: ', pixels_iniside_area)
           ############################
           actual_points = []

           # imgplot = plt.imshow(img,cmap='gray', vmin = 0, vmax = 255)
          
           # plt.show()  
           
           countours_considered.append([out_table])
           count = count + 1
    # imgplot = plt.imshow(image,cmap='gray', vmin = 0, vmax = 255)
    # plt.show()    
   
   
    #CLUSTERING LINES 
    cluster , theta_mean= clustering_lines(start_point, end_point, points, thresh.shape[0], thresh.shape[1], angular_coefficient, angular_line_versors)   
   
    
    new_region = []
    final_countours = []
    clusters = []
    same_mask = []
    flag_difference = False
    line_versors = []
    line_versors_final = []
    
    ### Creo differenti maschere ciascuna con i pannelli clusterizzati 
    for ii in range(0, len(cluster)):
       mask = np.zeros_like(image) 
       theta_mean_cluster_area = theta_mean[ii]
       # print(theta_mean_cluster_area)
       for area in range(0,len(cluster[ii])):

           for pixels in range(0, len(pixels_iniside_area[cluster[ii][area]][0][0][0][0])):
                
                 list_considered_row = pixels_iniside_area[cluster[ii][area]][0][0][0][0][pixels] #Prendo le righe relative la regione 1
                 list_considered_col= pixels_iniside_area[cluster[ii][area]][0][0][1][0][pixels] #Prendo le colonne relative la regione 1
                  
                 mask[list_considered_row][list_considered_col] = 250
    
    
       # imgplot = plt.imshow(mask,cmap='gray', vmin = 0, vmax = 255)
       # plt.show()          
       
       mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
       mask = cv2.convertScaleAbs(mask)
       _ ,contours, _= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
       
       contours1 =[]
       
       #Definisco i Contorni per ciascuna maschera 
       for hh in range(0, len(contours)):
           for hh1 in range(0, len(contours[hh])):
               
                contours1.append(contours[hh][hh1])
      
    ##### FInd equation of the survived lines 
       [vx,vy,x,y] = cv2.fitLine(np.float32(contours1),cv2.DIST_L2,0,0.01,0.01)
       
       #Appendo versori e punti 
       #----> x colonne 
       #----> y righe 
       line_versors.append([vx, vy, x, y]) 
       #print('x:  {0}, y: {1}'.format(line_versors[ii][2][0], line_versors[ii][3][0]))
      
       for cnt in contours: 
            if cv2.contourArea(cnt) < 500000 and cv2.contourArea(cnt) > th3_value:
             # print('th2_value:  {0}, th3_value: {1}'.format(th2_value, th3_value))
                 th4_value = 0.03
                 epsilon = th4_value*cv2.arcLength(cnt,True)   #0.1 is 10% of arc length
                 out_table = cv2.approxPolyDP(cnt, epsilon, True)
                
                 cv2.drawContours(image, [out_table], -1, (0, 255, 0), 3)
                 
                 lefty= int((-x*vy/vx)+y)
                  # print('lefty:',lefty)
                 #intersezione della retta con asse y della figura nel punto x = thresh.shape[1]
                 righty = int(((thresh.shape[1]-x)*vy/vx)+y)
                 #evaluate line angular coefficient:
                 
              # print('m_final: ', m_final)
        # cv2.line(image, (y_down, image.shape[0]), (b0, 0), (255,0,0), 2)
            # if theta_mean_cluster_area > 60:
            #       cv2.line(image, (y_down, image.shape[0]), (b0, 0), (255,0,0), 2)  #start_point (u1,v2) end_point(u2,v2)
            # else:
            #       cv2.line(image, (image.shape[0], y_down), (0, b0), (255,0,0), 2)    
     
       
              
       # cv2.line(image,(thresh.shape[1]-1,righty),(0,lefty),255,2)   
    
    
    #####     VOTING TECNIQUE RESPECT t-1 frame #####################
    voting_array = [None] * len(line_versors)
   
    #Verifico magnitudine tra i versori del frame precedente e di quello attuale
    final_line_versors_frame = []
    if (len(line_versors_old) > 0 and len(line_versors) > 0):
        for ii in range(0, len(line_versors_old)):
            for jj in range(0, len(line_versors)):
                
                #Take versors of line in both frames
                vx_ii = line_versors_old[ii][0][0]
                vy_ii = line_versors_old[ii][1][0]
                vx_jj =  line_versors[jj][0][0]
                vy_jj =  line_versors[jj][1][0]
               
                #Calcolo magnitudine dei due versori
                if abs(vx_ii) - abs(vx_jj) < 0.03 and abs(vy_ii) - abs(vy_jj) < 0.03:
                      Vx = abs(vx_ii) - abs(vx_jj)
                      Vy = abs(vy_ii) - abs(vy_jj)
                else:
                      Vx = vx_ii - vx_jj 
                      Vy = vy_ii - vy_jj
                V = math.sqrt(Vx*Vx + Vy*Vy)  
                
                if V < 0.15:
                   final_line_versors_frame.append([vx_jj, vy_jj, line_versors[jj][2][0], line_versors[jj][3][0]])
                  #Mantain perpendicular lines
                # elif ( V < 1.40 and V > 1): #perpendicular lines
                #     if (voting_array[jj] == None):
                #         voting_array[jj] = 1
                #     else:
                #          voting_array[jj] = voting_array[jj] + 1
                         
                else:
                  #Probabili linee da scartare 
                     if (voting_array[jj] == None):
                        voting_array[jj] = 1
                     else:
                         voting_array[jj] = voting_array[jj] + 1
   
    
   
    
    
    #--------> Check max voting value 
    #Definisco percentuale limite oltre la quale la retta va scartata  
    #LA retta va scartata se è votata da piu dell'80% delle rette presenti
    limit_percentage = 0.3 * len(line_versors)  
   
    rospy.loginfo('IMAGE_PRE_PROCESSING: limit_percentage: %f ', limit_percentage)
    for ii in range(0, len(voting_array)):
        #Definsico le rette da mantenere
        if voting_array[ii] < limit_percentage: 
            final_line_versors_frame.append([line_versors[ii][0][0], line_versors[ii][1][0], line_versors[ii][2][0], line_versors[ii][3][0]])
            print('#################  voting_array[ii]: ', voting_array[ii])
    #Create array for line points 
    line_points = []
    cartesian_distance_point_from_image_center = []
    # points = []
    # actual_points = []
    mean_distance_of_line_from_center_line = []
    for ii in range(0, len(final_line_versors_frame)):
        #print('final_line_versors_frame[ii]: ', final_line_versors_frame[ii])
        point_line_distance_actual = []
        ####Definisco rette sopravvisutte alla filtrazione rispetto frame precedente 
        vx = final_line_versors_frame[ii][0]
        vy = final_line_versors_frame[ii][1]  
        x =  final_line_versors_frame[ii][2]   #colonne immagine
        y =  final_line_versors_frame[ii][3]   #righe immagine
        xv = x + vx
        yv = y + vy
        ########## Exportation del punto (x,y) della retta e del punto Pv = (x + vx, y + vy) nel body frame del drone (considerando Ar drone 2.0)
        # line_point_exportation_in_drone_body_frame(x,y,xv,yv, (thresh.shape[0]/2), (thresh.shape[1]/2), drone_obj)  #Definsico locazione del body frame del drone rispetto l'image plane (esattamente al centro )
        
        line_points.append([x,y,xv,yv]) #line points in pixel coordinates
        
        
        
        
        lefty= int((-x*vy/vx)+y)
                  # print('lefty:',lefty)
                 #intersezione della retta con asse y della figura nel punto x = thresh.shape[1] 
        righty = int(((thresh.shape[1]-x)*vy/vx)+y)
                 #evaluate line angular coefficient:
        
        
        for jj in range(0, thresh.shape[1]): #itero sulle colonne  
               if (((jj-x)*vy/vx)+y >= 0 and ((jj-x)*vy/vx)+y <= thresh.shape[0]):
                       # points.append([[jj, int(((jj-x)*vy/vx)+y)],[ii]])
                       # actual_points.append([jj, int(((jj-x)*vy/vx)+y)])
                       
                       xp = jj
                       yp = int(((jj-x)*vy/vx)+y)
                       #### EValuate global distance pf the line from the center line 
                       # xp = points[0][0]
                       # yp = points[0][1]
                       
                      
                       # dis =  math.sqrt(pow(xp-thresh.shape[1]/2,2 ) + pow(yp-jj,2))
                       dis = abs(xp-thresh.shape[1]/2)
                       point_line_distance_actual.append(dis) 
        
        if (math.isnan(np.mean(point_line_distance_actual))):
               point_line_distance_actual = []
               for jj in range(0, thresh.shape[0]):#itero sulle righe 
                   if ((y - jj)*(1/(vy/vx))+x >= 0 and ((y - jj)*(1/(vy/vx))+x  <= thresh.shape[1])): #Cerco un valore di x quindi scorro su bordo orizzontale
                       yp = jj
                       xp = int(((jj-x)*vy/vx)+y) 
                       # dis =  math.sqrt(pow(xp-thresh.shape[1]/2,2 ) + pow(yp-jj,2))
                       dis = abs(xp-thresh.shape[1]/2)
                       point_line_distance_actual.append(dis) 
        
       
        #### EValuate mean distance pf the line from the center line 
        mean_distance_of_line_from_center_line.append(np.mean(point_line_distance_actual))  
      
    #    #shape return heigth and with--> righe e colonne immagine 
        
    #       #### Per il controllo considero la retta che ha distanza su x e y piu vicina alla al centro della foto
    #     a = math.sqrt(pow(x-thresh.shape[1]/2,2 ) + pow(y-thresh.shape[0]/2,2))
    #     cartesian_distance_point_from_image_center.append(a)
        try:      
            cv2.line(image,(thresh.shape[1]-1,righty),(0,lefty),255,2)     #start_point (u1,v2) end_point(u2,v2)
                                                                          #nel mio caso sono invertiti --> righty: intersezione bordo inferiore
                                                                                                            # lefty: intersezione bordo superiore
        except:
            pass
    # # print('cartesian_distance_point_from_image_center: ', cartesian_distance_point_from_image_center)
        
    first = True
    #Find line with minimum distance from image center 
   
    if (len(mean_distance_of_line_from_center_line) > 0):
    # if (len(cartesian_distance_point_from_image_center) > 0):
        #Ordering vector from lowest to highest value
          
            min_line = min(mean_distance_of_line_from_center_line)
    
             #Take the index of the first line 
            try:
                index = mean_distance_of_line_from_center_line.index(min_line)
            except:
                index = 1 #Prende la rpima linea se non trova l'index desiderato
           
          
            line_considered = line_points[index]
            print("########################line_considered: ", line_considered)
             ############# DEBUGGING TXT FILES ##########
            print("######################## mean_distance_of_line_from_center_line: ", mean_distance_of_line_from_center_line[index])
            # file1.write(str(mean_distance_of_line_from_center_line[index]) + "\n") 
            # file1.flush()
           
            #    ############################## Riguardare statsera....
            # #Probabilmente sto considerando righe per colonne.. 
            # #necessario ricontrollare
            # file21.write(str(line_considered[0]) + "\n")  #colonne immagine per punto 1 ---> y
           
            # file22.write(str(line_considered[1]) + "\n") #righe immagini per punto 1 --> x
            # file31.write(str(line_considered[2]) + "\n") 
            # file32.write(str(line_considered[3]) + "\n") 
            
            # file21.flush()
            # file22.flush()
            # file31.flush()
            # file32.flush()
          
             ########## Exportation del punto (x,y) della retta e del punto Pv = (x + vx, y + vy) nel body frame del drone (considerando Ar drone 2.0)
            line_point_exportation_in_drone_body_frame(line_considered[0],line_considered[1],line_considered[2],line_considered[3], (thresh.shape[0]/2), (thresh.shape[1]/2), drone_obj, first)  #Definsico locazione del body frame del drone rispetto l'image plane (esattamente al centro )
             
            # line_points.pop(index)
            # # print("line_points:  ", line_points)
            # mean_distance_of_line_from_center_line.pop(index)
            
            # if (len(mean_distance_of_line_from_center_line) > 0):
            #      min_line = min(mean_distance_of_line_from_center_line)
            
            #      try:
            #          index = mean_distance_of_line_from_center_line.index(min_line)
            #      except:
            #          index = 1 #Prende la rpima linea se non trova l'index desiderato
              
            #      # print("######################## line_points: ", line_points)
            #      # print("######################## index: ", index)
            #      line_considered = line_points[index]
            #      first = True
            #      ###### Exportation of teh second nearest line in world frame.. viene inviato su un altro topic
            #      line_point_exportation_in_drone_body_frame(line_considered[0],line_considered[1],line_considered[2],line_considered[3], (thresh.shape[0]/2), (thresh.shape[1]/2), drone_obj, first)  #Definsico locazione del body frame del drone rispetto l'image plane (esattamente al centro )
             
            # #Take the index of the second nearest line 
            flag_final_frame_analyzed = True
           
      
    if flag_final_frame_analyzed == True:
         counter_frame_analyzed = counter_frame_analyzed + 1
         flag_final_frame_analyzed  = False
        
            
    return  image, line_versors,  counter_frame_analyzed, counter_frame_received
    
    
    
def load_image(photo_number, inf, sup ,th2_value, th3_value, th4_value, rotation_degree):
     name = '/home/lucamora/image_analysis/Volo_01/DJI_0' + str(photo_number) + '.jpg'
     image = cv2.imread(name) 
     
     cluster_image = image_pre_processing(image,photo_number,inf, sup, th2_value, th3_value, th4_value, rotation_degree)
     
     return cluster_image
     


def load_video(video_name):
     cap = cv2.VideoCapture(video_name)
     frame_count = 0
     while frame_count < 6000:
         ret, frame = cap.read()
         frame_count = frame_count + 1
         
         blur1, distance_matrix = image_pre_processing(frame,11,th1[0], th1[1], th2, th3, th4)
         if cv2.waitKey(1) & 0xFF == ord('q'):
             break
     cap.release()
     cv2.destroyAllWindows()  
     return frame





    


def listener():
    rospy.init_node('termo_frame_elaboration', anonymous=True)
    #Create Image Publisher
    Camera_elaborated_frame_pub = rospy.Publisher('camera_vision_output', Image, queue_size=10)
    
   
    condition = 10000
    count = 0
    line_versors_old = []
    counter_frame_analyzed = 0 
    counter_frame_received = 0
    while(count < condition):
        #Save all informations in drone struct 
        start1 = time.time()
        drone_obj = takedronedata()
        end1= time.time() - start1
        # print('end1: ', end1)
        
        #Take only RGB o Thermal Camera if Video Stream is double:
        #0: RGB
        #1: Thermal
        select = 0
        image = take_drone_camera_frame()
        #Subdivide RGB and Thermal Images 
        width = image.shape[1]
        width_cut_off = int(width/2)
        image_RGB = image[:, :width_cut_off]
        image_thermo = image[:, width_cut_off:]
       
       
      

        clustered_image, line_versors, counter_frame_analyzed_out, counter_frame_received_out = image_pre_processing(image_thermo, line_versors_old, drone_obj, counter_frame_analyzed, counter_frame_received)
        
        counter_frame_analyzed = counter_frame_analyzed_out
        counter_frame_received = counter_frame_received_out
                
        
        #Publish output frame in rostopic
        publish_output_image(clustered_image)
       
       # end2= time.time() - start1
  
       
        #rospy.loginfo('MAIN LOOP: Loop time: %f ', 1/end2)
        cv2.imshow('image', image_RGB)
        cv2.waitKey(1)
        
        
        #print('width: {0} heigth: {1}'.format(width, heigth)) #---> 1280*720

        #Take only RGB o Thermal Camera if Video Stream is double:
       
       
    # imgplot = plt.imshow(image,cmap='gray', vmin = 0, vmax = 255)
    # plt.show()     
    #cluster_image = image_pre_processing(image)
        #line_versors_old = line_versors
        count = count + 1
        # start2 = time.time()
        # rospy.Rate(0.2).sleep()  # 1 Hz
        # end3= time.time() - start2
        # print('#####################end3: ', end3)
      
   # cv2.destroyAllWindows()    
  

      
if __name__ == '__main__':
   
   
    listener()
    # file1.close()
    # file21.close() 
    # file22.close()
    # file31.close()
    # file32.close() 
    # file4.close() 
    # file5.close() 
    # file6.close() 
    
 
#save elapsed tIme in txt file
# make up some data
# print(elapsed_time)
# text_file = open("elapsed_time_random_rotation_vertcal.txt", "w")
# for ii in range(0, len(elapsed_time)):
#       t = elapsed_time[ii]
#       n = text_file.write("%f\n" %t)
# text_file.close()


