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

import networkx 
from networkx.algorithms.components.connected import connected_components

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from sklearn import cluster
#from regression_line import regression_line
from random import randrange 
from ros_simulation_data import take_drone_camera_RGB_frame, takeEnvObservations, receive_estimated_control_point_P1, receive_estimated_control_point_P2
from ros_simulation_data import publish_navigation_RGB_point
from ros_simulation_data import publish_output_image
from ros_simulation_data import publish_output_RGB_image
from ros_simulation_data import publish_img_hsv 
# from ros_simulation_data import publish_img_first_threshold
from ros_simulation_data import publish_img_clustering_mask

# th1 = [20, 100]  #video
#th1 = [20, 90]

th3 = 12500 #area
th4 = 0.01  #epsilon aproxPolydp 
th5 = 70200#70000  #Area oltre il quale il cluster è considerato unico
index_th1 = 0
index_th2 = 0
index_th3 = 0
index_th4 = 0

#Txt Files for debugging
#file1 = open("sim_data_complete/frame_received_and_analyzed_RGB.txt","w") 
#file2 = open("sim_data_complete/time_to_analyze_frame_RGB.txt","w") 



class drone(object):
    #def __init__(self, pose):
    def __init__(self):
        #Store position data
        # self.x =  pose.point.x
        # self.y =  pose.point.y
        # self.z =  pose.point.z
        self.x =  0
        self.y =  0
        self.z = 7


#############  TAke drone status information ###################
def takedronedata():
    #poseData = takeEnvObservations()
    #Considero al momento poszione drone costante in 0,0,0-- >  a tempo debuito bisognera sottoscriversi al local frame per avere la poszione in metri
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
def line_point_exportation_in_drone_body_frame(u1,v1,u2,v2, v0, u0, drone_obj):
   #x1, y1: coordinate punto P per il quale passa la retta trovata da fitline
   #x2, y2: coordinate punto P1 per il quale passa la retta dato dall'offset dei versori 
   #u0: coordinate del body frame rispetto l'image plane della bottom camera  
   #v0: coordinate del body frame rispetto l'image plane della bottom camera 
   
    #k/f : 0.00146 coefficiente di trasformazione pixel in metri calcolata sperimentalmente per òa bottom camera 
   const = 0.00146
   
   gain_y = 0.5
   
   #Point 1 
   #xb1 = -(const) * ((y1 * original_image_shape[1])/v0 -  original_image_shape[1])* drone_obj.z
   xb1 = -(const) * ((v1  -  v0 )* drone_obj.z)
   #yb1 =  -(const) * (x1 - u0)* drone_obj.z
   
   #yb1 =  -(const) * ((x1 * original_image_shape[0])/u0 - original_image_shape[0] )* drone_obj.z
   yb1 =  -1*(const) * ((u1  - u0 ) * drone_obj.z)
   #Point2 
   xb2 = -(const) * ((v2 - v0 )* drone_obj.z)
   yb2 =  -1*(const) * ((u2 - u0  )* drone_obj.z)
   

   
#    print('v0: ', v0)
#    print('u0: ', u0)
   # print('original_image_shape[0]: ', original_image_shape[0])
   # print('original_image_shape[1]: ',original_image_shape[1])
#    print('xb1: {0} v1: {1}'.format(xb1, v1))
#    print('yb1: {0} u1: {1}'.format(yb1, u1))
#    print('xb2: {0} v2: {1}'.format(xb2, v2))
#    print('yb2: {0} u2: {1}'.format(yb2, u2))
    
   print('######### PANEL LINE RECOGNIZED, FOLLOWING')
   # Publish over the air The message with control points 
   publish_navigation_RGB_point(xb1, yb1, xb2, yb2)

   
def clustering_lines(start_point, end_point, points, row, col, angular_coefficient, angular_line_versors,area):
    """
    La funzione unisce, se possibile, le diverse shape rivlevate, in uno o piu cluster.
    I cluster vengono uniti se le shape presentano una direzione della retta di regressione simile e se distano meno di ua cerat distanza definita i
    pixel. Se una shape ha un area maggiore del threshold th5 allora viene considerata come singola e non accoppiata per direzione ad altre shaepe.
    Tuttavia il single cluster puo comunque essere unito ad altre shape sucessivamente.
    """
    point_line_distance_actual = []
    same_cluster_similar_direction = []
    same_cluster = []
    sum_list = []
    new_list = []
    count1 = 0
    
    #Consodero tutte le regioni che sono state trovate in processing_image    
    for ii in range(0, len(start_point)):
        if area[ii] > th5:
            #Se area è maggiore di un certo threshold il cluster viene considerato come singolo, quindi non viene unito ad una coppia 
            print('SINGLE CLUSTER')
            same_cluster.append([ii])
            #print("same_cluster: ", same_cluster)
          
            continue
       
        #Per ciascuna regione estrapolo i versori 
        vx = angular_line_versors[ii][0][0]
        vy = angular_line_versors[ii][0][1]
       
        same_cluster1 = []
        same_cluster1.append(ii)
      
      
        
        m = -end_point[ii][0][1] +start_point[ii][0][1]
        b = (end_point[ii][0][0] - start_point[ii][0][0])
        c = -1*(-end_point[ii][0][1]* start_point[ii][0][0] + end_point[ii][0][0] * start_point[ii][0][1])
       
        
        #Considero una seconda regione jj 
        for jj in range(0,len(start_point)):
            
            if start_point[jj][1][0] == ii:
                #Se regione ii = regione jj passo oltre --> non posso considerare due regioni uguali
               continue
            else:
              #Estrapolo versori retta regressione regione jj
               vx_jj = angular_line_versors[jj][0][0]
               vy_jj = angular_line_versors[jj][0][1]
               #siccome versori, gia allineati agli assi posso sommare le componenti dei descrittori delle due rette
               # delle regioni ii e jj per ottenere la magnitudine della risultante V
               Vx = vx - vx_jj 
               Vy = vy - vy_jj
               V = math.sqrt(Vx*Vx + Vy*Vy)
               
              
               print('ii: {0} jj: {1}'.format(ii, jj))
               #print('row_ii_start: {0}  row_jj_start: {1}  row_ii_end: {2} row_jj_end: {3}'.format(start_point[ii][0][0],start_point[jj][0][0], end_point[ii][0][0], end_point[jj][0][0]))
              # print('col_ii_start: {0}  col_jj_start: {1}  col_ii_end: {2} col_jj_end {3}'.format(start_point[ii][0][1],start_point[jj][0][1], end_point[ii][0][1], end_point[jj][0][1]))

              
                    # print('sono qui')
              # print('ii: {0} jj: {1}'.format(ii, jj))
              
             #  print('vx: {0}  vy: {1}  vx_jj: {2} vy_jj: {3}'.format(vx, vy, vx_jj, vy_jj))
              # print('V: ', V)
              
               
              #Considero la distanza tra le rette ii e jj. Se distanza = 0 allora le due rette si intersecano
               for mm in range(0, len(points)):
                   if (points[mm][1][0] == jj):
                      
                       x = points[mm][0][0]
                       y = points[mm][0][1]
                       
                       # point_line_distance.append([[np.abs(m * x + b*y + c)/math.sqrt(m*m + b*b)],[points[jj][1][0]]])
                       point_line_distance_actual.append(np.abs(m * x + b*y + c)/math.sqrt(m*m + b*b))
               
            
               #Se la coppia di rette ii e jj presenta una magnitudine del ettore risultatnte V generato dai diversi versori < th 
               #Allora entro in if
               if (V < 0.5):
               #In same_cluster_similar_direction insierisco le rette ii e jj indipendentemente se la 
               # loro min(point_line_distance_actual) rispetta il threshold sucessivo
                   same_cluster_similar_direction.append([ii]) 
                   same_cluster_similar_direction.append([jj])
                   #print('same_cluster_similar_direction: ', same_cluster_similar_direction)
                   if (min(point_line_distance_actual) < 200): 
                       #Le due retteii e jj formano una coppia.
                        same_cluster.append([ii, jj])
                        same_cluster1.append(jj)
            
               point_line_distance_actual = []
               count1 = count1 + 1
               
               
       # if ( (sum(same_cluster1) in sum_list) == True or len(same_cluster1) == 1):
            #continue
      
        sum_list.append(sum(same_cluster1))
        # print('sum_list: ',sum_list)
        new_list.append(same_cluster1)
       # print('-------> new_list: ', new_list)
    #print('-------> same_cluster: ', same_cluster)
    
    
     #############Commentare nel caso si volessse riconoscere meno pannelli ma con una precisione maggiore
    if len(same_cluster) == 0:
       #Se nessuna coppia è formata ma cis ono regioni con direzionii simili allora le considero comunque 
        #---> Utilizzo principale nella fine della vela
        new_list = same_cluster_similar_direction
       
    
   
    if len(same_cluster) > 0:
        #Verifico se riconosco pannelli che non vengono clutserizzati
        #ma che la retta di regressione possiede stessa inclinazione ma nessuna intersezione
        for ii in range(0, len(same_cluster_similar_direction)): ##Scorro tutti i cluster con direzione simile ma senza intgersezione 
            for jj in range(0, len(same_cluster)): #Scorro tutti gli elementi in same cluster, sia coppie che non 
                for kk in range(0, len(same_cluster[jj])): #Considero ciascun elemento
                    if same_cluster[jj][kk] == same_cluster_similar_direction[ii]: #Scarto gli elemti uguali, non posso unire un cluster a se stesso
                        continue
                    else:
                        #Inserisco in new_list regioni che non sono accoppiate ma che comunque hganno direzione simile 
                        new_list.append(same_cluster_similar_direction[ii])
                       
    ####################                
    #Unisco cluster ove sono presenti stesse regioni al fine di formare coppie, terne o quaterne di regioni che possono formare 
    # un unico cluster. --> Viee fatto perche fino ad ora la dimensione del massima del cluster erano coppie 
    # ma in piu coppie poteva cpmaprire la stessa regione associata con regioni differenti               
    #unisco le coppie o i singoli elemnti che presentano le stesse shape in cluster: esempio [2,1] [3,1] [5] [5,2] ---> [2,3,1,5]  
    G = to_graph(new_list)
    s = connected_components(G) 
    new_list = []
    for ss in s:
        new_list.append(list(ss))
       # print(new_list)
  
    
    
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
        
    print('-----> same_cluster: ',same_cluster )
    return same_cluster, theta_mean




def image_pre_processing(image,line_versors_old,  drone_obj, counter_frame_analyzed, counter_frame_received):
    ########## Rotation and resize of image ###########
    # image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    flag_final_frame_analyzed = False
    flag_frame_analyzed = False
    debug_image_publication = True 
    
    #Counting frame received for statistical analysis
    if counter_frame_analyzed > 1:
         counter_frame_received = counter_frame_received + 1
         
         
    th3_value = th3
    th4_value = th4
    
    percent =70 #70
    #Resize Image 
    width = int(image.shape[1] * percent/ 100)
    height = int(image.shape[0] * percent/ 100)
    dim = (width, height)
    
    image = cv2.resize(image, dim, interpolation =cv2.INTER_AREA)
    
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    if debug_image_publication:
        publish_img_hsv(img_hsv) 
    
    # cv2.imshow('img_hsv', img_hsv)
    # cv2.waitKey(1)
   
   
#--> Nella decodifica BGR2HSV: H --> B 
#                              s---> G
#                              V --> R

#IMPORTANTE: Nella decodifica BGR2HSV il blu rientra nei parametri:
#     lower_blue = np.array([100,130,0], np.uint8)
#     upper_blue = np.array([140,255,255], np.uint8)

#I pannelli su carta invce:
#    #Devo camboare parametri trasformazione rispetto codice in image_analysis ---> capire perche cambia la trasformazione
    lower_blue = np.array([90,40,40], np.uint8)  #Valori pannello in lab [80,0,40]
    upper_blue = np.array([180,180,220], np.uint8) #[160,100,220]
    
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(img_hsv, lower_blue, upper_blue)
    
    blur1 = cv2.bilateralFilter(mask,15,75,75)
    
    
    thresh = cv2.convertScaleAbs(blur1)
    
   # print(thresh.shape)
    #Banda nera per evitare venga definito come cluster unico
    # for jj in range(213, 346):
    #     for ii in range(0, thresh.shape[1]):
    #         thresh[jj][ii] = 0
   
    # if debug_image_publication:
    #     publish_img_first_threshold(thresh1)
    
    # cv2.imshow('First Threshold', thresh)
    # cv2.waitKey(1)
     
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
   
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
    
    area = []
    single_cluster = []
    counter_cnt = 0
    count = 0
  
    ############## INDIVIDUZIONE DEI DIVERSI CLUSTER E DELLE RISPETTIVE RETTE DI REGRESSIONE ############
    for cnt in contours: 
      
        if cv2.contourArea(cnt) < 5000000 and cv2.contourArea(cnt) > th3_value:
          # print(' cv2.contourArea(cnt): ', cv2.contourArea(cnt))
          # print('th2_value:  {0}, th3_value: {1}'.format(th2_value, th3_value))
           epsilon = th4_value*cv2.arcLength(cnt,True)   #0.1 is 10% of arc length
           out_table = cv2.approxPolyDP(cnt, epsilon, True)
          
           #cv2.drawContours(image, [out_table], -1, (0, 255, 0), 3)
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
           angular_coefficient.append([[(thresh.shape[1]-1)/(righty - lefty)],[count]])
               
           #cv2.line(image,(thresh.shape[1]-1,righty),(0,lefty),255,2) 
           # cv2.line(image,(righty,thresh.shape[1]-1),(lefty,0),255,2) 
         
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
               
                          
           #print('actual_points[0]: ', actual_points[0])
           start_point.append([actual_points[0],[count]])
               
           end_point.append([actual_points[-1],[count]])
           #print('start_point: ', [actual_points[0],[count]])
           #print('[end_point: ', [actual_points[-1],[count],[count]])                                     
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
           
           min_row = min(pts[0]) #asse y
           col_related_min_row = pts[1][np.where(pts[0] == min(pts[0]))[0][0]]  #asse x
           
           max_col = max(pts[1])
           row_related_max_col = pts[0][np.where(pts[1] == max(pts[1]))[0][0]]
           
           max_row = max(pts[0])
           col_related_max_row = pts[1][np.where(pts[0] == max(pts[0]))[0][0]]
           #Queste coordinate mi serviranno per capire quale cluster si trova sopra l'altro e per unire la max row di quello sopra con la min row di quello sotto in modo da avere un box finale
           
           # print('max_row ', max_row)
           # print('col_related_max_row ', col_related_max_row)
           region_pixels_size_coo.append([[min_row, col_related_min_row], [ row_related_max_col, max_col],  [max_row, col_related_max_row], [count]])
           # print('region_pixels_size_coo ', region_pixels_size_coo)
           #[minore riga, colonna corrispondente], [riga corrispondente, max colonna ][max riga, colonna corrispondente]
                                           
           pixels_iniside_area.append([lst_points])
           # print('pixels_iniside_area: ', pixels_iniside_area)
           ############################
           actual_points = []

           area.append(cv2.contourArea(cnt))
           
           #Se l'area maggiore di un certo threshold allora il pannello puo essere considerato come un singolo cluster, in quanto non ci sarebbero coppie da formare e in fase di unione dei cluster verrebbe scartato
           if cv2.contourArea(cnt) > th5:
               single_cluster.append(img)
           countours_considered.append([out_table])
           count = count + 1
    
    # imgplot = plt.imshow(image,cmap='gray', vmin = 0, vmax = 255)
    # plt.show()    
    #cv2.imshow('image', image)
    #cv2.waitKey(1)


    if len(start_point) == 1:
        cluster = [[0]]
    else:
        #CLUSTERING LINES 
        cluster , theta_mean= clustering_lines(start_point, end_point, points, thresh.shape[0], thresh.shape[1], angular_coefficient, angular_line_versors, area)   
    
    new_region = []
    final_countours = []
    clusters = []
    same_mask = []
    flag_difference = False
    line_versors = []
    line_versors_final = []
    
    
    
    ######### CREO UNA MASCHERA PER CIASCUN CLUSTER ###############
    
    for ii in range(0, len(cluster)):
       mask = np.zeros_like(image) 
       # theta_mean_cluster_area = theta_mean[ii]
       # print(theta_mean_cluster_area)
       for area in range(0,len(cluster[ii])):

           for pixels in range(0, len(pixels_iniside_area[cluster[ii][area]][0][0][0][0])):
                
                 list_considered_row = pixels_iniside_area[cluster[ii][area]][0][0][0][0][pixels] #Prendo le righe relative la regione 1
                 list_considered_col= pixels_iniside_area[cluster[ii][area]][0][0][1][0][pixels] #Prendo le colonne relative la regione 1
                  
                 mask[list_considered_row][list_considered_col] = 250
       
    
       if debug_image_publication:
           publish_img_clustering_mask(mask)
    #    cv2.imshow('Clustering Mask', mask)
    #    cv2.waitKey(1)
   
       mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
       mask = cv2.convertScaleAbs(mask)
       _, contours, _= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  ### in python 2 bisogna scrivere _, contours, _
       
       contours1 =[]
       
       for hh in range(0, len(contours)):
           for hh1 in range(0, len(contours[hh])):
               
                contours1.append(contours[hh][hh1])
      
    ##### FInd equation of the survived lines 
       [vx,vy,x,y] = cv2.fitLine(np.float32(contours1),cv2.DIST_L2,0,0.01,0.01)
       #Appendo versori e punti 
       line_versors.append([vx, vy, x, y]) 
       print('x:  {0}, y: {1}'.format(line_versors[ii][2][0], line_versors[ii][3][0]))
    
    
       for cnt in contours: 
            if cv2.contourArea(cnt) < 50000000 and cv2.contourArea(cnt) > th3_value:
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
            #       cv2.line(image, (y_down, image.shape[0]), (b0, 0), (255,0,0), 2) 
            # else:
            #       cv2.line(image, (image.shape[0], y_down), (0, b0), (255,0,0), 2)    
     
       
       try:      
           cv2.line(image,(thresh.shape[1]-1,righty),(0,lefty),255,2)   
       except:
           pass       
     
    
       # imgplot = plt.imshow(image,cmap='gray', vmin = 0, vmax = 255)
       # plt.show()  
    
    
       #cv2.imshow('image', image)
       #cv2.waitKey(1)
   
    
        #####     VOTING TECNIQUE RESPECT t-1 frame #####################
    voting_array = [None] *  len(line_versors)
   
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
                
                if V < 0.2:
                   final_line_versors_frame.append([vx_jj, vy_jj, line_versors[jj][2][0], line_versors[jj][3][0]])
                  #Mantain perpendicular lines
                elif ( V < 1.40 and V > 1): #perpendicular lines
                    if (voting_array[jj] == None):
                        voting_array[jj] = 1
                    else:
                         voting_array[jj] = voting_array[jj] + 1
                         
                else:
                  #Probabili linee da scartare 
                     if (voting_array[jj] == None):
                        voting_array[jj] = 1
                     else:
                         voting_array[jj] = voting_array[jj] + 1
   
    #--------> Check max voting value 
    #Definisco percentuale limite oltre la quale la retta va scartata  
    #LA retta va scartata se è votata da piu dell'80% delle rette presenti
    limit_percentage = 0.2 * len(line_versors)  
    for ii in range(0, len(voting_array)):
        #Definsico le rette da mantenere
        if voting_array[ii] >= limit_percentage: 
            final_line_versors_frame.append([line_versors[ii][0][0], line_versors[ii][1][0], line_versors[ii][2][0], line_versors[ii][3][0]])
            print('voting_array[ii]: ', voting_array[ii])
    #Create array for line points 
    line_points = []
    cartesian_distance_point_from_image_center = []
    point_line_distance_actual = []
    mean_distance_of_line_from_center_line = []
     
    for ii in range(0, len(final_line_versors_frame)):
        #print('final_line_versors_frame[ii]: ', final_line_versors_frame[ii])
        
        ####Definisco rette sopravvisutte alla filtrazione rispetto frame precedente 
        vx = final_line_versors_frame[ii][0]
        vy = final_line_versors_frame[ii][1]  
        x =  final_line_versors_frame[ii][2]  
        y =  final_line_versors_frame[ii][3]  
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
           
       
          #### Per il controllo considero la retta che ha la distanza media di tuttii punti
       ### piu vicina alla retta centrae delle foto
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
        
        # a = math.sqrt(pow(x-thresh.shape[0]/2,2 ) + pow(y-thresh.shape[1]/2,2))
        # cartesian_distance_point_from_image_center.append(a)
        
        
        try:      
           cv2.line(image,(thresh.shape[1]-1,righty),(0,lefty),255,2)   
        except:
            pass
    # print('cartesian_distance_point_from_image_center: ', cartesian_distance_point_from_image_center)
   
    
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
          
             ########## Exportation del punto (x,y) della retta e del punto Pv = (x + vx, y + vy) nel body frame del drone (considerando Ar drone 2.0)
            line_point_exportation_in_drone_body_frame(line_considered[0],line_considered[1],line_considered[2],line_considered[3], (thresh.shape[0]/2), (thresh.shape[1]/2), drone_obj)  #Definsico locazione del body frame del drone rispetto l'image plane (esattamente al centro )
            flag_final_frame_analyzed = True
            flag_frame_analyzed = True
      
    if flag_final_frame_analyzed == True:
         counter_frame_analyzed = counter_frame_analyzed + 1
         flag_final_frame_analyzed  = False
         
    return  image, line_versors, counter_frame_analyzed, counter_frame_received,flag_frame_analyzed
    
    # return image






# def load_RGB_image(photo_number, th3_value, th4_value, rotation_degree):
#      name = '/home/lucamora/image_analysis/Volo_01/DJI_0' + str(photo_number) + '.jpg'
#      image = cv2.imread(name) 
     
#      cluster_image = image_pre_processing(image,photo_number, th3_value, th4_value, rotation_degree)
     
#      return cluster_image
     


# def load_video(video_name):
#      cap = cv2.VideoCapture(video_name)
#      frame_count = 0
#      while frame_count < 6000:
#          ret, frame = cap.read()
#          frame_count = frame_count + 1
         
#          blur1, distance_matrix = image_pre_processing(frame,11, th3, th4)
#          if cv2.waitKey(1) & 0xFF == ord('q'):
#              break
#      cap.release()
#      cv2.destroyAllWindows()  
#      return frame


# video = False
# photo_number = 643
# rotation_degree = 0
# for ph in range(0,1):
#     # rotation_degree = np.random.randint(0, 90, size=(1, 1))
#     photo_counter = photo_number + ph
#     video_name = '/home/lucamora/image_analysis/Volo_01/DJI_0002.mp4' 
#     if video == True:
#          load_video(video_name)
#     else:
    
#          image = load_RGB_image(photo_counter, th3, th4, rotation_degree)
         
#          imgplot = plt.imshow(image)
#          plt.show()          
       
def draw_estimated_line(P1, P2, image, drone_obj):
    
    const = 0.00146
  
    u0 = image.shape[1] /2
    v0 = image.shape[0] /2

    v1 = v0 - P1.x/(drone_obj.z *const) #righe --> x
    u1 = u0 -P1.y/(drone_obj.z *const) #colonne --> y
    
    v2 = v0 - P2.x/(drone_obj.z *const)
    u2 = u0 -P2.y/(drone_obj.z *const)
    
    #Find the equation of the line in camera frame
    a = (u2 - u1)/(v2- v1)
    b = 1
    c = (-1*a  * v1 ) + u1
    c = abs(c)
    
    print("a: {}, c: {}".format(a,c))
    #Find all the points that lies on the line 

    cv2.line(image, (int(a*image.shape[0] + c), image.shape[1]), (int(c), 0), (0,0,255), 2)
    # cv2.imshow('KF Line', image)
    # cv2.waitKey(1)
    


def listener():
    rospy.init_node('termo_frame_elaboration', anonymous=True)
    #Create Image Publisher
    Camera_elaborated_frame_pub = rospy.Publisher('camera_vision_output_RGB', Image, queue_size=10)
    

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
        #Take image frame
        
        image = take_drone_camera_RGB_frame()
         #Subdivide RGB and Thermal Images 
        width = image.shape[1]
        width_cut_off = int(width/2)
        image_RGB = image[:, :width_cut_off]
        image_thermo = image[:, width_cut_off:]

   
  
        clustered_image, line_versors,counter_frame_analyzed_out, counter_frame_received_out, flag_frame_analyzed = image_pre_processing(image_RGB, line_versors_old, drone_obj,counter_frame_analyzed, counter_frame_received)
        
        counter_frame_analyzed = counter_frame_analyzed_out
        counter_frame_received = counter_frame_received_out
        
        #Write on txt file 
        #file1.write(str(counter_frame_received ) + "," + str(counter_frame_analyzed)+ "\n") 
        #file1.flush()
           
        #Add estimated KF line before publishing the image 
        # if flag_frame_analyzed == True:
         
        #    P1 = receive_estimated_control_point_P1()
        #    P2 = receive_estimated_control_point_P2()
        #    draw_estimated_line(P1, P2, clustered_image, drone_obj)
       
        #Publish output frame in rostopic
        publish_output_RGB_image(clustered_image)
        line_versors_old = line_versors
        end2= time.time() - start1
        #file2.write(str(end2 ) + "\n") 
        # file2.flush()
        print('##################### end2: ', end2)
       
      
        count = count + 1
   
      
if __name__ == '__main__':
    listener()
   # file1.close()
