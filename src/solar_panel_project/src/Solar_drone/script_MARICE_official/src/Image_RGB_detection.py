#!/usr/bin/env python
# -*- coding: utf-8 -*-


import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import cv2
import math
import networkx 
import time
from networkx.algorithms.components.connected import connected_components

from ros_simulation_data import take_drone_camera_frame, takeEnvObservations, receive_estimated_control_point_P1, receive_estimated_control_point_P2
from ros_simulation_data import publish_navigation_RGB_point
from ros_simulation_data import publish_navigation_Thermo_point2
from ros_simulation_data import publish_output_image
from ros_simulation_data import pub_gray_image, pub_th_image, pub_distance_image



class Image_RGB_elaboration:
    """
    La classe Image_RGB_elaboration contiene tutto il corpo delle funzioni necessarie per 
    l'RGB detection dei pannelli
    """
    def __init__(self):
        
        #Initialize Detection Parameters 
        self.th3 = 1500 #area
        self.th4 = 0.01  #epsilon aproxPolydp 
        self.th5 = 51200  #70000  #Area oltre il quale il cluster è considerato unico
        self.th6 = 0 #threshold decisio nel codice --> il rettangiolo di boundary box deve essere minore di una certa percentuale di area dell'immagine --> (Banalmente non puo essere tutta l'immaginen)
        self.index_th1 = 0
        self.index_th2 = 0
        self.index_th3 = 0
        self.index_th4 = 0
        
        #Define HSV Parameters 
        self.H_MIN = 0
        self.S_MIN = 0
        self.V_MIN = 0

        self.resize_percent = 0
        
        self.width = 0
        self.height = 0
        #General_parameters 
        self.lefty_array = []
        self.rigthy_array = []
        self.start_point = []
        self.end_point = []
        self.points = []
        self.angular_coefficient = []
        self.actual_points = []
        self.angular_line_versors = []
        self.countours_considered = []
        self.pixels_iniside_area = []
        self.region_pixels_size_coo = [] # coordinate of min col min row pixel, of max col min row pixel and max col, min row pixel
        
        self.mask_width = 0.0
        self.mask_width_array = []
        self.desired_mask_width = 0.0 #--> Verrà passata da script di calibrazione 
        self.central_cluster_obtained = False
        mask= []
        self.area = []
        self.single_cluster = []
        self.counter_cnt = 0
        self.count = 0
        self.flag_final_frame_analyzed = False
        self.counter_frame_analyzed = 0
        
      
        #parameters relatively to optimization
        self.parameters_optimization = False
        self.optimized_param = [] 
        self.optimized_param_old = np.zeros(3)
        self.selection = 0  #Relative to the User Parameters set selection
        #Secondo set di parametri
        new_region = []
        final_countours = []
        clusters = []
        same_mask = []
        flag_difference = False
        line_versors = []
        line_versors_final = []



        pass

    def to_edges(self, l):
        """ 
            treat `l` as a Graph and returns it's edges 
            to_edges(['a','b','c','d']) -> [(a,b), (b,c),(c,d)]
        """
        it = iter(l)
        last = next(it)
    
        for current in it:
            yield last, current
            last = current    


    def to_graph(self, l):
        G = networkx.Graph()
        for part in l:
            # each sublist is a bunch of nodes
            G.add_nodes_from(part)
            # it also imlies a number of edges:
            G.add_edges_from(self.to_edges(part))
        return G

    
    #Function related to Optimization Problem 
    def find_rectangles_in_image(self, sd,c, drone_obj, image):
        """
            c = countours 
            sd = classe per shape detection
            image = binary mask
            
            Output:
            shape, x, y, w, h = "rectangle", (x,y) coo vertice in alto a sinistra rettangolo, height width in px
            sd.coo_list = [shape, x, y, w, h]
        """
              
    
    
        #Mettere sogliatura grandezza
          # shape using only the contour
        M = cv2.moments(c)
            
            #Return the shape and the position of the shape
        shape, x, y, w, h, approx = sd.detect(c, 0.01)
       
    
        
        c = c.astype("float")
        c *= 1
        c = c.astype("int")
        
        #insert coo of object detected in vector
        sd.coo_list.append([shape,x,y,w,h])
        #sd.area_list.append([shape,  cv2.contourArea(c)])
        sd.area_list.append([shape,  w*h])
        # print("--------> AREA: ", w*h)
        # if img2 == True:
        #    print("IMG2")
        #    print(" sd.area_list: ",   sd.area_list)
        # else:
        #    print("IMG1")
        #    print(" sd.area_list: ",   sd.area_list)
        #print("AREA: ", cv2.contourArea(c))
       # print("AREA 2: " , w*h)
        # print(" sd.coo_list: ",  sd.coo_list)
        #print(" sd.area_list: ",   sd.area_list)
    
    
    
    
    def  first_cluster_search(self, contours, thresh, image):
      
        self.lefty_array = []
        self.rigthy_array = []
        self.start_point = []
        self.end_point = []
        self.points = []
        self.angular_coefficient = []
        self.actual_points = []
        self.angular_line_versors = []
        self.countours_considered = []
        self.pixels_iniside_area = []
        self.region_pixels_size_coo = [] # coordinate of min col min row pixel, of max col min row pixel and max col, min row pixel
        self.area = []
        self.single_cluster = []
        count = 0
        
        self.width = image.shape[0]
        self.height =  image.shape[1]
        self.th6 = (self.width* self.height) - ( (self.width* self.height * 10)/100)
        #print("self.th6: ", self.width* self.height)
        for cnt in contours: 
              
            if cv2.contourArea(cnt) < self.th6 and cv2.contourArea(cnt) > self.th3:
                # print('th2_value:  {0}, th3_value: {1}'.format(th2_value, th3_value))
                epsilon = self.th4*cv2.arcLength(cnt,True)   #0.1 is 10% of arc length
                out_table = cv2.approxPolyDP(cnt, epsilon, True)
               
                # cv2.drawContours(image, [out_table], -1, (0, 255, 0), 3)
                #ritorna i vettori normalizzati lungo x e y e un punto xo yo sulla retta
                [vx,vy,x,y] = cv2.fitLine(out_table,cv2.DIST_L2,0,0.01,0.01)
               
              # intersezione della retta con asse y della figura nel punto x = 0 
                lefty= int((-x*vy/vx)+y)
                # print('lefty:',lefty)
              # intersezione della retta con asse y della figura nel punto x = thresh.shape[1]
                righty = int(((thresh.shape[1]-x)*vy/vx)+y)
              # evaluate line angular coefficient:
                if righty - lefty == 0:
                    righty= righty + 1
                self.angular_coefficient.append([[(thresh.shape[1]-1)/(righty - lefty)],[count]])
                    
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
                            self.points.append([[ii, int(((ii-x)*vy/vx)+y)],[count]])
                            self.actual_points.append([ii, int(((ii-x)*vy/vx)+y)])
                            # print('points: ',  [ii, int(((ii-x)*vy/vx)+y)], [count])
                            # if count == 8:
                            #     time.sleep(1000)
               
                #Puo succedere che se la retta è molto verticale non vemgono trovati valori interi di y (righe) per ii che varia lungo asse x (colonne)
                #Se succede cerco valori di x al variare di y:
                if (len(self.actual_points) == 0):
                    for ii in range(0, thresh.shape[0]):
                        if ((y - ii)*(1/(vy/vx))+x >= 0 and ((y - ii)*(1/(vy/vx))+x  <= thresh.shape[1])): #Cerco un valore di x quindi scorro su bordo orizzontale
                               self.points.append([[int((y - ii)*(1/(vy/vx))+x), ii],[count]])
                               self.actual_points.append([int((y - ii)*(1/(vy/vx))+x), ii])
                    
                               
                # print('self.actual_points[0]: ', self.actual_points[0])
                self.start_point.append([self.actual_points[0],[count]])
                    
                self.end_point.append([self.actual_points[-1],[count]])
                # print('start_point: ', [self.actual_points[0],[count]])
                # print('[end_point: ', [self.actual_points[-1],[count],[count]])                                     
                self.angular_line_versors.append([[vx, vy], [count]])                
                 
              # ############################ 
              # Per conoscere i pixel dentro il contorno creo una maschera con valore 1 per i pixel dentro il cotorno
                lst_points = []
 
              #  Create a mask image that contains the contour filled in
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
                self.region_pixels_size_coo.append([[min_row, col_related_min_row], [ row_related_max_col, max_col],  [max_row, col_related_max_row], [count]])
                # print('self.region_pixels_size_coo ', self.region_pixels_size_coo)
                #[minore riga, colonna corrispondente], [riga corrispondente, max colonna ][max riga, colonna corrispondente]
                                                
                self.pixels_iniside_area.append([lst_points])
                # print('self.pixels_iniside_area: ', self.pixels_iniside_area)
                ############################
                self.actual_points = []
 
                self.area.append(cv2.contourArea(cnt))
                
                #Se l'area maggiore di un certo threshold allora il struttura puo essere considerato come un singolo cluster, in quanto non ci sarebbero coppie da formare e in fase di unione dei cluster verrebbe scartato
                if cv2.contourArea(cnt) > self.th5:
                    self.single_cluster.append(img)
                self.countours_considered.append([out_table])
                count = count + 1
               
 
     
    
    def clustering_lines(self, row, col):
        #Internal parameters input from class RGB: start_point, end_point, points, angular_coefficient, angular_line_versors, area
        
        point_line_distance_actual = []
        same_cluster_similar_direction = []
        same_cluster = []
        sum_list = []
        new_list = []
        count1 = 0
        
        # t3_start = process_time()  
       
       
        
        for ii in range(0, len(self.start_point)):
            if self.area[ii] > self.th5:
                print('SINGLE CLUSTER')
                same_cluster.append([ii])
                #print("same_cluster: ", same_cluster)
                #Se area è maggiore di un certo threshold il cluster viene considerato come singolo, quindi non viene unito ad una coppia 
                continue
           
            vx = self.angular_line_versors[ii][0][0]
            vy = self.angular_line_versors[ii][0][1]
            
            # print('x: ',start_point[ii][0][0])
            # print('y: ',start_point[ii][0][1])
            
            same_cluster1 = []
            same_cluster1.append(ii)
          
            # print('m: ',m)
            # print('b: ',b)
            # print('c: ',c)
            
            m = -self.end_point[ii][0][1] + self.start_point[ii][0][1]
            b = (self.end_point[ii][0][0] - self.start_point[ii][0][0])
            c = -1*(-self.end_point[ii][0][1]* self.start_point[ii][0][0] + self.end_point[ii][0][0] * self.start_point[ii][0][1])
           
            
            #Considero la retta ii e calcolo la distanzadella retta ii da tutti i punti che compongono le restanti rette jj
            for jj in range(0,len(self.start_point)):
                
                if self.start_point[jj][1][0] == ii:  #considero il valore di count
                    
                   continue
                else:
                    # print('ii: {0} jj: {1}'.format(ii, jj))
                    vx_jj = self.angular_line_versors[jj][0][0]
                    vy_jj = self.angular_line_versors[jj][0][1]
                    
                    #siccome versori, gia allineati agli assi posso sommare le componenti dei descrittori delle due rette per ottenere la magnitudine della risultante V
                    Vx = vx - vx_jj 
                    Vy = vy - vy_jj
                    V = math.sqrt(Vx*Vx + Vy*Vy)
                    
                    # print(V)
                    # print('ii: {0} jj: {1}'.format(ii, jj))
                    # print('row_ii_start: {0}  row_jj_start: {1}  row_ii_end: {2} row_jj_end: {3}'.format(line_ii_start_x,line_jj_start_x, line_ii_end_x, line_jj_end_x))
                    # print('col_ii_start: {0}  col_jj_start: {1}  col_ii_end: {2} col_jj_end {3}'.format(start_point[ii][0][1],start_point[jj][0][1], end_point[ii][0][1], end_point[jj][0][1]))
        
                   
                         # print('sono qui')
                    # print('ii: {0} jj: {1}'.format(ii, jj))
                   
                    # print('vx: {0}  vy: {1}  vx_jj: {2} vy_jj: {3}'.format(vx, vy, vx_jj, vy_jj))
                    # print('V: ', V)
                   
                    
                    #per verificare l'intersezione dovrei fare un check sui valori di start ed end delle line sull'asse x, quindi sulle colonne
                    #Se due linee hanno i valori di start ed end minori o maggiori allora si intersecano
                    for mm in range(0, len(self.points)):
                        if (self.points[mm][1][0] == jj):
                            
                            x = self.points[mm][0][0]
                            y = self.points[mm][0][1]
                    #Distanza punto retta 
                            # point_line_distance.append([[np.abs(m * x + b*y + c)/math.sqrt(m*m + b*b)],[points[jj][1][0]]])
                            point_line_distance_actual.append(np.abs(m * x + b*y + c)/math.sqrt(m*m + b*b))
                    
                 
                    #Select lines based on their minimum distance and angular values  
                    #LA distanza minima deve essere prossima allo zero e il valore di V maggiore di 1, in quanto corrisponde alla risultante della somma dei vettori vx1 con vx2 vy1 vy2
                    if (V < 0.3):
                        same_cluster_similar_direction.append([ii])
                        same_cluster_similar_direction.append([jj])
                        # print('-------> same_cluster: ################################################')
                        if (min(point_line_distance_actual) < 20):   #(intersection_flag == True): 
                            same_cluster.append([ii, jj])
                            #print('-------> same_cluster: ', same_cluster)
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
            #ma che la retta di regressione possieda stessa inclinazione ma nessuna intersezione
            if len(same_cluster_similar_direction) > 0:
                for ii in range(0, len(same_cluster_similar_direction)):
                    for jj in range(0, len(same_cluster)):
                        #print("same_cluster[jj]: ", same_cluster[jj])
                        for kk in range(0, len(same_cluster[jj])):
                            if same_cluster[jj][kk] == same_cluster_similar_direction[ii]:
                              
                                continue
                            else:
                                new_list.append(same_cluster_similar_direction[ii])
            else:
                for jj in range(0, len(same_cluster)):
                    #print("same_cluster[jj]: ", same_cluster[jj])
                    new_list.append(same_cluster[jj])

                        
        ####################                
                        
        
        G = self.to_graph(new_list)
        s = connected_components(G)
        new_list = []
        for ss in s:
            new_list.append(list(ss))
            #print(new_list)
       
        
        if len(new_list) > 0:
            same_cluster = []
            same_cluster = new_list
        else:
            new_list = same_cluster
         
        theta = []
        theta_mean = []
        #### Find angle theta for each selected cluster 
        for ii in range(0, len(same_cluster)):
            for jj in range(0, len(same_cluster[ii])):
                
                vx = self.angular_line_versors[same_cluster[ii][jj]][0][0]
                vy = self.angular_line_versors[same_cluster[ii][jj]][0][1]
                a = math.atan(abs(vy)/abs(vx))
                theta.append(a * 180/np.pi)
            theta_mean.append(np.mean(theta))
            theta = []
              
      
        # #Eliminate outliers from same cluster in order to avoid 
            
        # print('theta_mean: ',theta_mean )
        return same_cluster, theta_mean
    


    
    def evaluate_panel_width_for_altitude(self, mask):
        """
        Function to evaluate altitude contriol.
        WHene the drone is in navigation mode it is evaluated an offset between 
        a desired width pixel value of the shape in the mask and an actual value.
        It is considered only the mask which shape is centered in the central part.
        The width evaluated is inserted in an array where the mean value is computed in order to avoid 
        jump and non linearities.
        """
        # mask.shape[0] --> height
        # mask.shape[1] --> width
        white_col = []
        #Considero la porzione di colonne centrali dell'immagine 
        for jj in range(0, mask.shape[0]):
            if (mask[jj][mask.shape[1]/2] == 250):
                white_col.append(jj)
            #se la lunghezza dell'array di pixel bianchi è maggiore di meta immagine in height creo la lista white_array
            if (len(white_col) > mask.shape[0]/3):
    
                white_array = []
                for ii in range(0,  mask.shape[1] ):
                    if (mask[0][ii] == 250):
                        white_array.append(ii)
              
                self.mask_width_array.append(len(white_array))
                self.mask_width = sum(self.mask_width_array) / len(self.mask_width_array)
                if (len(self.mask_width_array) > 20): #definisce lunghezza dell'array su cui calcolare la media
                    self.mask_width_array.pop(0)
                print("Width for ALtitude COntrol Detected: ", self.mask_width)
                break
            
            # if (jj > mask.shape[0]/2):
            #     white_col = []
            #     continue
        return self.mask_width





    def create_cluster_mask(self, cluster, image, thresh, drone_obj):
        """
        Create mask for each cluster detected in image 

        """

        line_versors = []
        line_versors_final = []
        
        ######### CREO UNA MASCHERA PER CIASCUN CLUSTER ###############
      
        mask = np.zeros_like(image) 
        final_mask = np.zeros_like(image)
        final_mask = cv2.cvtColor(final_mask, cv2.COLOR_BGR2GRAY)
        final_mask= cv2.convertScaleAbs(final_mask) 
       
        for ii in range(0, len(cluster)):
          
            mask= np.zeros_like(image) 
            for area in range(0,len(cluster[ii])):
                for pixels in range(0, len(self.pixels_iniside_area[cluster[ii][area]][0][0][0][0])):
                     
                    list_considered_row = self.pixels_iniside_area[cluster[ii][area]][0][0][0][0][pixels] #Prendo le righe relative la regione 1
                    list_considered_col= self.pixels_iniside_area[cluster[ii][area]][0][0][1][0][pixels] #Prendo le colonne relative la regione 1
                    mask[list_considered_row][list_considered_col] = 250
                    final_mask[list_considered_row][list_considered_col] = 250
            
            #PROVVISORIO
            
           
                
            # cv2.imshow('mask', mask)
            # cv2.waitKey(1) 
            # time.sleep()
            mask= cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            mask= cv2.convertScaleAbs(mask)

            
            self.mask_width = self.evaluate_panel_width_for_altitude(mask)
            
           
            _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	                cv2.CHAIN_APPROX_SIMPLE)
              
           
            contours1 =[]
            
            for hh in range(0, len(contours)):
                for hh1 in range(0, len(contours[hh])):
                    
                     contours1.append(contours[hh][hh1])

                  
            
        ### ## FInd equation of the survived lines 
            [vx,vy,x,y] = cv2.fitLine(np.float32(contours1),cv2.DIST_L2,0,0.01,0.01)
            #Appendo versori e punti 
            line_versors.append([vx, vy, x, y]) 
            # print('x:  {0}, y: {1}'.format(line_versors[ii][2][0], line_versors[ii][3][0]))
         
            
            for cnt in contours: 
                if cv2.contourArea(cnt) < self.th6 and cv2.contourArea(cnt) > self.th3:
                 # print('th2_value:  {0}, th3_value: {1}'.format(th2_value, th3_value))
                    th4_value = 0.03
                    epsilon = th4_value*cv2.arcLength(cnt,True)   #0.1 is 10% of arc length
                    out_table = cv2.approxPolyDP(cnt, epsilon, True)
                    
                   
                    # if (Op.test == True):
                    cv2.drawContours(image, [out_table], -1, (0, 255, 0), 3)
                    #Definisco se l'elemento è un rettangolo e la sua posizione
                    a = math.atan(abs(vy)/abs(vx))
                   
                   
                   # self.find_rectangles_in_image(sd, cnt, drone_obj, mask.copy()) #Se ci fosse piu di un pannello la chima apiu volte in base ai cluster trovati. --> Bisognerebbe poi sommare le funzioni di costi

                    lefty= int((-x*vy/vx)+y)
                     # print('lefty:',lefty)
                    #intersezione della retta con asse y della figura nel punto x = thresh.shape[1]
                    righty = int(((thresh.shape[1]-x)*vy/vx)+y)
                    #evaluate line angular coefficient:
                     
                    
                  # print('m_final: ', m_final)
                #cv2.line(image, (y_down, image.shape[0]), (b0, 0), (255,0,0), 2)
                # if theta_mean_cluster_area > 60:
                #       cv2.line(image, (y_down, image.shape[0]), (b0, 0), (255,0,0), 2) 
                # else:
                #       cv2.line(image, (image.shape[0], y_down), (0, b0), (255,0,0), 2)    
        
           
            try:      
                cv2.line(image,(thresh.shape[1]-1,righty),(0,lefty),255,2)   
            except:
                pass       
          
       
         
        if (len(cluster) == 0):
            mask= np.zeros_like(image) 

            mask= cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            mask= cv2.convertScaleAbs(mask)
        
        # cv2.imshow('final_mask', final_mask)
        # cv2.waitKey(1) 
        return line_versors, mask, image

    
    
    
    def voting_technique(self, line_versors, line_versors_old, thresh):
        """ 
        Voting Technique respect frame t-1
        """
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
                    
                    if V < 0.3:
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
        limit_percentage = 0.1 * len(line_versors)  
        for ii in range(0, len(voting_array)):
            #Definsico le rette da mantenere
            if voting_array[ii] >= limit_percentage: 
                final_line_versors_frame.append([line_versors[ii][0][0], line_versors[ii][1][0], line_versors[ii][2][0], line_versors[ii][3][0]])
                # print('voting_array[ii]: ', voting_array[ii])
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
                ## piu vicina alla retta centrae delle foto
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
          
        return  point_line_distance_actual, mean_distance_of_line_from_center_line, line_points


          
    
    def evaluate_control_line(self, point_line_distance_actual, line_points):
        """
        """
        mean_distance_of_line_from_center_line = []
       
        mean_distance_of_line_from_center_line.append(np.mean(point_line_distance_actual)) 
        
        if (len(mean_distance_of_line_from_center_line) > 0):
                # (len(cartesian_distance_point_from_image_center) > 0):
        #Ordering vector from lowest to highest value
          
            min_line = min(mean_distance_of_line_from_center_line)
    
             #Take the index of the first line 
            try:
                index = mean_distance_of_line_from_center_line.index(min_line)
            except:
                index = 1 #Prende la rpima linea se non trova l'index desiderato
            
           
            line_considered = line_points[index]
          
            #line_point_exportation_in_drone_body_frame(line_considered[0],line_considered[1],line_considered[2],line_considered[3], (thresh.shape[0]/2), (thresh.shape[1]/2), drone_obj)  #Definsico locazione del body frame del drone rispetto l'image plane (esattamente al centro )
            flag_final_frame_analyzed = True
    
        return flag_final_frame_analyzed, line_considered


    ######## Esporto punti linea espressi in bottom iage frame in drone body frame   
    def line_point_exportation_in_drone_body_frame(self, u1,v1,u2,v2, v0, u0, drone_obj, first):
        #x1, y1: coordinate punto P per il quale passa la retta trovata da fitline
        #x2, y2: coordinate punto P1 per il quale passa la retta dato dall'offset dei versori 
        #u0: coordinate del body frame rispetto l'image plane della bottom camera  
        #v0: coordinate del body frame rispetto l'image plane della bottom camera 
        
         #k/f : 0.00146 coefficiente di trasformazione pixel in metri calcolata sperimentalmente per òa bottom camera 
        const = 0.00146
        gain_y= 0.8
        
        
        #Point 1 
        #xb1 = -(const) * ((y1 * original_image_shape[1])/v0 -  original_image_shape[1])* drone_obj.z
        xb1 = -(const) * ((v1  -  v0 )* drone_obj.z)
        #yb1 =  -(const) * (x1 - u0)* drone_obj.z
      
        
        #yb1 =  -(const) * ((x1 * original_image_shape[0])/u0 - original_image_shape[0] )* drone_obj.z
        yb1 =  -1*gain_y * (const) * ((u1  - u0 ) * drone_obj.z)
        #Point2 
        xb2 = -(const) * ((v2 - v0 )* drone_obj.z)
        yb2 =  -1*gain_y * (const) * ((u2 - u0  )* drone_obj.z)
        
        # file3.write(str(xb1) + '\n') 
        # # file3.write('%f\n'%xb1) 
        # file4.write(str(yb1) + '\n') 
        # # file5.write(str(xb2) + "\n") 
        # # file6.write(str(yb2) + "\n") 
        
        # file3.flush()
        # file4.flush()
        
        # print('v0: ', v0)
        # print('u0: ', u0)
        # print('xb1: {0} v1: {1}'.format(xb1, v1))
        # print('yb1: {0} u1: {1}'.format(yb1, u1))
        # print('xb2: {0} v2: {1}'.format(xb2, v2))
        # print('yb2: {0} u2: {1}'.format(yb2, u2))
         
        print('######### PANEL LINE RECOGNIZED, FOLLOWING')
       
        if (first == True):
              # Publish over the air The message with control points 
            publish_navigation_RGB_point(xb1, yb1, xb2, yb2)
      
        else:
            publish_navigation_RGB_point(xb1, yb1, xb2, yb2)

        
    def empty_lists(self):
        self.lefty_array = []
        self.rigthy_array = []
        self.start_point = []
        self.end_point = []
        self.points = []
        self.angular_coefficient = []
        self.actual_points = []
        self.angular_line_versors = []
        self.countours_considered = []
        self.pixels_iniside_area = []
        self.region_pixels_size_coo = []
        self.point_line_distance_actual = []
        self.same_cluster = []
        self.same_cluster_similar_direction = []
        self.sum_list = []
        self.new_list = []
        
                    
