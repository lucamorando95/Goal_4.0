#!/usr/bin/env python
# -*- coding: utf-8 -*-

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import cv2
from shape_detector import ShapeDetector as sd

class Optimizer:
    def __init__(self, image_shape):
        """
        La classe contiene la funzione di costo relativa alla detection dei pannelli.
        Legata allo script RGB_parameters_optimization
        """
        #Initialize parameters HSV range 
        self.h_low = 0
        self.s_low = 0
        self.v_low = 0
        
        self.corr_mask = []
### Thermal Optimization Parameters
        self.th_0 = 125.547  #mean intensity value 
        self.th_1_low = 120  #low intensity range value
        self.th_1_high = 180  #high intensity range value 
        self.th_2 = 9  #distance value 
        self.th_3 = 5000 #minimum area considered 
        self.template = []
        
        self.Optimization_cost_final_array = []
        self.Optimization_parameters_array = []

        self.step = 1
        self.num_white_pixel = 0
        self.panel_lenght = 3
        self.panel_roll = 30
        self.panel_projection_lenght = 0.0
        self.number_of_variables = 0

        self.cluster_orientation = 0.0
        self.lenght_panel_in_pixel = 0.0
        self.pixel_area = 0.0
        self.percentage_pixel_bn_area = 60
        #For RGB
        self.RGB_gaussian_mean_cost_value = 0.56
        self.RGB_std = 0.0
        #For Thermo
        self.Thermo_gaussian_mean_cost_value = 0.02
        self.Thermo_std = 0.2
        
        self.val_max = 0.0
        self.std_square = 0.0
        self.correlation_list = []
        self.corr = 0.0
    
        self.cost = 0.0
        self.final_cost_list = []
        self.image_shape = image_shape

        self.A_reward = []
        self.A = []
        
        self.test = False
        self.correlation = False

        self.counter_s_v_img1 = 0
        self.counter_h_img1 = 0
        #Initialize Gaussian function 
        self.val_max, self.std_square = self.evaluate_gaussian_std(self.percentage_pixel_bn_area)
        pass
    
    def evaluate_gaussian_std(self, best_mean):
        #right limit to the area
        val_max = 2 * best_mean
        step = 1
        s = np.zeros((1,val_max/1))
        for ii in range(0, val_max/1):
            s[0][ii] = np.power(ii - best_mean, 2)
        std = np.sum(s,axis = 1)
        self.std_square = (1/float(val_max)) * float(std[0])
        print("std_square: ", self.std_square)
        print("val_max: ", val_max)
        return val_max, self.std_square

    
    # def gaussian(self, x, mu, sig_square):
    #     return (np.exp(-np.power(x - mu, 2.) / (2 *sig_square)))

    def gaussian(self, x, mu, sig):
        a = x-mu
        a = a/sig
        a = a**2
        a = -0.5*a
        c = np.sqrt(2*np.pi)
        return np.exp(a) /(c*sig) #(np.exp(-np.power(x - mu, 2.) / (2 *sig_square)))

    


    def compute_cost_RGB_function(self, sd, line_versors):
        """
        Calcolo RGB Parameters data una gaussiana.
        Vengono anche considerati i lati del rettangolo definito nella maschera 
        """
        #Picco gaussiana si trova nel valore reale di area
        best_mean = self.RGB_gaussian_mean_cost_value #0.56 #Value where the gaussian has a peak --> correspond to the 70%of the image area occupied by panelk detection

        
        if  not sd.coo_list:
            print("No cluster Detected")
           # print((1 - np.absolute((best_mean - area)/best_mean)))
            self.cost =  0 #*self.gaussian(area, best_mean, self.std_square) #-  0.1*self.final_cost_list[-2]
            
            self.final_cost_list.append(self.cost)
        else:
            cost_vect = np.zeros((1,  len(sd.coo_list)))
            area_array = np.zeros((1,  len(sd.coo_list)))

            for ii in range(0, len(sd.coo_list)):
                vy = line_versors[ii][0]
                if sd.coo_list[ii][0] == 'rectangle':
                    area = sd.area_list[ii][1]
                    area_norm = (float(area)/float(self.image_shape[0]*self.image_shape[1]))
                    #print("area_norm Rectangle: ", area_norm)

                    area_array[0][ii] = area_norm
                    #print("area {}".format(area))
                    #print("rectangle")
                    #Come se sommassi il reward di ciascun rettangolo se sono piu di uno nell'immagine
                    #print("#### Gaussian: {}".format(self.gaussian(area, best_mean, self.std_square))) 
                    x1 =1 - np.absolute(((best_mean - area_norm)/best_mean))
                    # print("X1: ", x1)
                    x2 = self.gaussian(area_norm, best_mean, self.RGB_std)
                    #print("ssd.approx_final_arrayides: ", sd.approx_final_array)
                    sides = sd.approx_final_array[ii]
                  
                    if (sides >= 4 and sides <=8):
                       x3 = -10 + sides
                    else:
                       x3 = -1
                    
                    # print("x2: ", x2)
                    # print("X3: ", x3)
                    cost_vect[0][ii] =  1.5*abs(vy)*x2 -0.5*x3  #+ 10*self.corr # + self.final_cost_list[-1]  +cost_img1  #10*(area/(self.image_shape[0]*self.image_shape[1])))
                    # print("cost_vect: ", cost_vect[0][ii])
                    #print("cost_vect[0]: {}  area/best_mean: {}".format(cost_vect[0], area/best_mean))
                    #Considero differenza da area precednete nella figura. è verosimile avere piu rettangoli con area simile se rilveti correttamente.
                    if (ii > 1):
                        area_prev = sd.area_list[ii-1][1]
                        area_prev_norm = (float(area_prev)/float(self.image_shape[0]*self.image_shape[1]))
                        x5 = 1- abs(area_norm - area_prev_norm)
                        cost_vect[0][ii] =  1.5*abs(vy)*x2*x5 - 0.5*x3 

                
                
                elif sd.coo_list[ii][0] == 'square':
                    area = sd.area_list[ii][1]
                    area_norm = (float(area)/float(self.image_shape[0]*self.image_shape[1]))
                    area_array[0][ii] = area_norm
                    print("area_norm Square: ", area_norm)
                    if (area_norm > 0.7):
                       cost_vect[0][ii] = 0.0
                       continue
                   
                    x1 = 1 - np.absolute(((best_mean - area_norm)/best_mean))
                    x2 = self.gaussian(area_norm, best_mean, self.RGB_std)
                    sides = sd.approx_final_array[ii]
                  
                    if (sides >= 4 and sides <=8):
                       x3 = -10 + sides
                    else:
                       x3 = -1

                    cost_vect[0][ii]  = 0.02* abs(vy)*x2 - 0.5*x3  #+ 10*self.corr #+ self.final_cost_list[-1]
                    if (ii > 1):
                        area_prev = sd.area_list[ii-1][1]
                        area_prev_norm = (float(area_prev)/float(self.image_shape[0]*self.image_shape[1]))
                        x5 = 1- abs(area_norm - area_prev_norm)
                        cost_vect[0][ii] =  0.02*abs(vy)*x2*x5 - 0.5*x3 

                    #print("square")
                
                else:
                    area = sd.area_list[ii][1]
                    area_norm = (float(area)/float(self.image_shape[0]*self.image_shape[1]))
                    area_array[0][ii] = area_norm
                    
                    if (area_norm > 0.7):
                       cost_vect[0][ii] = 0.0
                       continue
                    
                    x1 = 1 - np.absolute(((best_mean - area_norm)/best_mean))
                    x2 = self.gaussian(area_norm, best_mean, self.RGB_std)
                    
                    sides = sd.approx_final_array[ii]

                    if (sides == 4):
                       x3 = -1
                    else:
                       x3 = sides/4

                    cost_vect[0][ii]  = 0.01*abs(vy)*x2 - 0.5*x3  #+ 10*self.corr #+ self.final_cost_list[-1]
                    if (ii > 1):
                        area_prev = sd.area_list[ii-1][1]
                        area_prev_norm = (float(area_prev)/float(self.image_shape[0]*self.image_shape[1]))
                        x5 = 1- abs(area_norm - area_prev_norm)
                        cost_vect[0][ii] =  0.01*abs(vy)*x2*x5 - 0.5*x3 
                   # print("other shape")
            #Lista con costo associata a set di parametri
            
            cost_vect = cost_vect * area_array
            self.cost = (1.0/float(len(sd.area_list)))*np.sum(cost_vect, axis = 1)
            self.cost = self.cost[0]
            # if (self.cost < 0.0):
            #     self.cost = 0.0
            self.final_cost_list.append(self.cost)
            print("COST: ",  self.cost)
        sd.coo_list = []
        sd.approx_final_array = []
        #print("cost: {}".format(self.cost))
        
        return  self.final_cost_list



################## COMPUTE COST FUNCTION FOR THERMAL OPT ####################à
    def compute_cost_function(self, sd, line_versors):
        """
        Compute cost function relative to teh thremal optimization.
        1) EValuate gaussian centered on best mean given by the percentage of the size of the cluster respect the total image .
        Funzione basata su diversi fattori:
        x2: Valore della gaussiana riferita all'area che occupa la shape ripestto al valore centrale (the mask taken is the mask with only one cluster repeated fro the number of clusters)
        x3: Numbero of sides of the shape. The maximum is obtained if the shae has 4 sides
        x4: Value of the correlation final mask masks respect the RGB image (The mask taken is the mask with alk teh cluster)
        x5: Additive cost which increments the reward if the area of each shape detected is similar to the other. Is reasonalble that the panels detected have all the same area.
        """
        #Picco gaussiana si trova nel valore reale di area
        best_mean = self.Thermo_gaussian_mean_cost_value #0.56 #Value where the gaussian has a peak --> correspond to the 70%of the image area occupied by panelk detection

        # if not self.final_cost_list:
        #     self.cost = 0
        #     self.final_cost_list.append(self.cost)
        if  not sd.coo_list:
            print("No cluster Detected")
           # print((1 - np.absolute((best_mean - area)/best_mean)))
            self.cost =  0 #*self.gaussian(area, best_mean, self.std_square) #-  0.1*self.final_cost_list[-2]
            
            self.final_cost_list.append(self.cost)
       
       
        else:
            cost_vect = np.zeros((1,  len(sd.coo_list)))
            area_array = np.zeros((1,  len(sd.coo_list)))

            for ii in range(0, len(sd.coo_list)):
                #COnsidero orientazione della figura, se allineata all'asse y dell'immagine,
                #quindi vy > vx allora il rettangolo individuato è piu probabile essere un filare di pannelli
                vy = line_versors[ii][0]
                if sd.coo_list[ii][0] == 'rectangle':
                    area = sd.area_list[ii][1]
                    area_norm = (float(area)/float(self.image_shape[0]*self.image_shape[1]))
                    area_array[0][ii] = area_norm
                    #print("area {}".format(area_norm))
                    
                    print(best_mean)
                    #Evaluate wit a pdf the area of the rectangle detected respect the size of the picture
                    x2 = self.gaussian(area_norm, best_mean, self.Thermo_std)
                    #Verifico i lati del rettangolo rilevato --> rettangoli sporchi valgono leggermente meno
                    sides = sd.approx_final_array[ii]
                    if (sides >= 4 and sides <=8):
                       x3 = -10 + sides
                    else:
                       x3 = -14

                    #Take Correlation Element
                    if (self.correlation == True):
                        print(self.correlation_list)
                        x4 = self.correlation_list[0]
                        cost_vect[0][ii] =  abs(vy)*x2 - 0.5*x3 + x4
                    else:
                        cost_vect[0][ii] =  abs(vy)*x2 - 0.5*x3
                   
                    
                    #Considero differenza da area precednete nella figura. è verosimile avere piu rettangoli con area simile se rilveti correttamente.
                    if (ii > 1):
                        area_prev = sd.area_list[ii-1][1]
                        area_prev_norm = (float(area_prev)/float(self.image_shape[0]*self.image_shape[1]))
                        x5 = 1- abs(area_norm - area_prev_norm)
                        if (self.correlation == True):
                            cost_vect[0][ii] =  abs(vy)*x2*x5 - 0.5*x3 + x4
                        else:
                            cost_vect[0][ii] =  abs(vy)*x2*x5 - 0.5*x3
                        


                elif sd.coo_list[ii][0] == 'square':
                    area = sd.area_list[ii][1]
                    area_norm = (float(area)/float(self.image_shape[0]*self.image_shape[1]))
                    area_array[0][ii] = area_norm

                    #x1 = 1 - np.absolute(((best_mean - area_norm)/best_mean))
                    x2 = self.gaussian(area_norm, best_mean, self.Thermo_std)
                    cost_vect[0][ii]  =  -0.5*x2 #+ 10*self.corr #+ self.final_cost_list[-1]

                    #print("square")
                
                else:
                    area = sd.area_list[ii][1]
                    area_norm = (float(area)/float(self.image_shape[0]*self.image_shape[1]))
                    area_array[0][ii] = area_norm

                    x2 = self.gaussian(area_norm, best_mean, self.Thermo_std)
                    cost_vect[0][ii]  =  -x2  #+ 10*self.corr #+ self.final_cost_list[-1]
                   # print("other shape")
            #Lista con costo associata a set di parametri
           
            #cost_vect = cost_vect * area_array
            self.cost = np.sum(cost_vect, axis = 1) #(1.0/float(len(sd.area_list)))*n
            self.cost = self.cost[0]
            self.final_cost_list.append(self.cost)
            print("COST: ",  self.cost)
        sd.coo_list = []
        sd.approx_final_array = []
        self.correlation_list = []
        #print("cost: {}".format(self.cost))
    
        return  self.final_cost_list
    
    



    
    def evaluate_real_panels_pixel_area_given_altitude(self, drone_obj):
        """
        Evaluate panel area in pixel given the drone altitude 
        --> guarda note ipad
        """
        k = 0.00146
        v0 = self.image_shape[0]/2
        u0 = self.image_shape[1]/2
       
        v1 = v0 
        u1 = u0 - self.panel_projection_lenght/2 * (1/(drone_obj.z * k))

        v2 = v0
        u2 = u0 + self.panel_projection_lenght/2 * (1/(drone_obj.z * k))
        self.lenght_panel_in_pixel = np.sqrt(np.power(v2 -v1,2) + np.power(u2 -u1,2))

        #Evaluate Real Pixel Area
        b = self.image_shape[0]/np.cos(np.abs(self.cluster_orientation))
        self.pixel_area = b*self.lenght_panel_in_pixel
        
        self.percentage_pixel_bn_area = self.pixel_area/(self.image_shape[0] * self.image_shape[1]) * 100
        #print("pixel_area: {} percentage: {}".format(self.pixel_area, self.percentage_pixel_bn_area))
        
        #Suppose fixed percentage between at 60%
        self.percentage_pixel_bn_area = 60
        best_mean = self.percentage_pixel_bn_area 
        self.val_max, self.std_square = self.evaluate_gaussian_std(best_mean)


    def correlation_coefficient(self, patch1, patch2):
        product = np.mean((patch1 - patch1.mean()) * (patch2 - patch2.mean()))
        stds = patch1.std() * patch2.std()
        if stds == 0:
            return 0
        else:
            product /= stds
            return product

    
    
    def compute_cross_correlation(self, template, img, cluster):
        if (len(cluster) == 0):
           img = np.zeros_like(img)
       
        if len(img.shape) == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if  len(template.shape) == 3:
            template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

        scale_percent = 30 # percent of original size
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
          
        # resize image
        img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        template = cv2.resize(template, dim, interpolation = cv2.INTER_AREA)
         
        
       
      
           # methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR','cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
        methods = ['cv2.TM_CCORR_NORMED']
        for meth in methods:
            method = eval(meth)
            self.corr = cv2.matchTemplate(img,template,method)
           # print ("Method {}  : Result{}") .format(method, self.corr[0][0]) 
            self.correlation_list.append(self.corr[0][0])
    
   
    
    ###################### COST FUNCTION BETWEEN TWO IMAGES
    def compute_cost_function_between_2_images(self, sd2, cost_img1, template, img2, cluster_img2):
        """
        Funzione chimaata da image2_processing().
        Calcola la funzione di costo tra ciasuna img2 e il template genearto (relativo a image2)
        tiene conto anche della crosscoreelazione tra entrambe le immagini 

       
        """
      
        best_mean = 0.50 #self.percentage_pixel_bn_area #0.56 #Value where the gaussian has a peak --> correspond to the 70%of the image area occupied by panelk detection
        
        #Compute Cross correlation Value
        self.compute_cross_correlation(template, img2, cluster_img2)


        if not self.final_cost_list:
            self.cost = 0
            self.final_cost_list.append(self.cost)
        if  not sd2.coo_list:
            print("No cluster Detected")
            area =  np.where(img2 == 255)
            area = len(area)
           # print((1 - np.absolute((best_mean - area)/best_mean)))
            self.cost =  0*self.gaussian(area, best_mean, self.std_square) #-  0.1*self.final_cost_list[-2]
            
            self.final_cost_list.append(self.cost)
        else:
            cost_vect = np.zeros((1,  len(sd2.coo_list)))
            area_array = np.zeros((1,  len(sd2.coo_list)))

            for ii in range(0, len(sd2.coo_list)):
               
                if sd2.coo_list[ii][0] == 'rectangle':
                    area = sd2.area_list[ii][1]
                    area_norm = (float(area)/float(self.image_shape[0]*self.image_shape[1]))
                    area_array[0][ii] = area_norm
                    print("area {}".format(area))
                    #print("rectangle")
                    #Come se sommassi il reward di ciascun rettangolo se sono piu di uno nell'immagine
                    #print("#### Gaussian: {}".format(self.gaussian(area, best_mean, self.std_square))) 
                    x1 =1 - np.absolute(((best_mean - area_norm)/best_mean))
                    x2 = self.gaussian(area_norm, best_mean, self.std_square)
                    cost_vect[0][ii] =  x1*x2#+ 10*self.corr # + self.final_cost_list[-1]  +cost_img1  #10*(area/(self.image_shape[0]*self.image_shape[1])))
                    #print("cost_vect[0]: {}  area/best_mean: {}".format(cost_vect[0], area/best_mean))
                elif sd2.coo_list[ii][0] == 'square':
                    area = sd2.area_list[ii][1]
                    area_norm = (float(area)/float(self.image_shape[0]*self.image_shape[1]))
                    area_array[0][ii] = area_norm

                    x1 = 1 - np.absolute(((best_mean - area_norm)/best_mean))
                    x2 = self.gaussian(area_norm, best_mean, self.std_square)
                    cost_vect[0][ii]  =  x1*x2 #+ 10*self.corr #+ self.final_cost_list[-1]

                    #print("square")
                
                else:
                    area = sd2.area_list[ii][1]
                    area_norm = (float(area)/float(self.image_shape[0]*self.image_shape[1]))
                    area_array[0][ii] = area_norm

                    x1 = 1 - np.absolute(((best_mean - area_norm)/best_mean))
                    x2 = self.gaussian(area_norm, best_mean, self.std_square)
                    cost_vect[0][ii]  =  x1*x2  #+ 10*self.corr #+ self.final_cost_list[-1]
                   # print("other shape")
            #Lista con costo associata a set di parametri
           
            cost_vect = cost_vect * area_array
            self.cost = (1.0/float(len(sd2.area_list)))*np.sum(cost_vect, axis = 1)
            self.cost = self.cost[0]
            self.final_cost_list.append(self.cost)
            print("COST: ",  self.cost)
        sd2.coo_list = []
        #print("cost: {}".format(self.cost))
    
        return  self.final_cost_list
    
    
    def create_cost_tensor(self, Op2, h_low_img2, s_low_img2, v_low_img2,counter_h_img2, counter_s_v_img2, cost, step):
        """
        Create Cost Matrix Baaed on Matrix A and Submatrices B
        """
        #Questa funzione viene chiamata da classe Op, quindi con riferimeto a img 1
     
        
        #Search correspondant matrix B in each element of A_reward 
        #Example: 
        #----> B_cons = self.A[self.h_low][self.counter_s_v_img1][h_low_img2]
        # Permits to obtain matrix B in channel H1, row counter_s_v_img1, colonna h_low_img2 di A
        # B_cons[counter_s_v_img2]---> Permette di entrare nella riga di B corrispondente alla combinazione di s, v img2
        
        
       
        self.A_reward[self.h_low][self.counter_s_v_img1][counter_h_img2][counter_s_v_img2] = cost
        # print("A_reward_element cost : ",  self.A_reward[self.h_low][self.counter_s_v_img1][h_low_img2][counter_s_v_img2])
     
       


    #Funzione chimaata da processing iage 1 --> OP per creare il tensore degli indici
    def create_HSV_index_tensor(self, Op2, h_img1_fixed_flag,  s_img1_fixed_flag,  v_img1_fixed_flag,
                                   h_img2_fixed_flag, s_img2_fixed_flag,  v_img2_fixed_flag, step):
        
        
        if len(self.h_range) == 1:
            h_high_img1 = self.h_range[0]+ 1
        else:
            h_high_img1 = self.h_range[-1]  + 1   #Canali in A 
        
        if len(self.s_range) == 1:
            s_high_img1 = self.s_range[0] + 1 #righe in A --> numero di el in ciascuna colonna di A
        else:
            s_high_img1 = self.s_range[-1]  + 1 
        
        if len(self.v_range) == 1:
            v_high_img1 = self.v_range[0] + 1 #righe in A --> numero di el in ciascuna colonna di A ---> S*V img1
        else: 
            v_high_img1 = self.v_range[-1]  + 1
        
        if len(Op2.h_range) == 1:
            h_high_img2 = Op2.h_range[0] + 1 #Colonne in A --> numero di el in ciascuna riga di A 
        else:
            h_high_img2 = Op2.h_range[-1]  + 1
        
        if len(Op2.s_range) == 1:
            s_high_img2 = Op2.s_range[0] + 1 #Righe in lista B
        else:
            s_high_img2 = Op2.s_range[-1] + 1
        
        if len(Op2.v_range) == 1:
            v_high_img2 = Op2.v_range[0]+ 1 # #Righe in lista B ---> S * V img2
        else:
            v_high_img2 = Op2.v_range[-1] + 1
            
        
        
        print("self.h_range[0]: {}, h_high_img1: {}".format(self.h_range[0],h_high_img1 ))
        print("self.s_range[0]: {}, s_high_img1: {}".format(self.s_range[0],s_high_img1))
        print("self.v_range[0: {}, v_high_img1: {}".format(self.v_range[0],v_high_img1 ))
        print("Op2.h_range[0]: {}, h_high_img2: {}".format(Op2.h_range[0],h_high_img2 ))
        print("Op2.s_range[0]: {}, s_high_img2: {}".format(Op2.s_range[0],s_high_img2 ))
        print("Op2.v_range[0]: {}, v_high_img2: {}".format(Op2.v_range[0],v_high_img2 ))
      
           
        col_B = 0
        print("CREATE INDEXING TENSOR: ")
        # A = [255*255, 255, 255]  --> righe: comb S,V img1 
                                #    --> col: comb H img2
                                #    ---> canali: H img1 
        #A [0,0] ---> sottomatrice B [255*255, 2] --> righe: comb S,V img2
                                                #   --> col: HSV img1, HSV img2 con H fisso
        # A contiene sulle righe tutte le possibili combinazioni di S V per img1 e sulle colonne le possibili combinazioni al variare di H per img 2
        
        
        #A[n_channel][n_row][n_col]
        A_n_channel =    h_high_img1 - self.h_range[0]
        s_img1_n = s_high_img1 - self.s_range[0]
        v_img1_n = v_high_img1 - self.v_range[0]
        A_n_row = s_img1_n * v_img1_n
        # print("s_img1_n: {}, v_img1_n: {}".format(s_img1_n, v_img1_n))
        # print("A_n_row: {}".format(A_n_row))

        h_img2_n = h_high_img2 -Op2.h_range[0]
        A_n_col = h_img2_n
        # print("A_n_col: ", A_n_col)
        
        A = np.empty((A_n_channel,A_n_row, A_n_col), dtype = np.object)
        A_reward = np.empty((A_n_channel,A_n_row, A_n_col), dtype = np.object)

        s_img2_n = s_high_img2 - Op2.s_range[0]
        v_img2_n = v_high_img2 - Op2.v_range[0]
        B_n_row = s_img2_n * v_img2_n
        B_n_col = 2
        print("s_img2_n: {}, v_img2_n: {}".format(s_img2_n, v_img2_n))
        print("B_n_row: {}".format(B_n_row))
        
        counter_h_img1 = 0
        counter_s_img1 = 0
        counter_v_img1 = 0
        
        counter_h_img2 = 0
        counter_s_img2 = 0
        counter_v_img2 = 0
    
        for h_img1 in range(0, A_n_channel, step):
            H_img1 = self.h_range[counter_h_img1]
            for s_v_img1 in range(0, A_n_row, step):
                S_img1 = self.s_range[counter_s_img1]
                V_img1 = self.v_range[counter_v_img1]
             
                for h_img2 in range(0, A_n_col, step):
                    #Generate elements of B matrix
                    H_img2 = Op2.h_range[counter_h_img2]
                    B = np.empty((B_n_row,B_n_col), dtype = np.object)
                    B_rew = np.empty((B_n_row,1), dtype = np.object)
                    for rows_v_img2 in range(0, B_n_row, step):
                       # print("counter_v_img2: ", counter_v_img2)
                        S_img2 = Op2.s_range[counter_s_img2]
                        V_img2 = Op2.v_range[counter_v_img2]
                        if (counter_s_img2 == 0):
                            row = (counter_s_img2 + 1) *counter_v_img2
                        else:
                            row = (counter_s_img2) *counter_v_img2

                        C1 = np.empty((1,3), dtype = np.object)
                        C2 = np.empty((1,3), dtype = np.object)
                        C1[0][0] = H_img1
                        C1[0][1] = S_img1
                        C1[0][2] = V_img1

                        C2[0][0] = H_img2
                        C2[0][1] = S_img2
                        C2[0][2] = V_img2


                        B[row][0] = C1
                        B[row][1] = C2

                        #For Reward Tensor
                        B_rew[row] = 0
                        #Update SV img2 counter 
                        if (counter_v_img2 >= v_img2_n/step):
                            counter_v_img2 = 0
                            if counter_s_img2 >= s_img2_n/step:
                                counter_s_img2 = counter_s_img2
                            else:
                                counter_s_img2 = counter_s_img2 + 1
                        else:  
                            counter_v_img2 = counter_v_img2 + 1
                        print("counter_v_img2: ", counter_v_img2)
                    #print("B: {}".format(B))
                    #Fill the matrix A with the matrix B in each position 
                    if (counter_s_img1 == 0):
                        row_A = (counter_s_img1 + 1) *counter_v_img2
                    else:
                        row_A = (counter_s_img1) *counter_v_img2
                    A[counter_h_img1][row_A][counter_h_img2] = B
                    A_reward[counter_h_img1][row_A][counter_h_img2] = B_rew
                    #print("A: ", A)
                    counter_h_img2 = counter_h_img2 + 1
                    print("counter_h_img2: ", counter_h_img2)
                if counter_v_img1 >= v_img1_n/step - 1:
                    counter_v_img1 = 0
                    counter_s_img1 = counter_s_img1 + 1
                counter_v_img1 = counter_v_img1 + 1
            counter_h_img1 = counter_h_img1 + 1 
        self.A = A
        self.A_reward = A_reward
      




   



    def load_template(self, name):
       
        template = cv2.imread(name)
        template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        return template

    def plot_cost_3D(self, x1, x2):
        """ 
        x1, x2 : array asse x e y
        """
        
        
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        x1, x2 = np.meshgrid(x1, x2)
        surf = ax.plot_surface(x1, x2, self.final_cost_list, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
        ax.set_xlabel('$s_{low}$', fontsize=20, rotation=150)
        ax.set_ylabel('$s_{high}$', fontsize=20, rotation=150)
        ax.set_zlabel('$reward$')
        plt.show()

    def write_txt_file(self, file, Op2):
        """
        Write txt file of 3D shape to import in matlab
        """
        # print("h_low: ", h_low)
        # print("s_low: ", s_low)
        # print("v_low: ", v_low)
        # print("self.final_cost_list: ", Op2.final_cost_list)
        # print("self.correlation_list[ii]: ", Op2.correlation_list)


        print("Writing TXT file")
        for ii in range(0, len(h_low)):
            file.write(str(h_low[ii]) + "," + str(s_low[ii])+ "," +  str(v_low[ii]) + "," + str(Op2.final_cost_list[ii]) + "," + str(Op2.correlation_list[ii]) + "," + str(step) + "\n") 
            file.flush()
        file.close()
