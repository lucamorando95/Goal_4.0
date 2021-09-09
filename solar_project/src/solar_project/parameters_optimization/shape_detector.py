import cv2
import numpy as np
class ShapeDetector:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.w = 0.0
        self.h = 0.0
        self.ratio =0.0
      
        self.coo_list = []
        self.area_list =[]
        self.approx_final_array = []
        #self.file2 = open("/home/lucamora/Desktop/genetic_algorithm/len_approx/epsilon_005.txt","w") 

        pass
    
    def detect(self, c, epsilon, line_versors_for_opt):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        
        approx = cv2.approxPolyDP(c, epsilon * peri, True)
        # print(len(approx))
        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"
            (self.x, self.y, self.w, self.h) = cv2.boundingRect(approx)
            self.ratio =  self.w / float(self.h)
            if (self.w >= float(self.h)):
                vy = 0.1
            else:
                vy = 0.9
            line_versors_for_opt.append([vy])
            self.approx_final_array.append(len(approx))
        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif (len(approx) >= 4 and len(approx) <= 8):
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratiod
            (self.x, self.y, self.w, self.h) = cv2.boundingRect(approx)
            self.ratio = self.w / float(self.h)
            #print("self.ratio : ", self.ratio)
          
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            if (self.w >= self.h):
                vy = 0.1
            else:
                vy = 0.9
            line_versors_for_opt.append([vy])
            self.approx_final_array.append(len(approx))
            shape = "square" if self.ratio >= 0.95 and self.ratio <= 1.50 else "rectangle"
            
           
        else:
            shape = "circle"
            (self.x, self.y, self.w, self.h) = cv2.boundingRect(approx)
            self.ratio = self.w / float(self.h)
            if (self.w >= float(self.h)):
                vy = 0.1
            else:
                vy = 0.9
            line_versors_for_opt.append([vy])
            self.approx_final_array.append(len(approx))
        # return the name of the shape
       
       
        # self.file2.write(str(len(approx)) + "," + str(epsilon)+ "\n") 
        # self.file2.flush()
       
            
        return shape, self.x, self.y, self.w, self.h, approx, line_versors_for_opt

    