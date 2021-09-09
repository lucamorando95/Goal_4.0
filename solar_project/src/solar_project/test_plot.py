import matplotlib.pyplot as plt
import numpy as np


def draw_plot():
    path = '/home/dji/DATA/dji_ws/simulation_data/test_Predosa/TEST_2/Exp_14/'
    text_file = open(path + 'des_y_vel.txt' , "r")
    des_y_vel = text_file.readlines()
    
    text_file1 = open(path + 'y_vel.txt' , "r")
    y_vel = text_file1.readlines()
    
    text_file2 = open(path + 'des_x_vel.txt' , "r")
    des_x_vel = text_file2.readlines()
    
    text_file3 = open(path + 'x_vel.txt' , "r")
    x_vel = text_file3.readlines()
    
    text_file4 = open(path + 'z_vel_des.txt' , "r")
    des_z_vel = text_file4.readlines()
    
    text_file5 = open(path + 'vel_z.txt' , "r")
    z_vel = text_file5.readlines()
   
    text_file6 = open(path + 'c_obs_RGB_GF.txt' , "r")
    a_obs_RGB_GF = text_file6.readlines()
     
    text_file7 = open(path + 'c_obs_thermo_GF.txt' , "r")
    a_obs_THERMO_GF = text_file7.readlines()

    text_file8 = open(path + 'control_x_coo.txt' , "r")
    x_coo_body = text_file8.readlines()
     
    text_file9 = open(path + 'control_y_coo.txt' , "r")
    y_coo_body = text_file9.readlines()

    text_file10 = open(path + 'error_from_vision_line.txt' , "r")
    line_error = text_file10.readlines()
     
    
    text_file11 = open(path + 'z_pos.txt' , "r")
    z_pos = text_file11.readlines()

    


    text_file1.close()
    text_file2.close()
    text_file3.close()
    text_file4.close()
    text_file5.close()
    text_file6.close()
    text_file7.close()
    text_file8.close()
    text_file9.close()
    text_file10.close()
    text_file11.close()

    #Make plot
    #Y vel plot
    fig = plt.figure()
  
    plt.plot(des_y_vel, 'r')
    plt.plot(y_vel, 'b')
    plt.xlabel('sample')
    plt.ylabel('m/s')
    plt.title('Vel Y')
    plt.show()
    
  
    # #X vel plot 
    # fig = plt.figure()
  
    # plt.plot(des_x_vel, 'r')
    # plt.plot(x_vel, 'b')
    # plt.xlabel('sample')
    # plt.ylabel('m/s')
    # plt.title('Vel X')
    # plt.show()

    # #Z vel plot
    fig = plt.figure()
  
    plt.plot(des_z_vel, 'r')
    plt.plot(z_vel, 'b')
    plt.xlabel('sample')
    plt.ylabel('m/s')
    plt.title('Vel Z')
    plt.show()
    
    #Z position
    fig = plt.figure()
  
    plt.plot(z_pos, 'r')
    plt.xlabel('sample')
    plt.ylabel('m/s')
    plt.title('Position on Z')
    plt.show()


    # fig = plt.figure()
  
    
    # plt.plot(a_obs_RGB_GF, 'b')
    # plt.xlabel('sample')
    # plt.ylabel('m/s')
    # plt.title('c obs RGB GF ')
    # plt.show()

    # fig = plt.figure()
  
   
    # plt.plot(a_obs_THERMO_GF, 'b')
    # plt.xlabel('sample')
    # plt.ylabel('m/s')
    # plt.title('c obs THERMO GF ')
    # plt.show()
    


    # fig = plt.figure()
  
   
    # plt.plot(x_coo_body, 'b')
    # plt.xlabel('sample')
    # plt.ylabel('m/s')
    # plt.title('x vel BF ')
    # plt.show()


   
    fig = plt.figure()
    plt.plot(y_coo_body, 'b')
    plt.xlabel('sample')
    plt.ylabel('m/s')
    plt.title('y vel BF')
    plt.show()
    

    line_error1 = np.empty([len(line_error)])
    for ii in range(0, len(line_error)):
        line_error1[ii] = -1*float(line_error[ii])
        



    fig = plt.figure()
  
   
    plt.plot(line_error1, 'b')
    plt.xlabel('sample')
    plt.ylabel('m')
    plt.title('line error BF')
    plt.show()


 








if __name__ == '__main__':
    draw_plot()

