#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <iostream>
#include <chrono>
#include <ctime>  
#include <string>
#include <sys/stat.h> 

#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/Drone.h"

#include <chrono>
using namespace std::chrono;
using namespace std;
 



// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "Overlapped_Photos");
  ros::NodeHandle nh;
  
  Drone drone;
  
 

  


  //Required to make the sdk able to control the drone 
  bool obtain_control_result = drone.obtain_control();
  

  //Field of View 13mm XT2
  float gamma_1 = 45; //degree
  float gamma_2 = 37; //degree 
  
  float Overlap = 80;
  string folder_name;
  
  bool Mission_enabled = false;
  nh.getParam("/camera_mission_param/XT2_FOV_angle_1",gamma_1);
  nh.getParam("/camera_mission_param/XT2_FOV_angle_2",gamma_2);
  nh.getParam("/camera_mission_param/Overlap",Overlap);
  nh.getParam("/camera_mission_param/Mission_enabled",Mission_enabled);
  nh.getParam("/camera_mission_param/folder_name", folder_name);
  //Create folder where save files 

  string stringpath = "/home/dji/DATA/dji_ws/simulation_data/Media/Predosa_23_07/" + folder_name + "/";
  
  int status = mkdir(stringpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (status != 0)
  {
     cout << "[CAMERA PHOTO] IMPOSSIBLE TO CREATE A NEW FOLDER " << endl;
  }
  
//Finalize the creation of the file txt
  std::ofstream outFile1(stringpath + "data.txt");
  
  //Triangle Base angle
  float alfa_1 = (180 - gamma_1)/2;
  float alfa_1_rad = alfa_1 * M_PI/180;
  float alfa_2 = (180 - gamma_2)/2;
  float alfa_2_rad = alfa_2 * M_PI/180;

  float gamma_1_rad = gamma_1 * M_PI/180;
  float gamma_2_rad = gamma_2 * M_PI/180;
 
  float h = 0.0;
  float time = 1000.0;
  auto start = high_resolution_clock::now();
  auto end = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  int counter = 0;
  char buffer[56]; //Requiired to change the directory
  bool check_time = true;
  bool flag_update_array_counter = true;

  bool time_init = true; 
  int photo_counter = 0;
  int array_counter = 0;
  ros::Rate r(20);
 
 
  while (nh.ok())
  {
   

    
    //Obtain drone altitude 
    h = 15; //drone.current_local_pos.z;

    //Obtain length of the big side of ground rectangle 
    float a1 = (sin(gamma_1_rad/2)/sin(alfa_1_rad)) * h;
    a1 = 2*a1;
    //Obtain length of the minor side of ground rectangle
    float a2 = (sin(gamma_2_rad/2)/sin(alfa_2_rad)) * h; 
    a2 = 2*a2;

    float vel_x =  drone.desired_velocity; //2.0; //drone.current_local_velocity.vector.x;
   
    if  (drone.desired_velocity == 0) 
    {
         vel_x = 2.0;
    }
    float vel_y = drone.current_local_velocity.vector.y; 

    float vel = sqrt(pow(vel_x,2) + pow(vel_y,2));
    
   

    //Evaluate time distance between two photo given the overlap amd the drone velocity 
    float distance_percentage = 100 - Overlap;
    float distance = (a2 * distance_percentage)/100;
    
    
    
    //Take Photos at each time istant t
     //check the array number where the dropne starts the navigation 
    array_counter = drone.array_number;
    
    if (Mission_enabled == true && drone.Navigation_flag == true)
    {
        if (vel > 0.2 && time_init == true)
        {
           
           time = distance/vel;
           start = high_resolution_clock::now();
           time_init = false;
           //Shoot a firs phot only at the start of the opt 
         if (counter == 0)
         {
            bool shoot_photo = drone.camera_take_photo();

            outFile1 << array_counter << ", " << photo_counter << ", "<< std::setprecision(9) << drone.current_gps.latitude <<", " <<  std::setprecision(9) << drone.current_gps.longitude << "," <<currentDateTime() << '\n';// std::ctime(&end_time) << '\n';
            photo_counter = photo_counter + 1; 

            counter = 1;
         }

        }
        
        
      
       end =  high_resolution_clock::now();
       duration = duration_cast<microseconds>(end - start)/1000000;
       
       if (duration.count() > time && vel > 0.2)
       {
            bool shoot_photo = drone.camera_take_photo();
            cout << "[CAMERA PHOTO]  shoot_photo: " << shoot_photo << endl;
            cout << "[CAMERA PHOTO]  Next photo will be taken at " << time << " seconds from now." << endl;

         
            //Take current time and date 
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);

            outFile1 << array_counter << ", " << photo_counter << ", "<< std::setprecision(9) << drone.current_gps.latitude <<", " <<  std::setprecision(9) << drone.current_gps.longitude << "," <<currentDateTime() << '\n';// std::ctime(&end_time) << '\n';
          
            check_time = true;
            time_init = true;
            
            photo_counter = photo_counter + 1; 

            if (photo_counter == 1)
            {
               flag_update_array_counter = true;
            }
            
       }

       //Check if time is too High 
       if (vel < 0.2)
       {
          time_init = true; 
       }
       
    }
    else
    {
       cout << "[CAMERA PHOTO]  Wait to enter in Navigation Mode" << endl;
       photo_counter = 0;
       counter = 0;
    }
    
    
    if (photo_counter == 1 && flag_update_array_counter == true)
    {
      //  if (array_counter >= 0)
      //  {
      //   // The filename buffer.
      //    snprintf(buffer, sizeof(char) * 32, "/home/dji/dji_ws/simulation_data/Media/Exp_1/data_%i.txt", 44);
      //    cout << "buffer: " << buffer << endl;
      //    std::ofstream outFile2(buffer);
      //    outFile2 << array_counter << ", " << photo_counter << ", "<< std::setprecision(9) << drone.current_gps.latitude <<", " <<  std::setprecision(9) << drone.current_gps.longitude << "\n";
      //  }
      
      
      //cout<<"Array Number: " << array_counter << endl;
      flag_update_array_counter = false;
    }


    
     drone.flagArraynumber = false;

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
