/** @file soalr_fligth_control.cpp
 *  @version 1.0
 *  @date January, 2021
 *
 *  @brief
 *  Control drone using thermo and RGB camera over solar arrays
 *
 *  @copyright 2021, Laboratorium. All rights reserved.
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Joy.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <fstream>
#include<iostream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>

#include "/home/dji/dji_ws/src/Onboard-SDK-ROS/src/solar_project/include/solar_project/mission.h"
#include "/home/dji/dji_ws/src/Onboard-SDK-ROS/src/solar_project/include/solar_project/image_converter.h"

#include "/home/dji/dji_ws/src/Onboard-SDK-ROS/src/solar_project/include/solar_project/KF.h"
#include "/home/dji/dji_ws/src/Onboard-SDK-ROS/src/solar_project/include/solar_project/pid.h"
#include "/home/dji/dji_ws/src/Onboard-SDK-ROS/src/solar_project/include/solar_project/Drone.h"
#include <dji_sdk/dji_sdk_node.h>

using namespace dji_osdk_ros;
using namespace std;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

int precision = std::numeric_limits<double>::max_digits10;


Mission mission;


dji_osdk_ros::SetupCameraStream setupCameraStream_;

//WRITE TXT FILE FOR DEBUG
std::ofstream outFile1("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/x_vel.txt");
std::ofstream outFile2("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/y_vel.txt");
std::ofstream outFile3("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/des_x_vel.txt");
std::ofstream outFile4("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/des_y_vel.txt");
std::ofstream outFile5("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/control_x_coo.txt");
std::ofstream outFile6("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/control_y_coo.txt");
std::ofstream outFile7("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/a_est_GF.txt");
std::ofstream outFile8("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/c_est_GF.txt");
std::ofstream outFile9("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/a_obs_BF.txt");
std::ofstream outFile10("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/c_obs_BF.txt");
std::ofstream outFile11("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/a_obs_est_BF.txt");
std::ofstream outFile12("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/c_obs_est_BF.txt");
std::ofstream outFile13("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/x_pos.txt");
std::ofstream outFile14("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/y_pos.txt");
std::ofstream outFile15("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/X_target_setpoint_BF.txt"); 
std::ofstream outFile16("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/Y_target_setpoint_BF.txt"); 
std::ofstream outFile17("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/X_target_setpoint_GF.txt"); 
std::ofstream outFile18("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/Y_target_setpoint_GF.txt"); 
std::ofstream outFile19("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/a_obs_thermo_GF.txt"); 
std::ofstream outFile20("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/c_obs_thermo_GF.txt"); 
std::ofstream outFile21("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/a_obs_RGB_GF.txt"); 
std::ofstream outFile22("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/c_obs_RGB_GF.txt");
std::ofstream outFile23("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/vel_z.txt");
std::ofstream outFile24("/home/dji/dji_ws/simulation_data/sim_data_complete/Gazebo_link_only_camera/External_exp/Exp_2/error_from_GPS_line.txt"); 

struct Waypoint_GPS {
   
    //Coordinata punto 1 (riferimento) del GPS. Da inizializzare in main con coordinata P1_x P1_y del primo pannello
    double GPS1_x = 0.0;
    double GPS1_y = 0.0;

    
    vector<double> GPS_waypoints_lat;
    vector<double> GPS_waypoints_lon;
    vector<double> waypoints_x_coo_world_frame;
    vector<double> waypoints_y_coo_world_frame;
    vector<double> waypoints_x_coo_gps_frame;
    vector<double> waypoints_y_coo_gps_frame;

    double GPS_P1_x = 0.0;
    double GPS_P1_y = 0.0;
    double GPS_P2_x = 0.0;
    double GPS_P2_y = 0.0;

    //Errore between drone and GPS LINE expressed in body frame
    double error_from_GPS_line = 0.0;
    //Vector to initialize the Kalamn filter state with the equation of the line passing through the GPS
    //Waypoints 
    vector<double> GPS_P1_waypoint; //Start P1 Waypoint 
    vector<double> GPS_P2_waypoint; //End P2 waypoint

    int counter = 0.0;
} waypoints;

//Flag to switch control from image to Gps
bool from_image = false;
bool from_image_Thermo = false;
bool from_image_RGB = false;


void load_GPS_coordinates_with_GAZEBO_OFFSET(float lat_0, float lon_0)
{
   float delta_x = 2;
   float delta_y = 0;
   float lat_1 = delta_x/C_EARTH + (lat_0 *deg2rad);
   lat_1 = lat_1*rad2deg;
   cout<< "lat1: " << lat_1 << endl;

  // waypoints.GPS_waypoints_lat.push_back(lat_1);
  // waypoints.GPS_waypoints_lon.push_back(lon_0);
  
  delta_y = 22;
  float lon_2 = (delta_y/C_EARTH) * 1/cos(lat_0*deg2rad) + lon_0*deg2rad;
  lon_2 = lon_2 *rad2deg;
  cout <<  "lon2: " << lon_2 << endl;

  // waypoints.GPS_waypoints_lat.push_back(lat_1);
  // waypoints.GPS_waypoints_lon.push_back(lon_2);

  std::array<double, 2> lat{44.611515,44.611515};// 45.557738, 9.125081, 45.558037, 9.124881
  std::array<double, 2> lon{ 8.859281,  8.859559059143066};

  for (int i = 0; i < lat.size(); i++)
{
   waypoints.GPS_waypoints_lat.push_back(lat[i]);
   waypoints.GPS_waypoints_lon.push_back(lon[i]);
}
  

  for (int i = 0; i < waypoints.GPS_waypoints_lat.size(); i++)
{
   
     cout<< std::setprecision(precision)  << "  waypoints.GPS_waypoints_lat: " <<  waypoints.GPS_waypoints_lat[i]<< endl;
     cout<< std::setprecision(precision)  << "   waypoints.GPS_waypoints_lon: " <<   waypoints.GPS_waypoints_lon[i]<< endl;
}
   
}







void load_GPS_coordinates()
{
  vector<double> ReplayBuffer;
  std::ifstream infile;          //creates stream myFile

  /*
  infile.open("GPS_coo.txt");  //opens .txt file

  if (!infile.is_open())  // check file is open, quit if not
  {
      std::cerr << "failed to open file\n";
      return; 
  }
   
  
  
  for (std::string f; getline(infile, f,','); ReplayBuffer.push_back(std::stod(f)))
    
   
  
  for (long double f : ReplayBuffer)
    std::cout << f << " , ";
   
  
  if (!infile.eof())
    {
      cerr << "Fooey!\n";
    }
  
  int length = ReplayBuffer.size();
  int count = 0;
  
  for (int i = 0; i < length; i++)
  {
      cout<<"ReplayBuffer: " << ReplayBuffer[i]<< endl;
      if (count == 0)
      {
          //waypoints.GPS_waypoints_lat.push_back(ReplayBuffer[i]);  
          count = 1;
      }
      else
      {
          //waypoints.GPS_waypoints_lon.push_back(ReplayBuffer[i]);  
           count = 0;
      }
      
      //Per il momento 

  }
*/
/* 
BOLLATE 
{45.558028, 45.558038, 45.557977, 45.557971}
{9.125616,9.124879, 9.124875, 9.125737}

VOLTAGGIO
{44.611031, 44.611361, 44.611352}
{ 8.860616,  8.859245,  8.859058,}

VOLTAGGIO 2 
{44.611031,44.611230, 44.611106,44.610929}
{ 8.860616,  8.859839, 8.859753,  8.860470}
*/

//44.611031
// 8.860616


std::array<double, 4> lat{44.611174,44.611230, 44.611106,44.611064};// 45.557738, 9.125081, 45.558037, 9.124881
std::array<double, 4> lon{ 8.860654,  8.859839, 8.859753, 8.860611};

//CASO LIMITE __> coo:45.558036, 9.124888, 45.557800, 9.124901
// std::array<double, 2> lat{45.558028, 45.558364};
// std::array<double, 2> lon{9.125616,9.125379};
cout<<" lat.size(): " <<  lat.size()<< endl;
for (int i = 0; i < lat.size(); i++)
 {
   waypoints.GPS_waypoints_lat.push_back(lat[i]);
   waypoints.GPS_waypoints_lon.push_back(lon[i]);
 }
      
    
for (int i = 0; i < waypoints.GPS_waypoints_lat.size(); i++)
 {
   
     cout<< std::setprecision(precision)  << "  waypoints.GPS_waypoints_lat: " <<  waypoints.GPS_waypoints_lat[i]<< endl;
     cout<< std::setprecision(precision)  << "   waypoints.GPS_waypoints_lon: " <<   waypoints.GPS_waypoints_lon[i]<< endl;
 }
}

//################### Evaluate differences between target coordinate and drone coordinates ###########
void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
 
  // ROS_INFO("##### target.longitude %f ....", target.longitude);

  // ROS_INFO("##### origin.latitude %f ....", origin.latitude);
  // ROS_INFO("##### origin.longitude %f ....", origin.longitude);

	//Calcolo offset tra posizone iniziale e attuale drone 
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  //Ottengo Offset in metri dato offset in latitudine e longitudine 
  deltaNed.x = deltaLat * deg2rad * C_EARTH;
  deltaNed.y = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude );
  deltaNed.z = target.altitude - origin.altitude;  //--> Target altitude è l'altuitudine attuale del drone 
  //ROS_INFO("##### target.altitude:  %f , origin.altitude : %f....",  target.altitude,  origin.altitude);
  
  // cout<<"[localOffsetFromGpsOffset]  deltaNed.x: " <<  deltaNed.x<< endl;
  // cout<<"[localOffsetFromGpsOffset]  deltaNed.y:" <<  deltaNed.y << endl;
}



//############### Publish Position /Velocity Command to drone #############
void publish_command_position_to_drone(Drone *drone)
{

    //Saturation 
  //   mission.speedFactor         = 1;
  //   if (abs(mission.z_offset_remaining) >=   mission.speedFactor)
  //   drone->z_pos_cmd = (mission.z_offset_remaining>0) ?   mission.speedFactor : -1 *   mission.speedFactor;
  //   else
  //  //Il comando sulla x è relativo all'offset remaining
  //   drone->z_pos_cmd  = mission.start_local_position.z + mission.target_offset_z;
    if (mission.cartesian_distance_2D  < 2.0 && mission.state > 1 ||  mission.target_reached  == true)
    {
      //StartBreaking 
      drone->x_vel_cmd = 0.0;
      drone->y_vel_cmd = 0.0;
      drone->z_vel_cmd = 0.0;
      drone->yaw_rate_cmd = 0.0;
      mission.target_reached = true;
      mission.breaking_counter = mission.breaking_counter + 1;
    }
    else
    {
    
       sensor_msgs::Joy controlPosYaw;
      controlPosYaw.axes.push_back(drone->x_pos_cmd);
      controlPosYaw.axes.push_back(drone->y_pos_cmd);
      controlPosYaw.axes.push_back(drone->z_pos_cmd);
      controlPosYaw.axes.push_back(drone->yaw_des_rad);
      drone->publish_cmd_position(controlPosYaw);
  
    }
    
    
}

void publish_command_velocity_to_drone(Drone *drone)
{

   if (mission.cartesian_distance_2D  < 2.0 && mission.state > 1 ||  mission.target_reached  == true)
   {
     drone->x_vel_cmd = 0.0;
     drone->y_vel_cmd = 0.0;
     mission.target_reached = true;
     mission.breaking_counter = mission.breaking_counter + 1;
     cout << "SONO QUI : " << mission.cartesian_distance_2D << endl;
   }

   sensor_msgs::Joy ctrlVelYaw;
    
    uint8_t flag = (Control::VERTICAL_VELOCITY   |
                Control::HORIZONTAL_VELOCITY |
                Control::YAW_RATE          |    //YAW_RATE o YAW_ANGLE
                Control::HORIZONTAL_BODY   |     // HORIZONTAL_BODY rispetto frame locale 
											     // HORIZONTAL_GROUND rispetto al body frasme 
                Control::STABLE_ENABLE);
    ctrlVelYaw.axes.push_back(drone->x_vel_cmd); //drone->x_vel_cmd
    ctrlVelYaw.axes.push_back(drone->y_vel_cmd);
    ctrlVelYaw.axes.push_back(drone->z_vel_cmd);
    ctrlVelYaw.axes.push_back(drone->yaw_rate_cmd); //drone->yaw_des_rad
    ctrlVelYaw.axes.push_back(flag);
   drone->publish_cmd_velocity(ctrlVelYaw);
 
  // cout <<"drone->x_vel_cmd : " <<drone->x_vel_cmd  << endl;
  // cout <<"drone->y_vel_cmd : " <<drone->y_vel_cmd  << endl;
  // cout <<"drone->z_vel_cmd : " <<drone->z_vel_cmd  << endl;
 // cout <<"drone->yaw_des_rad: " <<drone->yaw_des_rad  << endl;
}

void  evaluate_local_offset_between_GPS_target_and_body_frame(double target_lat, double target_lon, sensor_msgs::NavSatFix& drone_curr_GPS,   
                        geometry_msgs::Point drone_curr_local_pos, geometry_msgs::Vector3&  target_offset_respect_drone_position)
{
  /*
  Calcolo Offset tra Coo GPS Target e coo GPS drone.
  L'offset viene calacolato su X e Y con frame orienatto con asse X verso NORD e Y verso EST.
  Lo YAW è inteso essere di 90 gradi.
  --> Necessaria sucessiva ROtazione 
   
   */
  
  double deltaLon = target_lon - drone_curr_GPS.longitude;
  double deltaLat = target_lat - drone_curr_GPS.latitude;
  
  //Ottengo offset in metri del target da raggiungere 
  double delta_x = deltaLat * deg2rad * C_EARTH;
  double delta_y = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target_lat );
  double delta_z = mission.target_offset_z - drone_curr_local_pos.z;
  
  //NB--> Verificare sempre posizionamento frame GPS
  target_offset_respect_drone_position.x = delta_x; //drone_curr_local_pos.x + delta_x;
  target_offset_respect_drone_position.y = delta_y; //drone_curr_local_pos.y + delta_y;
  target_offset_respect_drone_position.z = delta_z; //drone_curr_local_pos.z + delta_z;
  // cout<< "target_offset_respect_drone_position.x: " <<  target_offset_respect_drone_position.x << endl;
  // cout<< "target_offset_respect_drone_position.y: " << target_offset_respect_drone_position.y << endl;
  // cout<< "target_offset_respect_drone_position.z: " <<  target_offset_respect_drone_position.z << endl;
   
}

void  evaluate_local_offset_between_GPS_target_and_local_frame(double target_lat, double target_lon, sensor_msgs::NavSatFix& drone_curr_GPS,   
                        geometry_msgs::Point drone_curr_local_pos,
                       geometry_msgs::Vector3& target_offset_respect_local_position)
{ 
  /*
 Calcolo Offset tra Coo GPS Target e coo GPS frame Locale.
 Il frame locale èun frame fittizio posizionato a piacere.
 NON COINCIDE CON IL FRAME DI HOME, IL QUALE é IL FRAME NEL PUNTO DI DECOLLO.
  L'offset viene calacolato su X e Y con frame orienatto con asse X verso NORD e Y verso EST.
  Lo YAW è inteso essere di 90 gradi.
  --> Necessaria sucessiva ROtazione 
  --> NOTAZIONE NED 
  */

  double deltaLon = target_lon - mission.start_gps_location.longitude;
  double deltaLat = target_lat -mission.start_gps_location.latitude;
  
  double delta_x = deltaLat * deg2rad * C_EARTH;
  double delta_y = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target_lat );
  double delta_z = mission.target_offset_z - 0;
  
  //NB --> Verifica se il frame GPS è allineato sempre con asse x verso NORD
    
  target_offset_respect_local_position.y = delta_x;
  target_offset_respect_local_position.x = delta_y;
  target_offset_respect_local_position.z = delta_z;

  // cout<< " target_offset_respect_local_position.x: " <<  target_offset_respect_local_position.x << endl;
  // cout<< "target_offset_respect_local_position.y: " << target_offset_respect_local_position.y << endl;
  // cout<< " target_offset_respect_local_position.z: " <<  target_offset_respect_local_position.z << endl;
   
}


void  evaluate_local_offset_between_GPS_target_and_HOME_frame(double target_lat, double target_lon,
                                                                 geometry_msgs::Vector3& target_offset_respect_HOME_position)
{
  double deltaLon = target_lon - mission.HOME_gps_location.longitude;
  double deltaLat = target_lat -mission.HOME_gps_location.latitude;
  
  double delta_x = deltaLat * deg2rad * C_EARTH;
  double delta_y = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target_lat );
  double delta_z = mission.target_offset_z - 0;
  
  target_offset_respect_HOME_position.x = delta_x;
  target_offset_respect_HOME_position.y = delta_y;
  target_offset_respect_HOME_position.z = delta_z;

  // cout<< " target_offset_respect_HOME_position.x: " <<  target_offset_respect_HOME_position.x << endl;
  // cout<< "target_offset_respect_HOME_position.y: " << target_offset_respect_HOME_position.y << endl;
  // cout<< " target_offset_respect_HOME_position.z: " <<  target_offset_respect_HOME_position.z << endl;
}


void evaluate_des_GPS_WAYPOINT_yaw_angle(Drone *drone)
{
  /*Evaluate Yaw Desired Angle to align drone with GPS waypoint 
  NB: Se waypoints_counter = 0 si intende il waypoint di start sul primo pannello
  */

  double start_lat =  waypoints.GPS_waypoints_lat[ waypoints.counter];
  double start_lon =  waypoints.GPS_waypoints_lon[ waypoints.counter];
  geometry_msgs::Vector3 target_offset_respect_drone_position_referred_to_local_position;

  evaluate_local_offset_between_GPS_target_and_body_frame(start_lat, start_lon, drone->current_gps, drone->current_local_pos, target_offset_respect_drone_position_referred_to_local_position);
  
  drone->yaw_des_rad = atan2(target_offset_respect_drone_position_referred_to_local_position.x, target_offset_respect_drone_position_referred_to_local_position.y);
  //float new_yaw_des_rad = drone->yaw_des_rad;
  
  if(drone->yaw_des_rad <= +C_PI && drone->yaw_des_rad > -C_PI/2)
  {
      drone->yaw_des_rad_sim = -drone->yaw_des_rad + C_PI/2;
  }
  else
  {
      drone->yaw_des_rad_sim  = -C_PI - drone->yaw_des_rad -C_PI/2;
  }
   //drone->yaw_des_rad = new_yaw_des_rad;


}


float change_yaw_for_translation_between_locals_frame(Drone *drone)
{
  float yaw_for_rot = 0.0;
  if (drone->yaw_in_Rad_sim > -C_PI && drone->yaw_in_Rad_sim <= C_PI/2)
  {
      yaw_for_rot = drone->yaw_in_Rad_sim  + C_PI/2;
  }
  else
  {
       yaw_for_rot = -C_PI + (drone->yaw_in_Rad_sim  - C_PI/2);
  }
  return yaw_for_rot;
}





void KF_estimation_exportation_in_BF_for_control_point_generation(Drone *drone, KF *Kalman_Filter)
{
    //Find two points on the line defined by the two paramter estimated
      float x1_w = 0.0;
      float y1_w = 0.0;
      float x2_w = 0.2;
      float y2_w = 0.0;
      
      float x1_b = 0.0;
      float y1_b = 0.0;
      float x2_b = 0.0;
      float y2_b = 0.0;

      float a_b = 0.0;
      float c_b = 0.0;
      
      float yaw_rot = 0.0;
     //Use the RGB KF estimation
      y1_w =  drone->xh_[0]*x1_w + drone->xh_[1];
      y2_w =  drone->xh_[0]*x2_w + drone->xh_[1];
      //cout<<"[KF EXP FUNC] KF RGB estimation exported for control Point Evaluation" << endl;
      //cout<<"[KF EXP FUNC] y1_w: " <<y1_w << ", "<< "y2_w: " << y2_w << endl;
      

      //ROtate both points in body frame 
      geometry_msgs::Point curr_local_pos;
  
      curr_local_pos.x  = drone-> new_current_local_pos.x;
      curr_local_pos.y =  drone-> new_current_local_pos.y;
      
      // cout << "[KF ESTIMATION EXPORTATION] X d: " <<  curr_local_pos.x << " Y d: " <<  curr_local_pos.y << endl;
      yaw_rot = change_yaw_for_translation_between_locals_frame(drone);
      drone->Rotation_local_GF_to_BF(curr_local_pos, x1_w, y1_w, drone->yaw_in_Rad);
      x1_b = drone->check_x_b;
      y1_b = drone->check_y_b;
      // cout << "[KF ESTIMATION EXPORTATION] x1_b: " << x1_b << " y1_b: " << y1_b << endl;

      drone->Rotation_local_GF_to_BF(curr_local_pos, x2_w, y2_w, drone->yaw_in_Rad);
      x2_b = drone->check_x_b;
      y2_b = drone->check_y_b;

      //Evaluate a,c parameters of the line in body frame
       a_b =  ((y2_b  - y1_b)/(x2_b - x1_b));
       c_b =  ((-(a_b) * x1_b) + y1_b);
      
      drone->xh_body << a_b, c_b;
      //cout << "[KF EXP FUNC] ------------------____> drone.xh_b: " << drone.xh_b << endl; 
       
}





float target_coo_given_distance_setpoint(float setpoint_distance, float x_target_body, float y_target_body)
{
  /*
  INPUT: Desired setpoint_distance on the line 
        Dx coo on BODY
        Dy coo on BODY
  
  OUTPUT: DX,DY coo on BODY at desired setpoint_distance from BODY FRAME
  */
  
  float i = sqrt(pow(x_target_body, 2) + pow(y_target_body, 2));
  float alfa = acos(x_target_body/i);
  if  (y_target_body < 0)
  {
    alfa = -1*alfa;
  }
  mission.x_target = setpoint_distance * cos(alfa);
  mission.y_target= setpoint_distance * sin(alfa);
  //cout << "ALFA: " << alfa << endl;
 
 // ros::Duration(3.0).sleep(); 
  return alfa;


}



void evaluate_GPS_line_error_in_background(Drone *drone)
{
  /*
  Errore punto Linea tra BF drone e GPS LINE in BACKGROUND
  */
   
  float a = 0.0; //equivale a m coefficente angolare
  float c = 0.0;
  float b = -1;
  
  float x_target_P1_body = 0.0;
  float y_target_P1_body = 0.0;
  float x_target_P2_body = 0.0;
  float y_target_P2_body = 0.0;
  
  float yaw_for_rot = 0.0;

  //Take coordinate of point P1 rotated respect drone body frame ---> Frame Local placed in P1 --> Il punto P1 rispetto al frame local P1 è in 0,0
  geometry_msgs::Point new_curr_local_position;
  new_curr_local_position.x = drone->new_current_local_pos.x;
  new_curr_local_position.y =  drone->new_current_local_pos.y;
  yaw_for_rot = change_yaw_for_translation_between_locals_frame(drone);
  drone->Rotation_local_GF_to_BF(new_curr_local_position, 0,0, drone->yaw_in_Rad );
  x_target_P1_body = drone->check_x_b;
  y_target_P1_body = drone->check_y_b;
  //Take coordinate of point P2 rotated respect drone body frame 
  
  x_target_P2_body =  drone->target_offset_respect_local_position.x; //Gia espresso in Local
  y_target_P2_body =  drone->target_offset_respect_local_position.y; 
  
  drone->Rotation_local_GF_to_BF(new_curr_local_position, x_target_P2_body,y_target_P2_body, drone->yaw_in_Rad );
  x_target_P2_body = drone->check_x_b;
  y_target_P2_body = drone->check_y_b;

  cout<< "[GPS BACKGROUND LINE]  BACKGROUND GPS LINE!" << endl;
  //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0 nel BODY FRAME
  a =  ((y_target_P2_body  -  y_target_P1_body)/(x_target_P2_body - x_target_P1_body));
  c = ((-(a) * x_target_P1_body) + y_target_P1_body);
 
  
  //Find distance point line: point is the body Origin and line is the line r
   double e = (abs(c)/(sqrt(pow(a,2) + pow(b,2))));
   waypoints.error_from_GPS_line = e;
   
  outFile24 <<   waypoints.error_from_GPS_line << "\n";
  cout<<"[GPS BACKGROUND LINE] POINT LINE ERROR: " <<  waypoints.error_from_GPS_line  <<endl;
}





void  evaluate_control_point(Drone *drone, bool from_image, bool GPS_background_line_control, bool KF_init)
{
/*
INPUT: Coo Punto P1 rispetto (LOCAL FRAME) rispetto Drone Body Frame 
       Coo Punto P2 (Target Waypoint Fine Panlleo) rispetto Drone Body Frame 
       Yaw come in DJI sim

OUTPUT:  COO di Target su linea --> Coordinate Punto D date da regola parallelogramma di Vx con Vy 
         Vx --> vettore unitario parallelo alla retta espressa nel BOPDY FRAME
         Vy -->  vettore unitario perp alla retta espressa nel BOPDY FRAME

         Le Coo in Output Sono espresse nel frame di HOME 
*/

    //Equation line parameters Passing through WAYPOINTS P1 and P2 ---> GPS LINE 
    float a = 0.0; //equivale a m coefficente angolare
    float c = 0.0;
    float b = -1;

    float x_target_P1_body = 0.0;
    float y_target_P1_body = 0.0;
    float x_target_P2_body = 0.0;
    float y_target_P2_body = 0.0;
     
    float yaw_for_rot = 0.0;
    
    //Take coordinate of point P1 rotated respect drone body frame ---> Frame Local placed in P1 --> Il punto P1 rispetto al frame local P1 è in 0,0
    geometry_msgs::Point new_curr_local_position;
    new_curr_local_position.x = drone->new_current_local_pos.x;
    new_curr_local_position.y =  drone->new_current_local_pos.y;
    yaw_for_rot = change_yaw_for_translation_between_locals_frame(drone);
    drone->Rotation_local_GF_to_BF(new_curr_local_position, 0,0, drone->yaw_in_Rad );
    x_target_P1_body = drone->check_x_b;
    y_target_P1_body = drone->check_y_b;
  
    //Take coordinate of point P2 rotated respect drone body frame 
    x_target_P2_body =  drone->target_offset_respect_local_position.x; //Gia espresso in Local
    y_target_P2_body =  drone->target_offset_respect_local_position.y; 
  
    drone->Rotation_local_GF_to_BF(new_curr_local_position, x_target_P2_body,y_target_P2_body, drone->yaw_in_Rad );
    x_target_P2_body = drone->check_x_b;
    y_target_P2_body = drone->check_y_b;

    if (from_image == false)
    {
      
      cout<< "[EVALUATE CONTROL LINE] #### FOLLOWING GPS LINE!" << endl;
  
      //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0 nel BODY FRAME
       a =  ((y_target_P2_body  -  y_target_P1_body)/(x_target_P2_body - x_target_P1_body));
       c = ((-(a) * x_target_P1_body) + y_target_P1_body);
       cout<<"[EVALUATE CONTROL LINE] a BODY GPS: " << a << " c BODY GPS: " << c <<endl;
    }
    else
    {
      cout << "                                                                " << endl;
      cout<< "[EVALUATE CONTROL LINE] ##### FOLLOWING VISION LINE!" << endl;
     
      a = drone->xh_body[0];
      c = drone->xh_body[1];
 
     
    
      cout<<"[EVALUATE CONTROL LINE] a BF VISION: " << a << " c BF VISION: " << c <<endl;
    }


    //Find distance point line: point is the body Origin and line is the line r
    double e = (abs(c)/(sqrt(pow(a,2) + pow(b,2))));
   //waypoints.error_from_GPS_line = e;
    //Define vector Vx starting from body frame origin and parallel to the line r
    double Vx[2] = {1/a, -1/b};
    
    double Kx = 0.9; //Coefficiente moltiplicativo del vettore parallelo alla retta r
    double Vx_norm[2] = {Kx * ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[0]), Kx* ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[1])};
    //cout<<"target_point: "<< target_point<<endl;
    float rot_Vx = 0.0;
    float rot_Vy = 0.0;
    
    if (mission.target_point == 1)
    {
      //punto di target è P1:
       if (x_target_P1_body >= 0 &&  Vx_norm[0] < 0)
       {
    
          rot_Vx =  Vx_norm[0] * cos(C_PI) - Vx_norm[1] * sin(C_PI);
          rot_Vy = Vx_norm[0] * sin(C_PI) + Vx_norm[1] * cos(C_PI);
          Vx_norm[0] = rot_Vx;
          Vx_norm[1] = rot_Vy;
          //cout<< "sono qui"<< endl;
       }
       else
       {
          Vx_norm[0] = Vx_norm[0];
          Vx_norm[1] = Vx_norm[1];
       }
    } 
    else
    {
      if (x_target_P2_body >= 0 &&  Vx_norm[0] < 0)
        {
          rot_Vx =  Vx_norm[0] * cos(C_PI) - Vx_norm[1] * sin(C_PI);
          rot_Vy = Vx_norm[0] * sin(C_PI) + Vx_norm[1] * cos(C_PI);
          Vx_norm[0] = rot_Vx;
          Vx_norm[1] = rot_Vy;
           //  cout<< "sono quiiiiii"<< endl;
       }
       else
       {
          Vx_norm[0] = Vx_norm[0];
          Vx_norm[1] = Vx_norm[1];
       }
    }
    
     //Define vector Vy starting from body frame origin and perpendicular to line r
    //Il vettore Vy è moltiplicato pe run guadagno Ky e anche per l'errore dato dalla distanza punto retta e, la quale deve tendere a zero
    float Ky = 1; //Coefficiente moltiplicativo del vettore perp alla retta r
    double Vy_norm[2] = {0.0,0.0};

    double Vy[2] = {1/b, 1/a};
    Vy_norm[0] = Ky *e*((1/(sqrt(pow(Vy[0],2) + pow(Vy[1],2)))) * Vy[0]); // * e
    Vy_norm[1] = Ky *e*((1/(sqrt(pow(Vy[0],2) + pow(Vy[1],2)))) * Vy[1]); // *e 
  
    if (c > 0 && Vy_norm[1] < 0)
    {
        Vy_norm[1] = -1 * Vy_norm[1];
    }
    else if (c < 0 && Vy_norm[1] > 0)
    {
        Vy_norm[1] = -1 * Vy_norm[1];
    }

    // cout<<" Vx_norm[0]: "<<  Vx_norm[0] << " Vx_norm[1]: "<<  Vx_norm[1] <<endl;
    // cout<<" Vy_norm[0]: "<<  Vy_norm[0] << " Vy_norm[1]: "<<  Vy_norm[1] <<endl;

    //Find equation parameters of the line r1 passing trhoug vector Vx. Line r1 is parallel to line r but passing throung origin --> ha los tessto coefficiente angolare a della retta r
    float a1 = a;
    float b1 = -1;
    float c1 = 0;
    //Find equation parameters of the line r2 passing trhoug vector Vy. Line r2 is perpendicular  to line r but passing throung origin --> coeff angolare è 
    float a2 = 1/a; //Vy_norm[1]/Vy_norm[0];
    float b2 = -1;
    float c2 = 0;
    
    /*++++++++++++++Parallelogram rules. Find Vector V = Kx*Vx_norm + Ky*e*Vy_norm ***********/
    
    //Trovo retta r3 parallela a retta r2 ma passante per il punto B a cui tende vettore Vx
    float a3 = a2;
    float b3 = -1;
    float c3 = -a3*Vx_norm[0] + Vx_norm[1];
    
    //Trovo retta r4 parallela a retta r1 ma passante per il punto C a cui tende vettore Vy
    float a4 = a1;
    float b4 = -1;
    float c4 = -a4*Vy_norm[0] + Vy_norm[1];
    
    //Trovo punto di intersezioe D, al quale tende il vettore V. Il punto D è il punto di controllo
    float D[2] = {((b4*c3 - b3*c4)/(b3*a4 - b4*a3)), 0};
    D[1] = (-a3 * D[0] - c3)/b3;
    drone->control_x_coo = abs(D[0]);
    drone->control_y_coo = D[1];
    //Se le coordinate sono NAN alla prima iterazione le metto 0.0;
    if (drone->control_x_coo != drone->control_x_coo && drone->control_y_coo != drone->control_y_coo)
    {
        drone->control_x_coo = 0.0;
        drone->control_y_coo = 0.0;
    }
  
   cout<<"[EVALUATE CONTROL LINE]  D[0]: "<< D[0] << " D[1]: "<<  D[1] <<endl;
   
  /*
   Il punto di controllo è espresso nel body frame.
   Necessario ruoptarlo rispetto a frame HOME per avere il target da inviare al controllo. 

  */
   
     //Place SetPoints Every tot METERS ALONG THE LINE Depending on drone Position --> SETPOINTS placed in BODY FRAME
    
    
     float alfa;
    cout << "[EVALUATE CONTROL LINE] DISTANCE TO LOCAL SETPOINT: " << sqrt(pow( drone->x_target - drone->current_local_pos.x ,2) + pow( drone->y_target - drone->current_local_pos.y ,2)) << endl;
    if (from_image == true && mission.inbound_counter > 200400)
    {

       
        if (sqrt(pow( drone->x_target - drone->current_local_pos.x ,2) + pow( drone->y_target - drone->current_local_pos.y ,2)) < 0.8 )
        {
        alfa = target_coo_given_distance_setpoint(mission.line_setpoint_distance,  drone->control_x_coo, drone->control_y_coo);
        // mission.x_target = drone->control_x_coo + mission.line_setpoint_distance;
        // mission.y_target = drone->control_y_coo;

        cout<<"[EVALUATE CONTROL LINE] -----> BODY FRAME x_target: " <<  mission.x_target << endl;
        cout<<"[EVALUATE CONTROL LINE] -----> BODY FRAME y_target: " <<  mission.y_target << endl; 
        
        //CHANGE YAW
        yaw_for_rot = change_yaw_for_translation_between_locals_frame(drone);
    //    //Ruotop punto di controllo dal BODY FRAME al HOME FRAME 
        drone->Rotation_BF_to_local_GF_des_pos_original(drone->current_local_pos, mission.x_target , mission.y_target, drone->yaw_in_Rad);
        drone->x_target = drone -> check_x_local;
        drone->y_target = drone -> check_y_local;
        }
      
    } 
    else
    {

    
    /*
    Rotation Around y axis of local frame:
    X_D^B = -X_D^H
    Y_D^B = Y_D^H
    */
  
    //SCOMMENTARE PER MUOVERSI SEGUENDO LA RETTA SENZA I SETpOINT
    yaw_for_rot = change_yaw_for_translation_between_locals_frame(drone);
    //cout<<"[EVALUATE CONTROL LINE] yaw_for_rot: " << yaw_for_rot << endl;
    drone->Rotation_BF_to_local_GF_des_pos_original(drone->current_local_pos, drone->control_x_coo , drone->control_y_coo,  drone->yaw_in_Rad);
    
    drone->x_target = drone->check_x_local;
    drone->y_target = drone->check_y_local;

    }
  //  drone->rotate_target_position_from_new_local_to_HOME_local(drone->x_target, drone->y_target );
  //  drone->x_target  = drone->x_target_HOME;
  //  drone-> y_target = drone->y_target_HOME;

   cout<<"[EVALUATE CONTROL LINE] -----> HOME FRAME LOCAL TARGET SETPOINT X : " << drone->x_target << endl;
   cout<<"[EVALUATE CONTROL LINE] -----> HOME FRAME LOCAL TARGET SETPOINT Y : " << drone->y_target << endl;
   cout<<"[EVALUATE CONTROL LINE] -----> inbound_counter: " << mission.inbound_counter << endl;
   cout << "                                                                                      " << endl;
  //Translate coordiante to LOCAL HOME frame --> referred to current_local_position
  //---> Velocità sono riferite al frame HOME LOCAL
  



}









//Switch Functions
void take_off(Drone *drone, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
  if ( mission.take_off_result == false)
  {
   if (drone->takeoff() ) 
   {
       ROS_INFO_STREAM("Take OFF Successfull");
      //  ros::Duration(5.0).sleep(); 
       mission.take_off_result = true;
       //Update Drone Frame Position 
       mission.state = 0;
       return; 
   }
  }
  //Reach Desired Flying Altitude to start the Mission
  else
  {
    
     mission.setTarget(0, 0, drone->z_des, 0); 
    // cout << "drone->current_gps: " << drone->current_gps << endl;
     //Evaluate offset done by the drone between its current position and starting position 
     localOffsetFromGpsOffset(drone->local_drone_position, drone->current_gps, mission.start_gps_location); 
     mission.x_offset_remaining = mission.target_offset_x - drone->local_drone_position.x; 
     mission.y_offset_remaining = mission.target_offset_y - drone->local_drone_position.y; 
     mission.z_offset_remaining = mission.target_offset_z - drone->local_drone_position.z; //localOffset è la oposizione attuale del drone rispetto al frame di riferimento dopo la trasformazione da GPS coo
     
     mission.cartesian_distance_2D = sqrt(pow( mission.x_offset_remaining,2) + pow(mission.y_offset_remaining,2));
     mission.cartesian_distance_3D = sqrt(pow(mission.target_offset_x - drone->current_local_pos.x,2) + pow(mission.target_offset_y - drone->current_local_pos.y,2) +  pow(mission.target_offset_z - drone->current_local_pos.z,2));
     
     drone->x_pos_cmd = 0;
     drone->y_pos_cmd = 0;
     drone->z_pos_cmd = mission.start_local_position.z + mission.target_offset_z; // = mission.target_offset_z  = 5
    
     drone->x_vel_cmd = 0;
     drone->y_vel_cmd = 0;
     drone->z_vel_cmd =  pid_z->position_control_knowing_velocity(mission.target_offset_z, drone->current_local_pos.z , 0, drone->current_local_velocity.vector.z); 
     drone->yaw_des_rad = 0;
     drone->yaw_rate_cmd = 0;
     
     //publish_command_position_to_drone(drone);
     publish_command_velocity_to_drone(drone);
     cout << "Yaw_drone in deg : " << drone->yaw_in_Rad << endl;
     cout << "Yaw_in_rad_sim: " << drone->yaw_in_Rad_sim  << endl;
  }

  cout << "CASE 0: Distance: " << mission.cartesian_distance_3D << endl;
  if (mission.cartesian_distance_3D > 1.0)
  {

      mission.state = 0;
  }
  else
  {
    ROS_INFO_STREAM("CASE 0: Desired Altitude Reached---> Reaching Start Waypoint");
    //check_battery_status
    //check_gps_signals
    //check_for_errors
    ros::Duration(3.0).sleep(); 
    mission.state = 1;
    mission.enter_state = 1;
    waypoints.counter = 0;
  }

   
}


void  align_Yaw_with_starting_GPS_waypoint(Drone *drone, PID *pid_z, PID *pid_yaw)
{
   //Evaluate Yaw between Drone Position and GPS waypoint
 
   evaluate_des_GPS_WAYPOINT_yaw_angle(drone);
   mission.setTarget(0, 0, drone->z_des, 0); 
   //evaluate_control_offset();
   drone->x_vel_cmd = 0;
   drone->y_vel_cmd = 0;
   drone->z_vel_cmd =  pid_z->position_control_knowing_velocity(mission.target_offset_z, drone->current_local_pos.z , 0, drone->current_local_velocity.vector.z); 
   drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity( drone->yaw_in_Rad_sim, drone->yaw_des_rad_sim, 0, drone-> yaw_vel);
  //  drone->yaw_err_rad =  abs(drone->yaw_des_rad) - abs(drone->yaw_in_Rad);
   drone->yaw_err_rad =  drone->yaw_in_Rad_sim - drone->yaw_des_rad_sim;
   cout<<"[YAW ALIGNMENT] drone->yaw_des: " << drone->yaw_des_rad << " drone->yaw: " << drone->yaw_in_Rad << endl;
   cout<<"[YAW ALIGNMENT] drone->yaw_in_Rad_sim: " << drone->yaw_in_Rad_sim << endl;
   cout<<"[YAW ALIGNMENT] drone->yaw_err: " <<  drone->yaw_err_rad << endl;
   
   publish_command_velocity_to_drone(drone);
   
   mission.cartesian_distance_2D  = sqrt(pow( mission.x_offset_remaining,2) + pow(mission.y_offset_remaining,2));
   if ( abs(drone->yaw_err_rad) < 0.2)
   {
     mission.state = mission.enter_state + 1; //Aumenta in automatico per passare allo stato jump panels se arrivo da navigation 
     ros::Duration(1.0).sleep(); 
     mission.inbound_counter = 0;
     mission.end_panel_reached = false;
   }
   else
   {
     //Align Yaw
     mission.state = 1; //
   }
   
   //Publish to Gazebo to maintain camera fixed in starting psoition
  drone-> control_x_coo = 0.0;
  drone-> control_y_coo = 0.0;
  drone-> publish_control_D_point_to_GAZEBO();
}


void reaching_starting_position(Drone *drone,PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
  /*
  La funzione fa riferimento al frame NED --> asse x verso Nord e asse y verso est
  */

  waypoints.counter = 0;
  if (mission.inbound_counter == 0)
  {
    double start_lat =  waypoints.GPS_waypoints_lat[ waypoints.counter];
    double start_lon =  waypoints.GPS_waypoints_lon[ waypoints.counter];

    evaluate_local_offset_between_GPS_target_and_body_frame(start_lat, start_lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_drone_position);
    mission.setTarget( drone->target_offset_respect_drone_position.x ,  drone->target_offset_respect_drone_position.y, drone->z_des, 0); 
  
    mission.breaking_counter = 0.0;
  }
  
  
  evaluate_des_GPS_WAYPOINT_yaw_angle(drone);
  localOffsetFromGpsOffset(drone->local_drone_position, drone->current_gps, mission.start_gps_location); 
  mission.x_offset_remaining = mission.target_offset_x - drone->local_drone_position.x; 
  mission.y_offset_remaining = mission.target_offset_y - drone->local_drone_position.y; 
  mission.z_offset_remaining = mission.target_offset_z - drone->local_drone_position.z; //localOffset è la oposizione attuale del drone rispetto al frame di riferimento dopo la trasformazione da GPS coo
  

  cout<<"[REACHING STARTING POSITION] x_off: " << mission.x_offset_remaining << endl;
  cout<<"[REACHING STARTING POSITION] y_off: " << mission.y_offset_remaining << endl;
  
  mission.cartesian_distance_2D = sqrt(pow( mission.x_offset_remaining,2) + pow(mission.y_offset_remaining,2));
  // mission.cartesian_distance_3D = sqrt(pow(mission.target_offset_x - drone->current_local_pos.x,2) + pow(mission.target_offset_y)

// VELOCITY CONTROL  --> 
/* NB: INVERTO LE COO DI TARGET DI X E Y PERCHE L'OFFSET LO CALCOLA RISPETTO AL FRAME LOCALE, IL QUALE PRESENTA X  E Y INVERTITE RISPETTO A QUELLO OTTENUTO 
TRAMITE LA TRASFORMAZIONE DELLA LAT LON DEL GPS A DELTA X DELTA Y */

  drone->x_vel_cmd = pid_x->position_control_knowing_velocity(mission.target_offset_y, drone->current_local_pos.x , 0, drone->current_local_velocity.vector.x);
  drone->y_vel_cmd = pid_y->position_control_knowing_velocity(mission.target_offset_x, drone->current_local_pos.y , 0, drone->current_local_velocity.vector.y);
  drone->z_vel_cmd = pid_z->position_control_knowing_velocity(mission.target_offset_z, drone->current_local_pos.z , 0, drone->current_local_velocity.vector.z); 
  
  drone->yaw_rate_cmd =  pid_yaw->position_control_knowing_velocity( drone->yaw_in_Rad_sim, drone->yaw_des_rad_sim, 0, drone-> yaw_vel);
  

  drone->x_pos_cmd =  0.4*mission.y_offset_remaining ;
  drone->y_pos_cmd = 0.4*mission.x_offset_remaining ;
  drone->z_pos_cmd = drone->z_des;
   
  // float sat_x_off = 1;
  // float sat_y_off = 1;
  // drone->saturation_offset( drone->x_pos_cmd,  drone->y_pos_cmd, sat_x_off, sat_y_off);
  publish_command_position_to_drone(drone);
  //publish_command_velocity_to_drone(drone);
 
  if (mission.target_reached == true)
  {
     mission.state = 1; //Allineo Yaw con Waypoint finale 
     mission.enter_state =  3; //2 For obtain control  /Permette di passare allo stato Navigation da stato align Yaw 
     mission.target_point = 2;
    
     mission.inbound_counter = 0;
     mission.target_reached = false;
     mission.breaking_counter = 0;
     //Initialize Drone Position --> Sort of transformation respect world frame placed in HOME POINT
     //The yaw evaluation is done automatically by the drone.
    //Save Last Z altitude
    drone->last_position_z = drone->current_local_pos.z;
    mission.start_gps_location = drone->current_gps;
    mission.start_local_position = drone->current_local_pos;
    mission.referred_local_yaw = drone-> yaw_in_Rad_sim; //Salvo lo yaw del drone nel nuovo local frame in modo da poter avere uno yaw di riferimento per le trasformazioni
    waypoints.counter = waypoints.counter + 1;
    ros::Duration(1.0).sleep();
   
  }
  else
  {
    mission.state = 2;
    mission.inbound_counter = mission.inbound_counter + 1;
    //For experiment 
   
  }
  //GAZEBO LINK
  drone-> control_x_coo = 0.0;
  drone-> control_y_coo = 0.0;
  drone-> publish_control_D_point_to_GAZEBO();
}

/*
IN NAVIGATION:
           ^x
 FRAME GPS: |                ---> Le coo locali sono espresse rispetto questo frame quando trasformo un waypoints in GPS coo in coo locali (asse x verso NORD? verificare)
            |
            _____> y

FRAME LOCAL: ^ y
             |
             |              ---> la posizione del drone viene data rispetto a questo frame, orientato come il frame GPS ma con assi inverti
             |____> x            il frame locale corrisponde al frame del drone, con asse x asse di pitch e y di roll.
                                 Da topic: YAW = PI/2 quando drone allineato al frame locale (esempio nel punto di decollo)
                                 


FRAME DXRONE CONSIDERATO NEL CODICE:
^ x
|
|                        YAW = 0 quando YAW = PI/2 da topic. 
|_____> y              --> L'angolo di Yaw è ruotato di PI/2 (vedi IPAD) dalla funxione --> evaluate_des_GPS_WAYPOINT_yaw_angle()



NEL CASO DI NAVIGATION VERSO WAYPOINT IN FOLLOW LINE:
Riferito a orientazione BF



                                                                    ^ y
                                                                        |
                                                                        |
                    ^ y                                                 |____> X 
                    |
                    |
              <-----|
              x

----> IN FOLLOW LINE:
BF è riferit al Local_FRAME tramite una ROT(theta)_Y = -PI , NEssuna rotazioen di YAW.
YAW_LOCAL in NAVIGATIO (come nel frame) --> YAW = -C_PI/2
--> Se drone varia il prorpio YAW in navigation, il nuovo YAW corrisponde a:

YAW LOCA + PI/2                  se YAW >-C_PI and YAW < C_PI/2
-C_PI + (YAW_LOCAL - C_PI/2)      otherwise
 

 guardare IPAD per magguiiori info
 */
float a_GF = 0.0;
float c_GF = 0.0;
void navigation(Drone *drone,KF *Kalman_Filter, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
   float GPS_background_line_control = false;
   float yaw_rot = 0.0;

   //Default 
   bool KF_init = false;
   

  //GPS COO OF P2 WAYPOINT
  double P2_lat =  waypoints.GPS_waypoints_lat[ waypoints.counter]; //--> P2 end point of the panel, used to initialize KF
  double P2_lon =  waypoints.GPS_waypoints_lon[ waypoints.counter];

  //Obtain the new curr drone local position respect the frame placed in point P1
 
  drone->rotate_drone_position_respect_new_origin_frame(mission.start_local_position ); 
  

   //Per rotazione intorno asse Y local del BF.. vedi sopra
   geometry_msgs::Point new_curr_local_pos;
   new_curr_local_pos.x = drone->new_current_local_pos.x;
   new_curr_local_pos.y = drone->new_current_local_pos.y;

  
   
  // Navigation using KF 
  if (mission.inbound_counter == 0)
  {
      
    //EValuate GPS TARGET WAYPOINT P2 RESPECT LOCAL FRAME ORIENTED AS IN HOME POINT --> coerente con local_position topic
    //---> inverto x y direttamente nella funzione 
    evaluate_local_offset_between_GPS_target_and_local_frame(P2_lat, P2_lon, drone->current_gps, drone->new_current_local_pos, drone->target_offset_respect_local_position);
    
    //EValuate GPS TARGET WAYPOINT P2 RESPECT HOME FRAME ORIENTED 
    evaluate_local_offset_between_GPS_target_and_HOME_frame(P2_lat, P2_lon, drone->target_offset_respect_HOME_position);
    mission.setGPSTarget( drone->target_offset_respect_HOME_position.x ,  drone->target_offset_respect_HOME_position.y , drone->z_des, drone->yaw_des_rad_sim);
    
    //EValuate GPS TARGET WAYPOINT P2 RESPECT DRONE FRAME ORIENTED AS IN HOME POINT 

    evaluate_local_offset_between_GPS_target_and_body_frame(P2_lat, P2_lon, drone->current_gps, drone->new_current_local_pos, drone->target_offset_respect_drone_position); 
    drone->target_offset_respect_drone_position.x =  drone->target_offset_respect_drone_position.y;
    drone->target_offset_respect_drone_position.y =  drone->target_offset_respect_drone_position.x;
    //ROTATE POINT P1 RESPECT DRONE BODY FRAME REFERRED TO LOCAL FRAME PLACED IN P1
    //---> BF has a rotation around Y local axis, the yaw start from zero after the rotation
    
    yaw_rot = change_yaw_for_translation_between_locals_frame(drone);
   
    drone->Rotation_local_GF_to_BF(new_curr_local_pos, drone->target_offset_respect_local_position.x, drone->target_offset_respect_local_position.y, drone->yaw_in_Rad );
    drone->target_offset_respect_drone_position.x = drone->check_x_b;
    drone->target_offset_respect_drone_position.y = drone->check_y_b;
   
    
     //Serve per avere lo yaw allineato con il waypoint di fine pamnello
    evaluate_des_GPS_WAYPOINT_yaw_angle(drone);
  
    /*Initialize Kalman Filter 
    KALMAN FILTER initialized with actual position of the drone and the point P2 at end of the panel express in local coordinate respect the new frame placed in point P1 
    */
  
    Kalman_Filter-> Kalman_filter_initialization(0.0,  0.0, drone->target_offset_respect_local_position.x, drone->target_offset_respect_local_position.y);
    cout << "a GPS: " << Kalman_Filter -> Obtain_Kalman_filter_GPS_state()[0] << " c GPS: " << Kalman_Filter -> Obtain_Kalman_filter_GPS_state()[1] << endl;
    ros::Duration(1.0).sleep(); 
    
    //Init position for evaluate control function:
    //---FA riferimento a new_curr_local_pos in quanto conseguenza della rotazione intorno a Y
    drone->x_target = drone->current_local_pos.x;
    drone->y_target = drone->current_local_pos.y;

    //mission.setTarget( drone->target_offset_respect_drone_position.x ,  drone->target_offset_respect_drone_position.y , drone->z_des, 0); 
    mission.breaking_counter = 0;
    mission.inbound_counter = mission.inbound_counter + 1;
    return;
  }
  
  
  //Evaluate TARGET Offset respect drone body frame referred to new current local frame in P1
  evaluate_local_offset_between_GPS_target_and_body_frame(P2_lat, P2_lon, drone->current_gps, drone->new_current_local_pos, drone->target_offset_respect_drone_position);
  drone->target_offset_respect_drone_position.x =  drone->target_offset_respect_drone_position.y; //devo invertirli perche li sto considerando con new_current_local_pos!!!!!!! --> con current_local_pos invece non necesario perche allineati a GPS
  drone->target_offset_respect_drone_position.y =  drone->target_offset_respect_drone_position.x;

  yaw_rot = change_yaw_for_translation_between_locals_frame(drone);
  drone->Rotation_local_GF_to_BF(new_curr_local_pos, drone->target_offset_respect_local_position.x, drone->target_offset_respect_local_position.y, yaw_rot);
  drone->target_offset_respect_drone_position.x = drone->check_x_b;
  drone->target_offset_respect_drone_position.y = drone->check_y_b;
 
  

// ################à GPS offset target : NOnostante si NAvighi con Kalman importante tenere sotto controllo il target GPS
// ----> Se usate mission.target_GPS_offset in PID il drone naviga diretto verso il waypoint (INSERIRE TARGET X E Y INVERTITI NEL PID IN QUANTO PRESI DA GPS FRAME)
  localOffsetFromGpsOffset(drone->local_drone_position, drone->current_gps, mission.HOME_gps_location); 
  mission.x_GPS_offset_remaining = mission.target_GPS_offset_x - drone->local_drone_position.x; 
  mission.y_GPS_offset_remaining = mission.target_GPS_offset_y - drone->local_drone_position.y; 
  mission.z_GPS_offset_remaining = mission.target_GPS_offset_z - drone->local_drone_position.z; //localOffset è la oposizione attuale del drone rispetto al frame di riferimento dopo la trasformazione da GPS coo
  
  
  /*######################### OBSERVATIONS #################################
  Rotate obs from drone BF to LF:
  ------> NB: Per mia nuova notazione il frame drone subisce una rotazione sull'asse Y e poi sucessivamente sullo yaw.
  ---> Il nuovo angolo di Yaw è calcolato prendendo come riferimento YAW = 0 la posizione di termine della rotazione intorno a Y dal LF.
  --> Conseguenza della rotazione: (considerp un punto D riferitoo a BF)
  X_D^B = -X_D^L
  Y_D^B = Y_D^L
  */

 // Le osservazioni nel BF del Matrice Presentano segno meno rispetto a BF ARDRONE 
  // drone-> control_thermo_point1_y = - 1*  drone-> control_thermo_point1_y;
  // drone-> control_thermo_point2_y = - 1*  drone-> control_thermo_point2_y;
  //  drone-> control_RGB_point1_y = -1* drone-> control_RGB_point1_y;
  //   drone-> control_RGB_point2_y = -1*  drone-> control_RGB_point2_y;



  //Nelle rotazioni da da BF a local GF va bene lasciare drone->new_current_local_pos --> (i segni sono gia cambiati nella funzione)
  //Le osservazioni sono prese nel frame ARDRONE --> orientato gia con local frame  (se asse y cambiata di segno) --> nessuna rotazione intorno asse y
 // yaw_rot = change_yaw_for_translation_between_locals_frame(drone);
  drone->Rotation_BF_to_local_GF_des_pos_original(drone->new_current_local_pos, drone->control_thermo_point1_x, drone-> control_thermo_point1_y, drone->yaw_in_Rad);
  drone-> thermo_control_obs_P1_x_local = drone -> check_x_local;
  drone-> thermo_control_obs_P1_y_local = drone -> check_y_local;
  
 
  drone->Rotation_BF_to_local_GF_des_pos_original(drone->new_current_local_pos, drone->control_thermo_point2_x, drone-> control_thermo_point2_y,  drone->yaw_in_Rad);
  drone-> thermo_control_obs_P2_x_local = drone -> check_x_local;
  drone-> thermo_control_obs_P2_y_local = drone -> check_y_local;

  //Evaluate line passing trhough observation in GF 
  a_GF = (drone-> thermo_control_obs_P2_y_local  -   drone-> thermo_control_obs_P1_y_local)/(drone-> thermo_control_obs_P2_x_local - drone-> thermo_control_obs_P1_x_local);
  c_GF = (-1*a_GF* drone-> thermo_control_obs_P1_x_local ) + drone-> thermo_control_obs_P1_y_local ;

  drone->obs_thermo_GF << a_GF, c_GF;
  // // ###################### Obtain Information from camera RGB and THERMAL camera #####################
  
//NB: Frame ardrone in gazebo presenta la y nel senso opposto a quello in DJI
//---> Le osservazioni positive sulla Y nel BF del drone in Gazebo in realta sono negative ion quelle di DJI
  if(drone->flagDroneThermoControlPoint1 == true && drone->flagDroneThermoControlPoint2 ==true && mission.panel_array_initialization == false || drone->THERMO_image_control_count < 500)
   {
       Kalman_Filter->pass_to_KF_class_OBS_in_GF(a_GF,c_GF);
       //Kalman_Filter->Kalman_Filter_calculate(drone->control_thermo_point1_x,  drone-> control_thermo_point1_y,  drone->control_thermo_point2_x ,  drone->control_thermo_point2_y);
       //-----> EKF 
       //Necessario Usare yaw_in_Rad_sim perche faccio riferimento al frame current local pose, allineato con il frame HOME con asse x verso est e asse Y verso Nord
       
       Kalman_Filter->EKF_calculate(drone->control_thermo_point1_x, drone->control_thermo_point1_y, drone->control_thermo_point2_x, drone->control_thermo_point2_y,
                                                                                              drone->new_current_local_pos.x ,  drone->new_current_local_pos.y, drone->yaw_in_Rad );
       drone->xh_ = Kalman_Filter->Obtain_Kalman_filter_estimated_state();
       drone->obs = Kalman_Filter->Obtain_Kalman_filter_observation();
       drone->yh_ = Kalman_Filter->Obtain_Kalman_filter_estimated_observation();
       
       cout << "[EKF THERMO  ------->] Observations in BF : a  " << drone->obs[0] << " c: " << drone->obs[1] << endl;
       cout << "[EKF THERMO  ------->] Estimated states: a   " << drone->xh_[0] << " c: " << drone->xh_[1] << endl;
       cout << "[EKF THERMO  ------->] Estimated Observation in BF:  a_BF " << drone->yh_[0] << " c_BF: " << drone->yh_[1] << endl; 
       cout << "[EKF THERMO  ------->]  Observations in GF : a  " << a_GF << " c: " << c_GF << endl;
      from_image_Thermo = true;
      from_image = true;
      if (drone->flagDroneThermoControlPoint1 == true && drone->flagDroneThermoControlPoint2== true &&  abs(waypoints.error_from_GPS_line) < 15) //drone.image_control_count: permette la guida guidata dal filtro anche quando non sono rileavate piu osservazioni,
                                                                                  //finche la condizione sull'errore dalla GPS line è rispettata 
      {
        //Resetto counter 
        drone->THERMO_image_control_count = 0;
      }
       KF_estimation_exportation_in_BF_for_control_point_generation(drone, Kalman_Filter);
      
   }
   else
   {
       //Se non ci sono piu nuove osservazioni anche per oltre il numero concesso di osservazioni 
       //torno alla navigazione via GPS
       from_image_Thermo = false;
       from_image = false; 
       GPS_background_line_control = false;
   }
   


  
  
   //--------------------> RGB OBSERVATIONS 
  //cout << "[KALMAN FILTER RGB] P1 X^B: " << drone->control_RGB_point1_x<< "  P1 Y^B: " << drone->control_RGB_point1_y<< endl;

  yaw_rot = change_yaw_for_translation_between_locals_frame(drone);
  drone->Rotation_BF_to_local_GF_des_pos_original(drone->new_current_local_pos, drone->control_RGB_point1_x, drone-> control_RGB_point1_y, drone->yaw_in_Rad);
  drone-> RGB_control_obs_P1_x_local = drone -> check_x_local;
  drone-> RGB_control_obs_P1_y_local = drone -> check_y_local;
  
  //cout << "[KALMAN FILTER RGB] P1 X^L: " << drone-> RGB_control_obs_P1_x_local << "  P1 Y^L: " << drone-> RGB_control_obs_P1_y_local << endl;

  drone->Rotation_BF_to_local_GF_des_pos_original(drone->new_current_local_pos, drone->control_RGB_point2_x, drone-> control_RGB_point2_y, drone->yaw_in_Rad);
  drone-> RGB_control_obs_P2_x_local = drone -> check_x_local;
  drone-> RGB_control_obs_P2_y_local = drone -> check_y_local;

  //Evaluate line passing trhough observation in GF 
  a_GF = (drone-> RGB_control_obs_P2_y_local  -   drone-> RGB_control_obs_P1_y_local)/(drone-> RGB_control_obs_P2_x_local - drone-> RGB_control_obs_P1_x_local );
  c_GF = (-1*a_GF* drone-> thermo_control_obs_P1_x_local ) + drone-> thermo_control_obs_P1_y_local ;

  drone->obs_RGB_GF << a_GF, c_GF;
  if(drone->flagDroneRGBControlPoint1 == true && drone->flagDroneRGBControlPoint2 ==true && mission.panel_array_initialization == false || drone->RGB_image_control_count < 500)
   {
       Kalman_Filter->pass_to_KF_class_OBS_in_GF(a_GF,c_GF);
      //In ground frame
       //Kalman_Filter->Kalman_Filter_calculate(drone->RGB_control_obs_P1_x_local ,  drone->RGB_control_obs_P1_y_local ,  drone->RGB_control_obs_P2_x_local ,  drone->RGB_control_obs_P2_y_local);
       //--> IN BF 
       //Kalman_Filter->Kalman_Filter_calculate(drone->control_RGB_point1_x , drone->control_RGB_point1_y , drone->control_RGB_point2_x, drone-> control_RGB_point2_y);
       Kalman_Filter->EKF_calculate(drone->control_RGB_point1_x, drone-> control_RGB_point1_y,drone-> control_RGB_point2_x, drone-> control_RGB_point2_y,
                                                                                              drone->new_current_local_pos.x ,  drone->new_current_local_pos.y, drone->yaw_in_Rad );
       drone->xh_ == Kalman_Filter->Obtain_Kalman_filter_observation();
       drone->yh_ = Kalman_Filter->Obtain_Kalman_filter_estimated_observation();

       
       cout << "[EKF RGB  ------->] Observations : a  " << drone->obs[0] << " c: " << drone->obs[1] << endl;
       cout << "[EKF RGB  ------->] Estimated states: a   " << drone->xh_[0] << " c: " << drone->xh_[1] << endl;
       cout << "[EKF Kalman_Filter->Obtain_Kalman_filter_estimated_state();
       drone->obs  RGB  ------->] Estimated Observation in BF:  a_BF " << drone->yh_[0] << " c_BF: " << drone->yh_[1] << endl; 
       cout << "[EKF RGB  ------->] Observations in GF : a  " << a_GF << " c: " << c_GF << endl;
      from_image = true;
      from_image_RGB = true;
      if (drone->flagDroneRGBControlPoint1 == true && drone->flagDroneRGBControlPoint2 == true &&  abs(waypoints.error_from_GPS_line) < 15) //drone.image_control_count: permette la guida guidata dal filtro anche quando non sono rileavate piu osservazioni,
                                                                                  //finche la condizione sull'errore dalla GPS line è rispettata 
      {
        //Resetto counter 
        drone->RGB_image_control_count = 0;
      }
       KF_estimation_exportation_in_BF_for_control_point_generation(drone, Kalman_Filter);
       
   }
   else
   {
       //Se non ci sono piu nuove osservazioni anche per oltre il numero concesso di osservazioni 
       //torno alla navigazione via GPS
       from_image_RGB = false;
       
       if (from_image_Thermo == true )
       {
           from_image = true; //per control
       }
       else
       {
           from_image = false; //per control
       
         
       }
       
   }

// Check for KF INIT 
   KF_init = Kalman_Filter->Obtain_KF_initialization_flag(); 
 
  evaluate_control_point(drone, from_image, GPS_background_line_control, KF_init);
  //Velocity To Zero while KF is in Initialization 
  if (KF_init == true)
  {
   
    cout << "[KF INIT] WAITING KF INITIALIZATION " << endl;
    drone->x_target = mission.start_local_position.x;
    drone->y_target = mission.start_local_position.y;
    drone->z_target = drone->last_position_z;
    //Take ultrasonic data about altitude
    drone->reference_altitude_value = drone->ultrasonic_altitude_value;
    from_image = false;
  }
  else
  {
   
     /* GPS EVALUATION BACKGROUND */ 
  

     /* ########## PUBLISH CONTOL POINT D IN BF TO ARDRONE IN GAZEBO ############# */
    drone-> publish_control_D_point_to_GAZEBO();

  }
  
   
  /* Evaluate the GPS line offset */
  evaluate_GPS_line_error_in_background(drone);
  
   
   
   

  
  




  

 /*
            ^x
 FRAME GPS: |
            |
            _____> y

FRAME LOCAL: ^ y
             |
             |
             |____> x
             
*/

/*
  //Offset Drone Target espresso rispetto HOME frame relativo a GPS Transfomration 
  cout<<"[NAVIGATION] HOME x_off: " << mission.x_GPS_offset_remaining << endl;
  cout<<"[NAVIGATION] HOME y_off: " << mission.y_GPS_offset_remaining << endl;

  //Offset Drone Target espresso rispetto HOME frame relativo a local Transformation 
  cout<<"[NAVIGATION] HOME x_off: " << mission.y_GPS_offset_remaining << endl;
  cout<<"[NAVIGATION] HOME y_off: " << mission.x_GPS_offset_remaining << endl;

  
  //Offset Drone Target espresso rispettp frame locale posto in P1
  float X_local_offset_remaining =  drone->target_offset_respect_local_position.x ;
  float Y_local_offset_remaining =  drone->target_offset_respect_local_position.y ;
  cout<<"[NAVIGATION] LOCAL x_off: " << X_local_offset_remaining << endl; 
  cout<<"[NAVIGATION] LOCAL y_off: " << Y_local_offset_remaining << endl; 

  //Offset Drone Target respect drone body Frame 
  float X_BODY_offset_remaining =  drone->target_offset_respect_drone_position.x;
  float Y_BODY_offset_remaining =  drone->target_offset_respect_drone_position.y;
  cout<<"[NAVIGATION] BODY x_off: " << X_BODY_offset_remaining << endl; 
  cout<<"[NAVIGATION] BODY y_off: " << Y_BODY_offset_remaining << endl; 

*/
  mission.cartesian_distance_2D = sqrt(pow( mission.x_GPS_offset_remaining,2) + pow(mission.y_GPS_offset_remaining,2));
 
  /*NB: 
    Siccome la funzione evaluate control esegue calcoli e trasfromazioni su coo GPS GIA RIPORTATE SUL FRAME LOCALE DELK DRONE, non devo piu considerare l'offset di 90 gradi 
    tra il frame locale e il frame relativo alla trasformazione da GPS LAT LON a X Y locali.
    IN questo caso il target x corrisponde al target Y !!!!!
    ---> NEL CASO STESSI CONSIDERANDO UNA TRASFORMAZIONE DA GPS LAT LON A X Y LOCALI ALLORA DOVREI CONSIDERARE:
    drone->x_target = y_GPS_TARGET_in_LOCAL_FRAME;
    drone->y_target = x_GPS_TARGET_in_LOCAL_FRAME;
    --> Maggiori informazioni su ipad 
  */ 

  // drone->x_vel_cmd = pid_x->position_control_knowing_velocity(mission.target_GPS_offset_y , drone->current_local_pos.x , 0, drone->current_local_velocity.vector.x); //mission.target_GPS_offset_y
  // drone->y_vel_cmd = pid_y->position_control_knowing_velocity(mission.target_GPS_offset_x , drone->current_local_pos.y , 0, drone->current_local_velocity.vector.y);  //mission.target_GPS_offset_x
  // drone->z_vel_cmd = pid_z->position_control_knowing_velocity(mission.target_GPS_offset_z, drone->current_local_pos.z , 0, drone->current_local_velocity.vector.z); 
  
  //VELOCITY CONTROL 
  drone->x_vel_cmd = pid_x->position_control_knowing_velocity(drone->x_target , drone->current_local_pos.x , 0, drone->current_local_velocity.vector.x); //mission.target_GPS_offset_y
  drone->y_vel_cmd = pid_y->position_control_knowing_velocity(drone->y_target , drone->current_local_pos.y , 0, drone->current_local_velocity.vector.y);  //mission.target_GPS_offset_x
  drone->z_vel_cmd = pid_z->position_control_knowing_velocity(mission.target_GPS_offset_z, drone->current_local_pos.z , 0, drone->current_local_velocity.vector.z); 
  
  //POSITION CONTROL OFFSET
  drone->x_pos_cmd =  drone->x_target - drone->current_local_pos.x;
  drone->y_pos_cmd = drone->y_target - drone->current_local_pos.y;
  drone->z_pos_cmd = drone->z_des;

  //IF the altitude is controlled respect grpund
  if (mission.height_above_ground == true)
  {
    //If using the panels images 
    drone->z_pos_cmd = drone->z_des + 0.005*drone->offset;
    //If usinge the /height_above_take_off topi with ultrasonic data
    
    drone->z_pos_cmd = drone->z_des + drone->ultrasonic_offset;

  }

  //Il comando di velocità viene dato rispetto al frame di partenza, devo ruotarlo rispetto a quel frame

  
 // drone->yaw_des_rad = atan2(drone->target_offset_respect_drone_position.x, drone->target_offset_respect_drone_position.y);
  drone->yaw_rate_cmd =  pid_yaw->position_control_knowing_velocity(  drone->yaw_in_Rad_sim,  drone->yaw_des_rad_sim, 0, drone-> yaw_vel);
  
 
  /* #################### VARIOUS CHECK ########## */
  //Flag per follow GPS line o Vision Line
  if (mission.inbound_counter > 100)
  {
       mission.panel_array_initialization = false;
      // from_image = false;
  }

 
 if (from_image == true)
 {
   drone->image_control_count = drone->image_control_count + 1;
 }
 
if (waypoints.error_from_GPS_line > 30)
{
  from_image = false;
}

if (from_image_Thermo == true)
{
    drone->THERMO_image_control_count = drone->THERMO_image_control_count  + 1;
}

if (from_image_RGB == true)
{
    //FInche flag rimane true continuo ad incrementare counter. 
    // verra resettato solo quando i flag callback sono ture 
    drone->RGB_image_control_count = drone->RGB_image_control_count  + 1;
  
}


/* ARRIVO AL GPS WAYPOINT DI FINE PANNELLO 
Compute derivative of the distance function step by step in orde to see a change in variation:
Se la derivata ha segno negativo la distaza con il waypoint diminuisce 
Se la derivata ha segno positivo la disatnza con il waypoint aumenta.
--> rilevo cambio di pendenza per capire quando raggioungere il waypoint se la distanza non è rispettata.
*/
float derivative = mission.compute_distance_derivative();


if ( mission.cartesian_distance_2D < 25.0)
{
   mission.compute_path_difference( drone->new_current_local_pos.x,  drone->new_current_local_pos.x, drone->target_offset_respect_local_position);
} 

//mission.flag_navigate_to_waypoints = false;
// if ( mission.flag_navigate_to_waypoints==true  && KF_init == false)
// {
//   cout<<"[INFO NAVIGATION] ----> REACHING WAYPOINT: " << endl;
//   drone->x_vel_cmd = pid_x->position_control_knowing_velocity(mission.target_GPS_offset_y, drone->current_local_pos.x , 0, drone->current_local_velocity.vector.x); //mission.target_GPS_offset_y
//   drone->y_vel_cmd = pid_y->position_control_knowing_velocity(mission.target_GPS_offset_x , drone->current_local_pos.y , 0, drone->current_local_velocity.vector.y);  //mission.target_GPS_offset_x
//   drone->z_vel_cmd = pid_z->position_control_knowing_velocity(mission.target_GPS_offset_z, drone->current_local_pos.z , 0, drone->current_local_velocity.vector.z); 
//   cout << "[INFO NAVIGATION] X GPS: " << mission.target_GPS_offset_x << " Y GPS: " << mission.target_GPS_offset_y << endl;
  
//   //POSITION CONTROL OFFSET
//   drone->x_pos_cmd = mission.target_GPS_offset_y - drone->current_local_pos.x;
//   drone->y_pos_cmd =mission.target_GPS_offset_x- drone->current_local_pos.y;
//   drone->z_pos_cmd = drone->z_des;
  
// }

//Final Check 
if (KF_init == true)
{
  drone->x_vel_cmd = 0.0;
  drone->y_vel_cmd = 0.0;
  
  drone->x_pos_cmd = 0.0;
  drone->y_pos_cmd =0.0;

}
/* PUBLISH COMMAND VELOCITY TO DRONE */

// drone->sat_x_vel = 3;
// drone-> sat_y_vel = 3;
// drone->saturation(drone->x_vel_cmd , drone->y_vel_cmd, drone->sat_x_vel, drone->sat_y_vel);
//publish_command_velocity_to_drone(drone);
float sat_x_off = 1;
float sat_y_off = 1;
drone->saturation_offset( drone->x_pos_cmd,  drone->y_pos_cmd, sat_x_off, sat_y_off);
publish_command_position_to_drone(drone);

if (mission.target_reached == true)
{
   cout<<  "[INFO NAVIGATION] End Panel Waypoint Reached" << endl;
   mission.state = 2; //2  //Allineo Yaw con Waypoint sucessivo //1
   mission.enter_state =  1; //1 --> permette di raggiungere Reaching starting position //4 //Permette di passare allo stato Jump_panels da stato align Yaw, perche viene sommato + 1 
   mission.target_point = 1;
   mission.inbound_counter = 0;
   mission.target_reached = false;
   mission.flag_navigate_to_waypoints = false;
   mission.breaking_counter = 0;
   mission.panel_array_initialization = true;
   mission.end_panel_reached = true; //Necessario per camera in Gazebo per orientarsi su nuovo pannello
   //Publish zero control point to ARDRONE
   drone-> control_x_coo = 0.0;
   drone-> control_y_coo = 0.0;
   drone-> publish_control_D_point_to_GAZEBO();
   
   drone->last_position_z =  drone->current_local_pos.z;

   //Place New Frame in Waypoint
   mission.start_gps_location = drone->current_gps;
   mission.start_local_position = drone->current_local_pos;
   mission.referred_local_yaw = drone-> yaw_in_Rad_sim; //Salvo lo yaw del drone nel nuovo local frame in modo da poter avere uno yaw di riferimento per le trasformazioni
   drone->image_control_count = 0;
   drone->THERMO_image_control_count = 6000;
   drone->RGB_image_control_count = 6000;
   waypoints.counter = waypoints.counter + 1;
   ros::Duration(2.0).sleep();


}
else
{
  mission.inbound_counter = mission.inbound_counter + 1;
  mission.cartesian_distance_2D_old = mission.cartesian_distance_2D;
}


}




void jump_panels(Drone *drone, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{

  if (mission.inbound_counter == 0)
  {
   
    double start_lat =  waypoints.GPS_waypoints_lat[ waypoints.counter];
    double start_lon =  waypoints.GPS_waypoints_lon[ waypoints.counter];

    evaluate_local_offset_between_GPS_target_and_body_frame(start_lat, start_lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_drone_position);
    mission.setTarget(  drone->current_local_pos.y + drone->target_offset_respect_drone_position.x ,  drone->current_local_pos.x + drone->target_offset_respect_drone_position.y, drone->z_des, 0); 
  
    mission.breaking_counter = 0.0;
  }
  
  //evaluate_des_GPS_WAYPOINT_yaw_angle(drone);
  localOffsetFromGpsOffset(drone->local_drone_position, drone->current_gps, mission.HOME_gps_location); 
  mission.x_offset_remaining = mission.target_offset_x - drone->local_drone_position.x; 
  mission.y_offset_remaining = mission.target_offset_y - drone->local_drone_position.y; 
  mission.z_offset_remaining = mission.target_offset_z - drone->local_drone_position.z; //localOffset è la oposizione attuale del drone rispetto al frame di riferimento dopo la trasformazione da GPS coo
  

  cout<<"[PANELS JUMP] x_off: " << mission.x_offset_remaining << endl;
  cout<<"[PANELS JUMP] y_off: " << mission.y_offset_remaining << endl;
  cout<< "[PANELS JUMP] mission.target_offset_x: " << mission.target_offset_x  <<  " mission.target_offset_y: " <<  mission.target_offset_y << endl;
  cout<< "[PANELS JUMP] drone->local_drone_position.x: " << drone->local_drone_position.x <<  " drone->local_drone_position.y: " << drone->local_drone_position.y <<  endl;

  mission.cartesian_distance_2D = sqrt(pow( mission.x_offset_remaining,2) + pow(mission.y_offset_remaining,2));
  // mission.cartesian_distance_3D = sqrt(pow(mission.target_offset_x - drone->current_local_pos.x,2) + pow(mission.target_offset_y)

// VELOCITY CONTROL  --> 
/* NB: INVERTO LE COO DI TARGET DI X E Y PERCHE L'OFFSET LO CALCOLA RISPETTO AL FRAME LOCALE, IL QUALE PRESENTA X  E Y INVERTITE RISPETTO A QUELLO OTTENUTO 
TRAMITE LA TRASFORMAZIONE DELLA LAT LON DEL GPS A DELTA X DELTA Y */

  drone->x_vel_cmd = pid_x->position_control_knowing_velocity(mission.target_offset_y, drone->current_local_pos.x , 0, drone->current_local_velocity.vector.x);
  drone->y_vel_cmd = pid_y->position_control_knowing_velocity(mission.target_offset_x, drone->current_local_pos.y , 0, drone->current_local_velocity.vector.y);
  drone->z_vel_cmd = pid_z->position_control_knowing_velocity(mission.target_offset_z, drone->current_local_pos.z , 0, drone->current_local_velocity.vector.z); 
  
  drone->yaw_rate_cmd =  pid_yaw->position_control_knowing_velocity( drone->yaw_in_Rad_sim, drone->yaw_des_rad_sim, 0, drone-> yaw_vel);
  
// POSITION CONTROL --> Sempre pubblicato rispetto local_position frame
  drone->x_pos_cmd =  0.8*mission.y_offset_remaining ;
  drone->y_pos_cmd = 0.8*mission.x_offset_remaining ;
  drone->z_pos_cmd = drone->last_position_z; //drone->z_des;
 
  publish_command_position_to_drone(drone);
  //publish_command_velocity_to_drone(drone);
 

  //Continue to sent Command to Gazebo.
  //Sent x = 0.0 e y = 0.0 --> Arddrone sstop in its last position
  drone-> control_x_coo = 0.0;
  drone-> control_y_coo = 0.0;
  drone-> publish_control_D_point_to_GAZEBO();
  if (mission.target_reached == true)
  {
     mission.state = 1; //Allineo Yaw con Waypoint finale 
     mission.enter_state = 3; //Permette di passare a navigation dopo Yaw  alignment state 
     mission.target_point = 2;
    
     mission.inbound_counter = 0;
     mission.target_reached = false;
     mission.breaking_counter = 0;
   
    mission.start_gps_location = drone->current_gps;
    mission.start_local_position = drone->current_local_pos;
    mission.referred_local_yaw = drone-> yaw_in_Rad_sim; //Salvo lo yaw del drone nel nuovo local frame in modo da poter avere uno yaw di riferimento per le trasformazioni
    waypoints.counter = waypoints.counter + 1;
    ros::Duration(3.0).sleep();
   
  }
  else
  {
    mission.state = 5;
    mission.inbound_counter = mission.inbound_counter + 1;
  }

}









int main(int argc, char** argv)
{
  ros::init(argc, argv, "Solar_flight_control");
  ros::NodeHandle nh;
  

  //Initialize Drone and KF class
  ImageConverter ic;
  Drone drone;
  KF Kalman_Filter = KF();

  //Default Parameters 
  drone.z_des = 15;
  int desired_KF_waiting = 100; 
  mission.state = 0;
  mission.enter_state =0;

  bool control_altitude_by_image_flag = false;
  
  //LOAD PARAMS 
  nh.getParam("/solar_param/z_des", drone.z_des);
  nh.getParam("/solar_param/initial_state", mission.state);
  nh.getParam("/solar_param/initial_enter_state", mission.enter_state);
  nh.getParam("/solar_param/desired_waiting_KF_init_it", desired_KF_waiting);
  nh.getParam("/solar_param/control_altitude_by_image", control_altitude_by_image_flag);


  //Load GPS Coordinate from file txt
  load_GPS_coordinates();
  
  // load_GPS_coordinates_with_GAZEBO_OFFSET(HOME_lat, HOME_lon);
  Kalman_Filter.pass_to_class_initialization_waiting_value(desired_KF_waiting);
  Kalman_Filter.Kalman_filter_initialization(0.0,0.0, 0.0,0.0);
  
  mission.GAZEBO_LINK = true; //abilitare quando ho connessione con drone in Gazebo Link per osservazioni

   //Define Gains Controller
  float Kp_z = 0.5;
  float Kd_z = 0.4;
  float Kp_yaw = 0.6;
  float Kd_yaw = 0.3;
  float Kp_x = 0.9;
  float Kp_y = 0.85;//0.2;/dji_osdk_ros/velocity

  float Kd_x = 0.55;
  float Kd_y = 0.45;
  float Ki_x = 0.1;//0.9; //0.1
  float Ki_y = 0.1;//0.55; //0.1
  double integralx(0);
  double integraly(0);
  float dt = 0.01;
  float time = 0.0;
  //Initialize PID controller for x, y, z and yaw direction (dt, max_value, min_value, Kp, Kd, Ki)
  //Output of PID are desired velocities
  
  PID pid_x = PID(dt, 1.5, -1.5, Kp_x, Kd_x, Ki_x);
  PID pid_y = PID(dt, 1.5, -1.5, Kp_y, Kd_y, Ki_y);
   
  PID pid_z = PID(dt, 1, -1, Kp_z, Kd_z, 0.01);
  PID pid_yaw = PID(dt, 0.3, -0.3, Kp_yaw, Kd_yaw, 0.01);

  // Saturation for only FOLLOW LINW MODE
  drone.sat_x_vel = 1;
  drone.sat_y_vel = 1;

  Eigen::Matrix2f R;
  Eigen::Matrix2f Px;
  
  R << 0.5, 0.0,  
   0.0, 2;
  Px << 4.0, 0.0,
    0.0, 4.0;
     
  Kalman_Filter.pass_to_class_initialization_matrices(Px, R);
  
  //Obtain Drone Control
  bool obtain_control_result = drone.obtain_control(); 
  if (!obtain_control_result)
  {
    ROS_ERROR("Impossible to Obtain Drone Control by OSDK. Exiting.");
    //return 1;
  }
  else
  {
    ROS_INFO("Drone Control Obtained!");
    obtain_control_result = false;
  }
  
   
  if (!drone.set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    //return 1;
  }
  else
  {
    ROS_INFO("Local Position Settled!");
  }

  
  //Initialize Mission Switch
  //mission.state = 0; //0
 //mission.enter_state = 0;
  std_msgs::Bool user_control;
  int acquire_GPS_counter = 0;
  bool acquire_GPS_flag = true;
  int case_3_counter =  0;
  int counter_altitude_init = 0;
  ros::Rate r(30); 
  while(nh.ok()) 
  {
    //Loop local variables 
    float vel_x = 0.0;
    float vel_y = 0.0;
    float y = 0.0;
    //drone.z_des = 5.0;
    
    
   // cout << "latitude " << drone.current_gps.latitude << endl;
    switch(mission.state)
    {   
      //Take OFF ---> Reach Initial altitude of 5.0 meters
      case 0:
        mission.reset();
      //Taking OFF ---> Reach Desired Altitude 
        if (acquire_GPS_flag == true)
        {
          //SETTING HOME POSITION 
          mission.HOME_gps_location = drone.current_gps;
          mission.HOME_local_position = drone.current_local_pos;
          mission.start_gps_location = mission.HOME_gps_location ;
          mission.start_local_position =mission.HOME_local_position;

          cout << " mission.HOME_gps_location.lat: "<< mission.HOME_gps_location.latitude << endl;
          cout << " mission.HOME_gps_location.lon: "<< mission.HOME_gps_location.longitude << endl;
          if (acquire_GPS_counter > 20)
          {
            acquire_GPS_flag = false;
            acquire_GPS_counter = 0;
          }
          acquire_GPS_counter = acquire_GPS_counter  + 1;
        }
        else
        {
          take_off(&drone, &pid_x, &pid_y, &pid_z, &pid_yaw);
        }
        if (mission.take_off_result == false)
        {
          break;
        }
      break;
      
      case 1:
      
    //PER TEST FUORI *************************************** ELIMINAre
  /*    if (acquire_GPS_flag == true)
        {
          //SETTING HOME POSITION 
          mission.HOME_gps_location = drone.current_gps;
          mission.HOME_local_position = drone.current_local_pos;
          mission.start_gps_location = mission.HOME_gps_location ;
          mission.start_local_position =mission.HOME_local_position;

          cout << " mission.HOME_gps_location.lat: "<< mission.HOME_gps_location.latitude << endl;
          cout << " mission.HOME_gps_location.lon: "<< mission.HOME_gps_location.longitude << endl;
          if (acquire_GPS_counter > 20)
          {
            acquire_GPS_flag = false;
            acquire_GPS_counter = 0;
          }
          acquire_GPS_counter = acquire_GPS_counter  + 1;
        }
        //*****************************************************************
        else
        {
    */    
          //Align Yaw with starting point after take off
          align_Yaw_with_starting_GPS_waypoint(&drone,&pid_z, &pid_yaw);
    //    }
      break;
     
      case 2: 
          reaching_starting_position(&drone,&pid_x, &pid_y, &pid_z, &pid_yaw);
          
      break;

      case 3:
          
          if (case_3_counter == 0)
          {
            drone.release_control(); 
            if (!obtain_control_result)
            {
              ROS_ERROR("Impossible to Release Drone Control by OSDK. Exiting.");
            }
            else
            {
              ROS_INFO("Drone Control Released! ----> Place Drone correctly on the first Solar Array");
              
            }
          
          }
          ROS_INFO("Drone Control Released! ----> Place Drone correctly on the first Solar Array");
          obtain_control_result = true;

          if (case_3_counter > 200)
          {
            bool obtain_control_result = drone.obtain_control(); 
            if (!obtain_control_result)
            {
              ROS_ERROR("Impossible to Obtain Drone Control by OSDK. Exiting.");
              //return 1;
            }
            else
            {
              ROS_INFO("Drone Control Obtained!");
              obtain_control_result = false;
              //For safety reason, redeclaration of all the flags required for navigation
              mission.target_point = 2;
    
              mission.inbound_counter = 0;
              mission.target_reached = false;
              mission.breaking_counter = 0;
              //Initialize Drone Position --> Sort of transformation respect world frame placed in HOME POINT
              //The yaw evaluation is done automatically by the drone.
              //Save Last Z altitude
              drone->last_position_z = drone->current_local_pos.z;
              mission.start_gps_location = drone->current_gps;
              mission.start_local_position = drone->current_local_pos;
              mission.referred_local_yaw = drone-> yaw_in_Rad_sim; //Salvo lo yaw del drone nel nuovo local frame in modo da poter avere uno yaw di riferimento per le trasformazioni
             // waypoints.counter = waypoints.counter + 1; //decommentare solo se si arriva diretti a questo caso d atake off 


              mission.state = 4;
            }
          }
          case_3_counter = case_3_counter + 1;
        break;

      case 4:
           obtain_control_result = false;
          /*Navigation WIth KF */
           navigation(&drone, &Kalman_Filter, &pid_x, &pid_y, &pid_z, &pid_yaw);
                      // //Reaching end of panel point P2 from point P1
            //    navigation(&panel, &pid_x, &pid_y, &pid_z, &pid_yaw);
            // break;

            // case 2:
            // //Cambio di vela 
            //    jump_panels_array(&panel,  &pid_x, &pid_y, &pid_z, &pid_yaw);
      break;

      case 5:
          /*Jumping between panels*/
          jump_panels(&drone, &pid_x, &pid_y, &pid_z, &pid_yaw);
          break;
      
      case 6:
           /*Test OUTDOOR */
           if (acquire_GPS_flag == true)
           {
             //SETTING HOME POSITION 
             mission.HOME_gps_location = drone.current_gps;
             mission.HOME_local_position = drone.current_local_pos;
             mission.start_gps_location = mission.HOME_gps_location ;
             mission.start_local_position =mission.HOME_local_position;
   
             cout << " mission.HOME_gps_location.lat: "<< mission.HOME_gps_location.latitude << endl;
             cout << " mission.HOME_gps_location.lon: "<< mission.HOME_gps_location.longitude << endl;
             if (acquire_GPS_counter > 20)
             {
               acquire_GPS_flag = false;
               acquire_GPS_counter = 0;
             }
             acquire_GPS_counter = acquire_GPS_counter  + 1;
             mission.state = 5;
             }
        //*****************************************************************
            else
            {
             mission.state = 1;
            }
      break;

    }
    
//----  ALTITUDE CONTROL BY IMAGES -------- //
     //Publish KF INIT FLAG for reset of the altitude width array 
    std_msgs::Bool KF_initialization;
    if  (mission.panel_array_initialization == true )
    {
        mission.flag_altitude_init = true;

       
    }
    
    if (mission.flag_altitude_init == true && control_altitude_by_image_flag == true)
    {
        cout<< "Initialize PIXEL OFFSET for altitude control " << counter_altitude_init << endl;
        if (counter_altitude_init > 200) 
        {
            mission.flag_altitude_init = false;
            mission.height_above_ground = true;
            counter_altitude_init  = 0;
        }
           
        counter_altitude_init = counter_altitude_init + 1;
    }
     
    KF_initialization.data =  mission.flag_altitude_init;
    drone.publish_KF_init(KF_initialization);








     
     outFile1 << drone.current_local_velocity.vector.x <<"\n";
     outFile1 << drone.current_local_velocity.vector.y <<"\n";
     outFile3 << drone.x_vel_cmd << "\n";
     outFile4 << drone.y_vel_cmd << "\n";
     outFile5 << drone.control_x_coo<< "\n";
     outFile6 << drone.control_y_coo<< "\n";
     outFile7 << drone.xh_[0]<< "\n";
     outFile8 << drone.xh_[1]<< "\n";
     outFile9 << drone.obs[0]<< "\n";
     outFile10 << drone.obs[1]<< "\n";
     outFile11 << drone.yh_[0] << "\n";
     outFile12 << drone.yh_[1] << "\n";
     outFile13 << drone.current_local_pos.x <<"\n";
     outFile14 << drone.current_local_pos.y <<"\n";
     outFile15 << mission.x_target <<"\n";
     outFile16 << mission.y_target <<"\n";
     outFile17 <<drone.x_target<<"\n";
     outFile18 << drone.y_target << "\n";
     outFile19 <<   drone.obs_thermo_GF[0] << "\n";
     outFile20 <<   drone.obs_thermo_GF[1] << "\n";
     outFile21 <<   drone.obs_RGB_GF[0] << "\n";
     outFile22 <<   drone.obs_RGB_GF[1] << "\n";
     outFile23 << drone.current_local_velocity.vector.z << "\n";
   


     


     
     


     

      cout<<"                                   ##### TELEMETRY ####                                      "<<endl;
      cout <<" X_W : " <<  drone.current_local_pos.x  << " Y_W: "<< drone.current_local_pos.y << " Z_W:  " << drone.current_local_pos.z <<endl;
      cout <<" X_W_new : " <<  drone.new_current_local_pos.x  << " Y_W_new: "<< drone.new_current_local_pos.y << " Z_W_new:  " << drone.new_current_local_pos.z <<endl;
      cout << "YAW DES: " << drone.yaw_des_rad_sim << " YAW: " << drone.yaw_in_Rad_sim<< endl;
     // cout <<"target_x : " <<  mission.target_GPS_offset_x << " target_y: " << mission.target_GPS_offset_y << " target_z: " << mission.target_GPS_offset_z << endl;
      cout<< "Cartesian Distance to NEXT GPS WAYPOINT : " <<  mission.cartesian_distance_2D << endl;
      cout<<"                                   ####################                                             " <<endl;
      

      //Publish To gezebo Simm
      drone.publish_new_current_local_position(mission.end_panel_reached);
      //Publish flag to script detection python for define the user control
      drone.publish_obtain_control_flag(obtain_control_result);


      drone.flagDroneRGBControlPoint1 = false; 
      drone.flagDroneRGBControlPoint2 = false;
      drone.flagDroneThermoControlPoint1 = false;
      drone.flagDroneThermoControlPoint2 = false;
      drone.flagDroneAttitude = false;
      drone.flagDroneGPSpos = false;
      drone.flagDroneLocalpos = false;
      drone.flagDroneLocalvel = false;
      drone.flagDroneAngularVel = false;
      drone.flagAltitudeOffset = false;
      drone.flagUltrasonic = false;


      ros::spinOnce();
      r.sleep();
   } 
    
  //  // Kalman_Filter.Kalman_Filter_calculate(0.0, 0.0, 0.0, 0.0,  0.0 ,  0.0 , 0.0 ,  0.0);
      
    
     
    

  //   ros::spinOnce();
  //   r.sleep();
  // }
  return 0;
}
