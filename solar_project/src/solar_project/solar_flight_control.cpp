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
#include <iostream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <sys/stat.h> 

#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/mission.h"
#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/image_converter.h"

#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/KF.h"
#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/pid.h"
#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/Drone.h"

//############ SOLO TEST 
#include <chrono>
using namespace std::chrono;
//#############################

#define HOR_ROLL_PITCH_ANGLE 0x00
#define HORIZONTAL_VELOCITY 0x40
#define HOR_POSITION_OFFSET 0x80
#define HOR_ANGULAR_RATES 0xC0

#define VERTICAL_VELOCITY 0x00
#define VER_ALTITUDE 0x10
#define VER_THRUST 0x20

#define YAW_ANGLE 0x00
#define YAW_RATE 0x08

#define COORD_GROUND_FRAME 0x00
#define HORIZONTAL_BODY 0x02

#define BRAKE_NO 0x00
#define STABLE_ENABLE 0x01

//#include <dji_sdk/dji_sdk_node.h>

using namespace dji_osdk_ros;
using namespace std;

const float deg2rad = C_PI / 180.0;
const float rad2deg = 180.0 / C_PI;

int precision = std::numeric_limits<double>::max_digits10;

Mission mission;

dji_osdk_ros::SetupCameraStream setupCameraStream_;

//WRITE TXT FILE FOR DEBUtest_GPS_coo_rototraslated

// /home/dji/DATA/dji_ws/simulation_data/test_Predosa/TEST_2/Exp_1/


struct Waypoint_GPS
{

  //Coordinata punto 1 (riferimento) del GPS. Da inizializzare in main con coordinata P1_x P1_y del primo pannello
  double GPS1_x = 0.0;
  double GPS1_y = 0.0;

  vector<double> GPS_waypoints_lat;
  vector<double> GPS_waypoints_lon;

  //Per test su singolo array utilizzo due waypoints fissi contenuti in questo array
  vector<double> GPS_fixed_waypoints_lat;
  vector<double> GPS_fixed_waypoints_lon;

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

  //

  vector<double> distance_from_GPS_coo;
  vector<double> angle_vector;

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
  float lat_1 = delta_x / C_EARTH + (lat_0 * deg2rad);
  lat_1 = lat_1 * rad2deg;
  cout << "lat1: " << lat_1 << endl;

  // waypoints.GPS_waypoints_lat.push_back(lat_1);
  // waypoints.GPS_waypoints_lon.push_back(lon_0);

  delta_y = 22;
  float lon_2 = (delta_y / C_EARTH) * 1 / cos(lat_0 * deg2rad) + lon_0 * deg2rad;
  lon_2 = lon_2 * rad2deg;
  cout << "lon2: " << lon_2 << endl;

  // waypoints.GPS_waypoints_lat.push_back(lat_1);
  // waypoints.GPS_waypoints_lon.push_back(lon_2);

  std::array<double, 2> lat{44.611515, 44.611515}; // 45.557738, 9.125081, 45.558037, 9.124881
  std::array<double, 2> lon{8.859281, 8.859559059143066};

  for (int i = 0; i < lat.size(); i++)
  {
    waypoints.GPS_waypoints_lat.push_back(lat[i]);
    waypoints.GPS_waypoints_lon.push_back(lon[i]);
  }

  for (int i = 0; i < waypoints.GPS_waypoints_lat.size(); i++)
  {

    cout << std::setprecision(precision) << "  waypoints.GPS_waypoints_lat: " << waypoints.GPS_waypoints_lat[i] << endl;
    cout << std::setprecision(precision) << "   waypoints.GPS_waypoints_lon: " << waypoints.GPS_waypoints_lon[i] << endl;
  }
}

/*This functiuon loads only two fixed GPS coordinates for the isnpectuion test of a singlke solar array.
When the conrol is released from the drone and re-acquired by the user, two new waypoints are selected,
based on the positions ofn the two fixewd waypoints.
The waypoints P1 is the drone positioon, the waypoints P2 is placed knowing the longituide adn latituide offset between the 
two fixed waypoints. The direction is decided by the yaw drone orientation.
*/

void load_only_two_fixed_GPS_coordinates(vector<double> lat, vector<double> lon)
{
 
   
  // //P2_lon > P1_lon sempre!!!!
  // std::array<double, 2> lat{44.7810029, 44.780996}; //44.781715, 44.781720,
  // std::array<double, 2> lon{8.639929, 8.641029};    //8.640793, 8.640425,

  for (int i = 0; i < lat.size(); i++)
  {
    waypoints.GPS_fixed_waypoints_lat.push_back(lat[i]);
    waypoints.GPS_fixed_waypoints_lon.push_back(lon[i]);
    cout << "  waypoints.GPS_fixed_waypoints_lat: " << waypoints.GPS_fixed_waypoints_lat[i] << endl;
    cout << "  waypoints.GPS_fixed_waypoints_lon: " << waypoints.GPS_fixed_waypoints_lon[i] << endl;
  }

  // for (int i = 0; i < waypoints.GPS_fixed_waypoints_lat.size(); i++)
  // {

  //   cout << std::setprecision(precision) << "  waypoints.GPS_fixed_waypoints_lat: " << waypoints.GPS_fixed_waypoints_lat[i] << endl;
  //   cout << std::setprecision(precision) << "   waypoints.GPS_fixed_waypoints_lon: " << waypoints.GPS_fixed_waypoints_lon[i] << endl;
  // }
}

void load_GPS_coordinates()
{
  vector<double> ReplayBuffer;
  std::ifstream infile; //creates stream myFile

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

  //Predosa
  // {44.780000, 44.780003, 44.780068, 44.780068}
  // {8.641377,  8.641078, 8.641071, 8.641406}

  // std::array<double, 4> lat{44.611174,44.611230, 44.611106,44.611064};// 45.557738, 9.125081, 45.558037, 9.124881
  // std::array<double, 4> lon{ 8.860654,  8.859839, 8.859753, 8.860611};

  // std::array<double, 4> lat{44.781004, 44.780999, 44.781079, 44.781074};
  // std::array<double, 4> lon{8.639930, 8.641020, 8.640976, 8.639906};

  //Test WAYPOINTS PREDOSA---> Vele centrali reali waypoints

  std::array<double, 4> lat{ 44.781003, 44.781003, 44.781063, 44.781066}; //44.781715, 44.781720,
  std::array<double, 4> lon{ 8.639930, 8.641033,8.641009,8.639904}; //8.640793, 8.640425,

  //#########################################################################################################à
  //Test WAYPOINTS PREDOSA ---> Vele centrali  waypoints Ruotati

  //-------> Traslation tx = 1.0 ty = 0.8 and rotation 0.1 rad
  // std::array<double, 6> lat{ 44.7810029, 44.78112798,  44.78117141,    44.78106554,  44.780996, 44.781066}; //44.781715, 44.781720,
  // std::array<double, 6> lon{ 8.639929, 8.640984,   8.640976,    8.639905,    8.641022, 8.639914}; //8.640793, 8.640425,


  // Rotation 0.15 rad


  // std::array<double, 6> lat{44.7810029, 44.78112079, 44.78116423, 44.78105835, 44.780996, 44.781066}; //44.781715, 44.781720,
  // std::array<double, 6> lon{8.639929, 8.64097144,  8.64096335, 8.6398933, 8.641032, 8.639914};        //8.640793, 8.640425,

  //#####################################################################################################à

  //Waypoint Predosa prova

  // std::array<double, 8> lat{ 44.7810029, 44.781000, 44.781065, 44.78105958,44.781198, 44.781197, 44.780996,  44.781066};
  // std::array<double, 8> lon{ 8.639929, 8.640180, 8.640178, 8.63989724, 8.639893, 8.640160,  8.641022, 8.639914}; //8.640793, 8.640425,
  // std::array<double, 12> lat{44.781201, 44.781194, 44.781127, 44.781134, 44.781068, 44.781061, 44.781069, 44.781005, 44.780996, 44.781003, 44.780936, 44.780933};
  // std::array<double, 12> lon{8.639860, 8.640949, 8.640982, 8.639875, 8.639912, 8.641003, 8.639906, 8.639932, 8.641037, 8.639930, 8.639961, 8.641057};

  //CASO LIMITE __> coo:45.558036, 9.124888, 45.557800, 9.124901
  // std::array<double, 2> lat{45.558028, 45.558364};
  // std::array<double, 2> lon{9.125616,9.125379};

  for (int i = 0; i < lat.size(); i++)
  {
    waypoints.GPS_waypoints_lat.push_back(lat[i]);
    waypoints.GPS_waypoints_lon.push_back(lon[i]);
  }

  for (int i = 0; i < waypoints.GPS_waypoints_lat.size(); i++)
  {

    cout << std::setprecision(precision) << "  waypoints.GPS_waypoints_lat: " << waypoints.GPS_waypoints_lat[i] << endl;
    cout << std::setprecision(precision) << "   waypoints.GPS_waypoints_lon: " << waypoints.GPS_waypoints_lon[i] << endl;
  }
}

//################### Evaluate differences between target coordinate and drone coordinates ###########
void localOffsetFromGpsOffset(geometry_msgs::Vector3 &deltaNed,
                              sensor_msgs::NavSatFix &target,
                              sensor_msgs::NavSatFix &origin)
{

  // ROS_INFO("##### target.longitude %f ....", target.longitude);

  // ROS_INFO("##### origin.latitude %f ....", origin.latitude);
  // ROS_INFO("##### origin.longitude %f ....", origin.longitude);

  //Calcolo offset tra posizone iniziale e attuale drone
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  //Ottengo Offset in metri dato offset in latitudine e longitudine
  deltaNed.x = deltaLat * deg2rad * C_EARTH;
  deltaNed.y = deltaLon * deg2rad * C_EARTH * cos(deg2rad * target.latitude);
  deltaNed.z = target.altitude - origin.altitude; //--> Target altitude è l'altuitudine attuale del drone
  //ROS_INFO("##### target.altitude:  %f , origin.altitude : %f....",  target.altitude,  origin.altitude);

  // cout<<"[localOffsetFromGpsOffset]  deltaNed.x: " <<  deltaNed.x<< endl;
  // cout<<"[localOffsetFromGpsOffset]  deltaNed.y:" <<  deltaNed.y << endl;
}

//############### Publish Position /Velocity Command to drone #############
void publish_command_position_to_drone(Drone *drone)
{
  bool over_borders = false;
  //Saturation
  //   mission.speedFactor         = 1;
  //   if (abs(mission.z_offset_remaining) >=   mission.speedFactor)
  //   drone->z_pos_cmd = (mission.z_offset_remaining>0) ?   mission.speedFactor : -1 *   mission.speedFactor;
  //   else
  //  //Il comando sulla x è relativo all'offset remaining
  //   drone->z_pos_cmd  = mission.start_local_position.z + mission.target_offset_z;
  if (mission.inNavigation == true)
  {

    
    if (mission.start_gps_location.longitude < mission.P2_lon)
    {
      float diff = drone->current_gps.longitude - mission.P2_lon;
      
      if (diff > 0)
      {
        over_borders = true;
      }
    }
    else
    {
      float diff = drone->current_gps.longitude - mission.P2_lon;
      if (diff < 0)
      {
        over_borders = true;
      }
    }
  }
  if (mission.cartesian_distance_2D < mission.distance_to_target && mission.state > 1 && mission.state != 1 || mission.target_reached == true || over_borders == true)
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
  bool over_borders = false;
  //Saturation
  //   mission.speedFactor         = 1;
  //   if (abs(mission.z_offset_remaining) >=   mission.speedFactor)
  //   drone->z_pos_cmd = (mission.z_offset_remaining>0) ?   mission.speedFactor : -1 *   mission.speedFactor;
  //   else
  //  //Il comando sulla x è relativo all'offset remaining
  //   drone->z_pos_cmd  = mission.start_local_position.z + mission.target_offset_z;
  if (mission.inNavigation == true)
  {

    cout << "  mission.P2_lon: " << mission.P2_lon << endl;
    if (mission.start_gps_location.longitude < mission.P2_lon)
    {
      float diff = drone->current_gps.longitude - mission.P2_lon;
      cout << "-----------> difference between GPS coo: " << diff << endl;
      if (diff > 0)
      {
        over_borders = true;
      }
    }
    else
    {
      float diff = drone->current_gps.longitude - mission.P2_lon;
      if (diff < 0)
      {
        over_borders = true;
      }
    }
  }


  if (mission.cartesian_distance_2D < mission.distance_to_target && mission.state > 1 && mission.state != 1 || mission.target_reached == true || over_borders == true)
  {
    drone->x_vel_cmd = 0.0;
    drone->y_vel_cmd = 0.0;
    mission.target_reached = true;
    mission.breaking_counter = mission.breaking_counter + 1;
  }

  //Check also via borders

  sensor_msgs::Joy ctrlVelYaw;

  // uint8_t flag = (Control::VERTICAL_VELOCITY   |
  //             Control::HORIZONTAL_VELOCITY |
  //             Control::YAW_RATE          |    //YAW_RATE o YAW_ANGLE
  //             Control::HORIZONTAL_BODY   |     // HORIZONTAL_BODY rispetto frame locale
  // 									     // HORIZONTAL_GROUND rispetto al body frasme
  //             Control::STABLE_ENABLE);

  uint8_t flag = (VERTICAL_VELOCITY |
                  HORIZONTAL_VELOCITY |
                  YAW_RATE |        //YAW_RATE o YAW_ANGLE
                  HORIZONTAL_BODY | // HORIZONTAL_BODY rispetto frame locale
                  // HORIZONTAL_GROUND rispetto al body frasme
                  STABLE_ENABLE);
  ctrlVelYaw.axes.push_back(drone->x_vel_cmd); //drone->x_vel_cmd
  ctrlVelYaw.axes.push_back(drone->y_vel_cmd);
  ctrlVelYaw.axes.push_back(drone->z_vel_cmd);
  ctrlVelYaw.axes.push_back(drone->yaw_rate_cmd); //drone->yaw_rate_cmd); //drone->yaw_des_rad
  ctrlVelYaw.axes.push_back(flag);
  drone->publish_cmd_velocity(ctrlVelYaw);

  // cout <<"drone->x_vel_cmd : " <<drone->x_vel_cmd  << endl;
  // cout <<"drone->y_vel_cmd : " <<drone->y_vel_cmd  << endl;
  // cout <<"drone->z_vel_cmd : " <<drone->z_vel_cmd  << endl;
  // cout <<"drone->yaw_des_rad: " <<drone->yaw_des_rad  << endl;
}

void evaluate_local_offset_between_GPS_target_and_body_frame(double target_lat, double target_lon, sensor_msgs::NavSatFix &drone_curr_GPS,
                                                             geometry_msgs::Point drone_curr_local_pos, geometry_msgs::Vector3 &target_offset_respect_drone_position)
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
  double delta_y = deltaLon * deg2rad * C_EARTH * cos(deg2rad * target_lat);
  double delta_z = mission.target_offset_z - drone_curr_local_pos.z;

  //NB--> Verificare sempre posizionamento frame GPS
  target_offset_respect_drone_position.x = delta_x; //drone_curr_local_pos.x + delta_x;
  target_offset_respect_drone_position.y = delta_y; //drone_curr_local_pos.y + delta_y;
  target_offset_respect_drone_position.z = delta_z; //drone_curr_local_pos.z + delta_z;
  // cout<< "target_offset_respect_drone_position.x: " <<  target_offset_respect_drone_position.x << endl;
  // cout<< "target_offset_respect_drone_position.y: " << target_offset_respect_drone_position.y << endl;
  // cout<< "target_offset_respect_drone_position.z: " <<  target_offset_respect_drone_position.z << endl;
}

void evaluate_local_offset_between_GPS_target_and_local_frame(double target_lat, double target_lon, sensor_msgs::NavSatFix &drone_curr_GPS,
                                                              geometry_msgs::Point drone_curr_local_pos,
                                                              geometry_msgs::Vector3 &target_offset_respect_local_position)
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
  double deltaLat = target_lat - mission.start_gps_location.latitude;

  double delta_x = deltaLat * deg2rad * C_EARTH;
  double delta_y = deltaLon * deg2rad * C_EARTH * cos(deg2rad * target_lat);
  double delta_z = mission.target_offset_z - 0;

  //NB --> Verifica se il frame GPS è allineato sempre con asse x verso NORD

  target_offset_respect_local_position.y = delta_x;
  target_offset_respect_local_position.x = delta_y;
  target_offset_respect_local_position.z = delta_z;

  // cout<< " target_offset_respect_local_position.x: " <<  target_offset_respect_local_position.x << endl;
  // cout<< "target_offset_respect_local_position.y: " << target_offset_respect_local_position.y << endl;
  // cout<< " target_offset_respect_local_position.z: " <<  target_offset_respect_local_position.z << endl;
}

void evaluate_local_offset_between_GPS_target_and_HOME_frame(double target_lat, double target_lon,
                                                             geometry_msgs::Vector3 &target_offset_respect_HOME_position)
{
  double deltaLon = target_lon - mission.HOME_gps_location.longitude;
  double deltaLat = target_lat - mission.HOME_gps_location.latitude;

  double delta_x = deltaLat * deg2rad * C_EARTH;
  double delta_y = deltaLon * deg2rad * C_EARTH * cos(deg2rad * target_lat);
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
  double start_lat = 0.0;
  double start_lon = 0.0;
  if (mission.enter_state == 4)
  {
    //Se arrivo qui da navigazione allora mi devo allineare con inizio pannello precedente
    //Sostanzialmente ruotare di 180 gradi al fine i raggiungere il waypoint sucessivo
    start_lat = waypoints.GPS_waypoints_lat[waypoints.counter - 2];
    start_lon = waypoints.GPS_waypoints_lon[waypoints.counter - 2];
  }
  else
  {
    
  
    start_lat = waypoints.GPS_waypoints_lat[waypoints.counter];
    start_lon = waypoints.GPS_waypoints_lon[waypoints.counter];
  
  }

  /*Evaluate Yaw Desired Angle to align drone with GPS waypoint 
  NB: Se waypoints_counter = 0 si intende il waypoint di start sul primo pannello
  */

  geometry_msgs::Vector3 target_offset_respect_drone_position_referred_to_local_position;

  evaluate_local_offset_between_GPS_target_and_body_frame(start_lat, start_lon, drone->current_gps, drone->current_local_pos, target_offset_respect_drone_position_referred_to_local_position);

  drone->yaw_des_rad = atan2(target_offset_respect_drone_position_referred_to_local_position.x, target_offset_respect_drone_position_referred_to_local_position.y);
 
  //float new_yaw_des_rad = drone->yaw_des_rad;

  if (drone->yaw_des_rad <= +C_PI && drone->yaw_des_rad > -C_PI / 2)
  {
    drone->yaw_des_rad_sim = -drone->yaw_des_rad + C_PI / 2;
  }
  else
  {
    drone->yaw_des_rad_sim = -C_PI - drone->yaw_des_rad - C_PI / 2;
  }
  //drone->yaw_des_rad = new_yaw_des_rad;
}

float change_yaw_for_translation_between_locals_frame(Drone *drone)
{
  float yaw_for_rot = 0.0;
  if (drone->yaw_in_Rad_sim > -C_PI && drone->yaw_in_Rad_sim <= C_PI / 2)
  {
    yaw_for_rot = drone->yaw_in_Rad_sim + C_PI / 2;
  }
  else
  {
    yaw_for_rot = -C_PI + (drone->yaw_in_Rad_sim - C_PI / 2);
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
  y1_w = drone->xh_[0] * x1_w + drone->xh_[1];
  y2_w = drone->xh_[0] * x2_w + drone->xh_[1];
  //cout<<"[KF EXP FUNC] KF RGB estimation exported for control Point Evaluation" << endl;
  //cout<<"[KF EXP FUNC] y1_w: " <<y1_w << ", "<< "y2_w: " << y2_w << endl;

  //ROtate both points in body frame
  geometry_msgs::Point curr_local_pos;

  curr_local_pos.x = drone->new_current_local_pos.x;
  curr_local_pos.y = drone->new_current_local_pos.y;

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
  a_b = ((y2_b - y1_b) / (x2_b - x1_b));
  c_b = ((-(a_b)*x1_b) + y1_b);

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
  float alfa = acos(x_target_body / i);
  if (y_target_body < 0)
  {
    alfa = -1 * alfa;
  }
  mission.x_target = setpoint_distance * cos(alfa);
  mission.y_target = setpoint_distance * sin(alfa);
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
  new_curr_local_position.y = drone->new_current_local_pos.y;
  yaw_for_rot = change_yaw_for_translation_between_locals_frame(drone);
  drone->Rotation_local_GF_to_BF(new_curr_local_position, 0, 0, drone->yaw_in_Rad);
  x_target_P1_body = drone->check_x_b;
  y_target_P1_body = drone->check_y_b;
  //Take coordinate of point P2 rotated respect drone body frame

  x_target_P2_body = drone->target_offset_respect_local_position.x; //Gia espresso in Local
  y_target_P2_body = drone->target_offset_respect_local_position.y;

  drone->Rotation_local_GF_to_BF(new_curr_local_position, x_target_P2_body, y_target_P2_body, drone->yaw_in_Rad);
  x_target_P2_body = drone->check_x_b;
  y_target_P2_body = drone->check_y_b;

  cout << "[GPS BACKGROUND LINE]  BACKGROUND GPS LINE!" << endl;
  //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0 nel BODY FRAME
  a = ((y_target_P2_body - y_target_P1_body) / (x_target_P2_body - x_target_P1_body));
  c = ((-(a)*x_target_P1_body) + y_target_P1_body);

  //Find distance point line: point is the body Origin and line is the line r
  double e = (abs(c) / (sqrt(pow(a, 2) + pow(b, 2))));
  waypoints.error_from_GPS_line = e;


  cout << "[GPS BACKGROUND LINE] POINT LINE ERROR: " << waypoints.error_from_GPS_line << endl;
}

void evaluate_control_point(Drone *drone, bool from_image, bool GPS_background_line_control, bool KF_init)
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
  new_curr_local_position.y = drone->new_current_local_pos.y;
  yaw_for_rot = change_yaw_for_translation_between_locals_frame(drone);
  drone->Rotation_local_GF_to_BF(new_curr_local_position, 0, 0, drone->yaw_in_Rad);
  x_target_P1_body = drone->check_x_b;
  y_target_P1_body = drone->check_y_b;

  //Take coordinate of point P2 rotated respect drone body frame
  x_target_P2_body = drone->target_offset_respect_local_position.x; //Gia espresso in Local
  y_target_P2_body = drone->target_offset_respect_local_position.y;

  drone->Rotation_local_GF_to_BF(new_curr_local_position, x_target_P2_body, y_target_P2_body, drone->yaw_in_Rad);
  x_target_P2_body = drone->check_x_b;
  y_target_P2_body = drone->check_y_b;

  if (from_image == false)
  {

    cout << "[EVALUATE CONTROL LINE] #### FOLLOWING GPS LINE!" << endl;

    //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0 nel BODY FRAME
    a = ((y_target_P2_body - y_target_P1_body) / (x_target_P2_body - x_target_P1_body));
    c = ((-(a)*x_target_P1_body) + y_target_P1_body);
    cout << "[EVALUATE CONTROL LINE] a BODY GPS: " << a << " c BODY GPS: " << c << endl;
  }
  else
  {
    cout << "                                                                " << endl;
    cout << "[EVALUATE CONTROL LINE] ##### FOLLOWING VISION LINE!" << endl;

    a = drone->xh_body[0];
    c = drone->xh_body[1];
    
    // a =  drone->obs_RGB_GF[0];
    // c = drone->obs_RGB_GF[1];
    cout << "[EVALUATE CONTROL LINE] a BF VISION: " << a << " c BF VISION: " << c << endl;
  }

  //Find distance point line: point is the body Origin and line is the line r
  double e = (abs(c) / (sqrt(pow(a, 2) + pow(b, 2))));
  mission.error_from_vision_line = e;
  //waypoints.error_from_GPS_line = e;
  //Define vector Vx starting from body frame origin and parallel to the line r
  double Vx[2] = {1 / a, -1 / b};
  
  //###########################################
  //The value of Kx varies depending on the desired velocity along the x direction in navigation 
  double Kx = 1.9;  //Coefficiente moltiplicativo del vettore parallelo alla retta r
  if ( mission.enable_navigation_velocity_control_flag == true)
  {
    /*
    IMPORTANTISSIMO PER LA DEFINIZIONE DELLA VELOCITà DSIERATA  
    */
    //Nel caso stia considerando il controllo in velocità, pongo il fattore moltiplicativo del vettore unitario parallelo alla retta
    // uguale alla velocita lineare desiderata di navigazione 
     if (drone->flagUserDesiredVelocity == true)
     {
       drone->sat_x_vel = drone->desired_velocity;
     }
     //Il valore di Kx deve essere sempre positivo perche espresso su asse x del body frame del drone.
     Kx =  drone->sat_x_vel;
  }
  

  
  double Vx_norm[2] = {Kx * ((1 / (sqrt(pow(Vx[0], 2) + pow(Vx[1], 2)))) * Vx[0]), Kx * ((1 / (sqrt(pow(Vx[0], 2) + pow(Vx[1], 2)))) * Vx[1])};
  //cout<<"target_point: "<< target_point<<endl;
  float rot_Vx = 0.0;
  float rot_Vy = 0.0;

  if (mission.target_point == 1)
  {
    //punto di target è P1:
    if (x_target_P1_body >= 0 && Vx_norm[0] < 0)
    {

      rot_Vx = Vx_norm[0] * cos(C_PI) - Vx_norm[1] * sin(C_PI);
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
    if (x_target_P2_body >= 0 && Vx_norm[0] < 0)
    {
      rot_Vx = Vx_norm[0] * cos(C_PI) - Vx_norm[1] * sin(C_PI);
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
   

   //Aggiungere la possibilita di cambiare solo per test la velocita desiderata su Y
   // Salvare velocit drone e velocita desiderata raggiunta 
  


  //Define vector Vy starting from body frame origin and perpendicular to line r
  //Il vettore Vy è moltiplicato pe run guadagno Ky e anche per l'errore dato dalla distanza punto retta e, la quale deve tendere a zero

  float Ky = 1; //Coefficiente moltiplicativo del vettore perp alla retta r
  // if ( mission.enable_navigation_velocity_control_flag == true)
  // {
  //   /*
  //   IMPORTANTISSIMO PER LA DEFINIZIONE DELLA VELOCITà DSIERATA  
  //   */
  //   //Nel caso stia considerando il controllo in velocità, pongo il fattore moltiplicativo del vettore unitario parallelo alla retta
  //   // uguale alla velocita lineare desiderata di navigazione 
  //    if (drone->flagUserDesiredVelocity == true)
  //    {
  //      drone->sat_y_vel = drone->desired_velocity_y;
  //    }
  //    //Il valore di Kx deve essere sempre positivo perche espresso su asse x del body frame del drone.
  //    Ky =  drone->sat_y_vel;
  // }
  
  double Vy_norm[2] = {0.0, 0.0};

  double Vy[2] = {1 / b, 1 / a};
  Vy_norm[0] = Ky * e * ((1 / (sqrt(pow(Vy[0], 2) + pow(Vy[1], 2)))) * Vy[0]); // * e
  Vy_norm[1] = Ky * e * ((1 / (sqrt(pow(Vy[0], 2) + pow(Vy[1], 2)))) * Vy[1]); // *e

 

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
  float a2 = 1 / a; //Vy_norm[1]/Vy_norm[0];
  float b2 = -1;
  float c2 = 0;

  /*++++++++++++++Parallelogram rules. Find Vector V = Kx*Vx_norm + Ky*e*Vy_norm ***********/

  //Trovo retta r3 parallela a retta r2 ma passante per il punto B a cui tende vettore Vx
  float a3 = a2;
  float b3 = -1;
  float c3 = -a3 * Vx_norm[0] + Vx_norm[1];

  //Trovo retta r4 parallela a retta r1 ma passante per il punto C a cui tende vettore Vy
  float a4 = a1;
  float b4 = -1;
  float c4 = -a4 * Vy_norm[0] + Vy_norm[1];

  //Trovo punto di intersezioe D, al quale tende il vettore V. Il punto D è il punto di controllo
  float D[2] = {((b4 * c3 - b3 * c4) / (b3 * a4 - b4 * a3)), 0};
  D[1] = (-a3 * D[0] - c3) / b3;
  drone->control_x_coo = abs(D[0]);
  drone->control_y_coo = D[1];

  if  (drone->control_y_coo > 1.5)
  {
      drone->control_y_coo = 0.0;
  } 

  if (drone->control_y_coo < -1.5)
  {
      drone->control_y_coo = 0.0;
  }

  //Only for plot Purposes 
  if (drone->control_y_coo > 0)
  {
    mission.error_from_vision_line = -1*mission.error_from_vision_line;
  }
  //Se le coordinate sono NAN alla prima iterazione le metto 0.0;
  if (drone->control_x_coo != drone->control_x_coo && drone->control_y_coo != drone->control_y_coo)
  {
    drone->control_x_coo = 0.0;
    drone->control_y_coo = 0.0;
  }

  cout << "[EVALUATE CONTROL LINE]  D[0]: " << drone->control_x_coo << " D[1]: " << drone->control_y_coo << endl;

  /*
   Il punto di controllo è espresso nel body frame.
   Necessario ruoptarlo rispetto a frame HOME per avere il target da inviare al controllo. 

  */

  //Place SetPoints Every tot METERS ALONG THE LINE Depending on drone Position --> SETPOINTS placed in BODY FRAME

  float alfa;
  // cout << "[EVALUATE CONTROL LINE] DISTANCE TO LOCAL SETPOINT: " << sqrt(pow(drone->x_target - drone->current_local_pos.x, 2) + pow(drone->y_target - drone->current_local_pos.y, 2)) << endl;
  // if (from_image == true && mission.inbound_counter > 200400)
  // {

  //   if (sqrt(pow(drone->x_target - drone->current_local_pos.x, 2) + pow(drone->y_target - drone->current_local_pos.y, 2)) < 0.8)
  //   {
  //     alfa = target_coo_given_distance_setpoint(mission.line_setpoint_distance, drone->control_x_coo, drone->control_y_coo);
  //     // mission.x_target = drone->control_x_coo + mission.line_setpoint_distance;
  //     // mission.y_target = drone->control_y_coo;

  //     // cout << "[EVALUATE CONTROL LINE] -----> BODY FRAME x_target: " << mission.x_target << endl;
  //     // cout << "[EVALUATE CONTROL LINE] -----> BODY FRAME y_target: " << mission.y_target << endl;

  //     //CHANGE YAW
  //     yaw_for_rot = change_yaw_for_translation_between_locals_frame(drone);
  //     //    //Ruotop punto di controllo dal BODY FRAME al HOME FRAME
  //     drone->Rotation_BF_to_local_GF_des_pos_original(drone->current_local_pos, mission.x_target, mission.y_target, drone->yaw_in_Rad);
  //     drone->x_target = drone->check_x_local;
  //     drone->y_target = drone->check_y_local;
  //   }
  // }
  // else
  // {

    /*
    Rotation Around y axis of local frame:
    X_D^B = -X_D^H
    Y_D^B = Y_D^H
    */

    //SCOMMENTARE PER MUOVERSI SEGUENDO LA RETTA SENZA I SETpOINT
    yaw_for_rot = change_yaw_for_translation_between_locals_frame(drone);
    //cout<<"[EVALUATE CONTROL LINE] yaw_for_rot: " << yaw_for_rot << endl;
    drone->Rotation_BF_to_local_GF_des_pos_original(drone->current_local_pos, drone->control_x_coo, drone->control_y_coo, drone->yaw_in_Rad);

    drone->x_target = drone->check_x_local;
    drone->y_target = drone->check_y_local;
  

  if ( mission.enable_navigation_velocity_control_flag == true)
  {
    //Nel caso stia considerando il controllo in velocità,
    //l'input del controllo sono i vettori lungo la direzione x e y, non il punto considrato 
    //Il controllo è esprsesso nel body Frame 
    drone->x_target = drone->check_x_local ;
    drone->y_target =drone->check_y_local ;
  }
  //  drone->rotate_target_position_from_new_local_to_HOME_local(drone->x_target, drone->y_target );
  //  drone->x_target  = drone->x_target_HOME;
  //  drone-> y_target = drone->y_target_HOME;

  // cout << "[EVALUATE CONTROL LINE] -----> HOME FRAME LOCAL TARGET SETPOINT X : " << drone->x_target << endl;
  // cout << "[EVALUATE CONTROL LINE] -----> HOME FRAME LOCAL TARGET SETPOINT Y : " << drone->y_target << endl;
  // cout << "[EVALUATE CONTROL LINE] -----> inbound_counter: " << mission.inbound_counter << endl;
  // cout << "                                                                                      " << endl;
  //Translate coordiante to LOCAL HOME frame --> referred to current_local_position
  //---> Velocità sono riferite al frame HOME LOCAL
}

//Switch Functions
void take_off(Drone *drone, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
  if (mission.take_off_result == false)
  {
    if (drone->takeoff())
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

    mission.cartesian_distance_2D = sqrt(pow(mission.x_offset_remaining, 2) + pow(mission.y_offset_remaining, 2));
    mission.cartesian_distance_3D = sqrt(pow(mission.target_offset_x - drone->current_local_pos.x, 2) + pow(mission.target_offset_y - drone->current_local_pos.y, 2) + pow(mission.target_offset_z - drone->current_local_pos.z, 2));

    drone->x_pos_cmd = 0;
    drone->y_pos_cmd = 0;
    drone->z_pos_cmd = mission.start_local_position.z + mission.target_offset_z; // = mission.target_offset_z  = 5

    drone->x_vel_cmd = 0;
    drone->y_vel_cmd = 0;
    drone->z_vel_cmd = pid_z->position_control_knowing_velocity(mission.target_offset_z, drone->current_local_pos.z, 0, drone->current_local_velocity.vector.z);
    drone->yaw_des_rad = 0;
    drone->yaw_rate_cmd = 0;

    //publish_command_position_to_drone(drone);
    publish_command_velocity_to_drone(drone);
    cout << "[TAKE OFF] Yaw_drone in RAD : " << drone->yaw_in_Rad << endl;
   
    //Publish to Gazebo to maintain camera fixed in starting psoition
    drone->control_x_coo = 0.0;
    drone->control_y_coo = 0.0;
    drone->publish_control_D_point_to_GAZEBO(mission.height_above_ground ,0.0);
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
    //ros::Duration(1.0).sleep();
    mission.state = 1;
    mission.enter_state = 1;
    waypoints.counter = 0;
  }
}

void rotate_180_degrees_yaw(Drone *drone)
{
  if (drone->yaw_in_Rad < 0.4 && drone->yaw_in_Rad > -0.4)
  {
    drone->yaw_des_rad = M_PI;
  }
  else
  {
    drone->yaw_des_rad = 0;
  }
}

void align_Yaw_with_starting_GPS_waypoint(Drone *drone, PID *pid_z, PID *pid_yaw)
{

  //Ci entra se arriva da navigatiuon
  if (mission.enter_state == 4)
  {
    if (mission.desired_yaw_flag == true)
    {
      rotate_180_degrees_yaw(drone);
      mission.desired_yaw_flag = false;
    }

    mission.setTarget(0, 0, drone->z_des, 0);

    // drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_in_Rad, drone->yaw_des_rad, 0, drone->yaw_vel);
    drone->yaw_err_rad = drone->yaw_in_Rad - drone->yaw_des_rad;
    drone->x_pos_cmd = 0.0;
    drone->y_pos_cmd = 0.0;
    // drone->z_pos_cmd =  drone->z_des; //drone->last_altitude ;
    publish_command_position_to_drone(drone);
  }
  else
  {
    //Evaluate Yaw between Drone Position and GPS waypoint

    evaluate_des_GPS_WAYPOINT_yaw_angle(drone);
    mission.setTarget(0, 0, drone->z_des, 0);
    //evaluate_control_offset();

    drone->yaw_des_rad = drone->yaw_in_Rad_sim;
    drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_in_Rad_sim, drone->yaw_des_rad_sim, 0, drone->yaw_vel);
    drone->yaw_err_rad = drone->yaw_in_Rad_sim - drone->yaw_des_rad_sim;
    //publish_command_velocity_to_drone(drone);
  }

  drone->x_vel_cmd = 0;
  drone->y_vel_cmd = 0;
  drone->z_vel_cmd = pid_z->position_control_knowing_velocity(mission.target_offset_z, drone->current_local_pos.z, 0, drone->current_local_velocity.vector.z);
  publish_command_velocity_to_drone(drone);


  cout << "[YAW ALIGNMENT] vel_z: " <<  drone->current_local_velocity.vector.z << "\n";
  cout << "[YAW ALIGNMENT] drone->yaw_des: " << drone->yaw_des_rad << " drone->yaw: " << drone->yaw_in_Rad << endl;
  cout << "[YAW ALIGNMENsT] drone->yaw_in_Rad_sim: " << drone->yaw_in_Rad_sim << endl;
  cout << "[YAW ALIGNMENT] drone->yaw_err: " << drone->yaw_err_rad << endl;
  cout << "[YAW ALIGNMENT] drone->z_vel_cmd : " << drone->z_vel_cmd  << " mission.target_offset_z : " <<  mission.target_offset_z << endl;

  mission.cartesian_distance_2D = sqrt(pow(mission.x_offset_remaining, 2) + pow(mission.y_offset_remaining, 2));
  if (abs(drone->yaw_err_rad) < 0.05)
  {
    mission.state = mission.enter_state + 1; //Aumenta in automatico per passare allo stato jump panels se arrivo da navigation
                                             //NEl caso di test avanti e inidetro lungo singolo pannello
    mission.start_gps_location = drone->current_gps;
    mission.start_local_position = drone->current_local_pos;
    //ros::Duration(1.0).sleep();
    mission.inbound_counter = 0;
    mission.end_panel_reached = false;
  }
  else
  {
    //Align Yaw
    mission.state = 1; //
  }

  //Publish to Gazebo to maintain camera fixed in starting psoition
  drone->control_x_coo = 0.0;
  drone->control_y_coo = 0.0;
  drone->publish_control_D_point_to_GAZEBO(mission.height_above_ground ,0.0);

  //Ottengo controllo manuale del drone
  if (drone->key_for_control_is_pressed == true)
  {
    mission.inbound_counter = 0;
    mission.target_reached = false;
    mission.flag_navigate_to_waypoints = false;
    mission.breaking_counter = 0;
    mission.panel_array_initialization = true;
    mission.end_panel_reached = true; //Necessario per camera in Gazebo per orientarsi su nuovo pannello
    mission.inNavigation = false;
    //Publish zero control point to ARDRONE
    drone->control_x_coo = 0.0;
    drone->control_y_coo = 0.0;
    //drone->publish_control_D_point_to_GAZEBO();

    drone->last_altitude = drone->current_local_pos.z;//drone->altitude;
    drone->last_position_z = drone->current_local_pos.z;
    drone->z_des =  drone->last_position_z ;
    drone->image_control_count = 0;
    drone->THERMO_image_control_count = 6000;
    drone->RGB_image_control_count = 6000;

    drone->key_for_control_is_pressed = false;
    mission.state = 8;
  }
}

void reaching_starting_position(Drone *drone, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
  /*
  La funzione fa riferimento al frame NED --> asse x verso Nord e asse y verso est
  */

  waypoints.counter = 0;
  if (mission.inbound_counter == 0)
  {
    double start_lat = waypoints.GPS_waypoints_lat[waypoints.counter];
    double start_lon = waypoints.GPS_waypoints_lon[waypoints.counter];

    evaluate_local_offset_between_GPS_target_and_body_frame(start_lat, start_lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_drone_position);
    mission.setTarget(drone->target_offset_respect_drone_position.x, drone->target_offset_respect_drone_position.y, drone->z_des, 0);

    mission.breaking_counter = 0.0;
  }

  evaluate_des_GPS_WAYPOINT_yaw_angle(drone);
  localOffsetFromGpsOffset(drone->local_drone_position, drone->current_gps, mission.start_gps_location);
  mission.x_offset_remaining = mission.target_offset_x - drone->local_drone_position.x;
  mission.y_offset_remaining = mission.target_offset_y - drone->local_drone_position.y;
  mission.z_offset_remaining = mission.target_offset_z - drone->local_drone_position.z; //localOffset è la oposizione attuale del drone rispetto al frame di riferimento dopo la trasformazione da GPS coo

  cout << "[REACHING STARTING POSITION] x_off: " << mission.x_offset_remaining << endl;
  cout << "[REACHING STARTING POSITION] y_off: " << mission.y_offset_remaining << endl;

  mission.cartesian_distance_2D = sqrt(pow(mission.x_offset_remaining, 2) + pow(mission.y_offset_remaining, 2));
  // mission.cartesian_distance_3D = sqrt(pow(mission.target_offset_x - drone->current_local_pos.x,2) + pow(mission.target_offset_y)

  // VELOCITY CONTROL  -->
  /* NB: INVERTO LE COO DI TARGET DI X E Y PERCHE L'OFFSET LO CALCOLA RISPETTO AL FRAME LOCALE, IL QUALE PRESENTA X  E Y INVERTITE RISPETTO A QUELLO OTTENUTO 
TRAMITE LA TRASFORMAZIONE DELLA LAT LON DEL GPS A DELTA X DELTA Y */

  drone->x_vel_cmd = pid_x->position_control_knowing_velocity(mission.target_offset_y, drone->current_local_pos.x, 0, drone->current_local_velocity.vector.x);
  drone->y_vel_cmd = pid_y->position_control_knowing_velocity(mission.target_offset_x, drone->current_local_pos.y, 0, drone->current_local_velocity.vector.y);
  drone->z_vel_cmd = pid_z->position_control_knowing_velocity(drone->z_des, drone->current_local_pos.z, 0, drone->current_local_velocity.vector.z);

  drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_in_Rad_sim, drone->yaw_des_rad_sim, 0, drone->yaw_vel);

  drone->x_pos_cmd = 0.3 * mission.y_offset_remaining;
  drone->y_pos_cmd = 0.3 * mission.x_offset_remaining;
  drone->z_pos_cmd = drone->z_des;
  
  cout << "[REACHING STARTING POSITIONs] drone->z_vel_cmd : " << drone->z_vel_cmd  << " mission.target_offset_z : " <<  mission.target_offset_z << endl;
  cout << "[REACHING STARTING POSITIONs] vel_z: " <<  drone->current_local_velocity.vector.z << "\n";
  // float sat_x_off = 1;
  // float sat_y_off = 1;
  // drone->saturation_offset( drone->x_pos_cmd,  drone->y_pos_cmd, sat_x_off, sat_y_off);
  //publish_command_position_to_drone(drone);
  publish_command_velocity_to_drone(drone);
  mission.distance_to_target = 0.8;
  if (mission.target_reached == true)
  {
    mission.state = 1;       //Allineo Yaw con Waypoint finale
    mission.enter_state = 2; //2 For obtain control  /Permette di passare allo stato Navigation da stato align Yaw
    mission.target_point = 2;

    mission.inbound_counter = 0;
    mission.target_reached = false;
    mission.breaking_counter = 0;
    //Initialize Drone Position --> Sort of transformation respect world frame placed in HOME POINT
    //The yaw evaluation is done automatically by the drone.
    //Save Last Z altitude
    drone->last_altitude = drone->current_local_pos.z; //drone->altitude;
    drone->last_position_z = drone->current_local_pos.z;
    drone->z_des =  drone->last_position_z ; //drone->current_local_pos.z;
    mission.start_gps_location = drone->current_gps;
    mission.start_local_position = drone->current_local_pos;
    mission.referred_local_yaw = drone->yaw_in_Rad_sim; //Salvo lo yaw del drone nel nuovo local frame in modo da poter avere uno yaw di riferimento per le trasformazioni
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
  drone->control_x_coo = 0.0;
  drone->control_y_coo = 0.0;
  drone->publish_control_D_point_to_GAZEBO(mission.height_above_ground ,0.0);
}



//Evaluate Yaw rate for control 
void evaluate_yaw_rate_for_control(Drone *drone, PID *pid_yaw)
{
  if (drone->yaw_des_rad - drone->yaw_in_Rad <= - M_PI)
  {
      drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity( drone->yaw_in_Rad + 2*M_PI, drone->yaw_des_rad, 0, drone->yaw_vel);
      cout << "SONO QUI 1" << endl;
  }
  else if (drone->yaw_des_rad - drone->yaw_in_Rad >= M_PI)
  {
      drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_in_Rad - 2*M_PI, drone->yaw_des_rad, 0, drone->yaw_vel);
      cout << "SONO QUI 2" << endl;
  }
  else
  {
      drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_des_rad , drone->yaw_in_Rad, 0, drone->yaw_vel);
      cout << "SONO QUI 3" << endl;
  }

  // if(drone->yaw_des_rad >= 0 && drone->yaw_in_Rad >= 0)
  // {
  //     drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_des_rad , drone->yaw_in_Rad, 0, drone->yaw_vel);
  // }
  // else if(drone->yaw_des_rad <= 0 && drone->yaw_in_Rad <= 0)
  // {
  //     drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_des_rad , drone->yaw_in_Rad, 0, drone->yaw_vel);
  // }
  // else if (drone->yaw_des_rad >= 0 &&  drone->yaw_in_Rad <= 0 )
  // {
  //   if (drone->yaw_des_rad - drone->yaw_in_Rad >= M_PI)
  //   {
  //      drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_in_Rad - 2*M_PI, drone->yaw_des_rad, 0, drone->yaw_vel);
  //   }
  //   else
  //   {
  //      drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_in_Rad, drone->yaw_des_rad, 0, drone->yaw_vel);
  //   }
       
  // }
  // else 
  // {
  //   if (drone->yaw_des_rad - drone->yaw_in_Rad <= -M_PI)
  //   {
  //      drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_in_Rad + 2*M_PI, drone->yaw_des_rad, 0, drone->yaw_vel);
  //   }
  //   else
  //   {
  //      drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_in_Rad, drone->yaw_des_rad, 0, drone->yaw_vel);
  //   } 
   
  //      //drone->yaw_rate_cmd = -1.0*drone->yaw_rate_cmd;
  // }

  cout << "DRONE YAW RATE CMD : " <<  drone->yaw_rate_cmd  << endl;


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
 




uardare IPAD per magguiiori info
 */



float a_GF = 0.0;
float c_GF = 0.0;
//int counter_altitude_sensor = 0;
void navigation(Drone *drone, KF *Kalman_Filter, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
  float GPS_background_line_control = false;
  float yaw_rot = 0.0;

  //Default
  bool KF_init = false;

  //GPS COO OF P2 WAYPOINT
  double P2_lat = waypoints.GPS_waypoints_lat[waypoints.counter]; //--> P2 end point of the panel, used to initialize KF
  double P2_lon = waypoints.GPS_waypoints_lon[waypoints.counter];

  mission.P2_lat = P2_lat;
  mission.P2_lon = P2_lon;
  //Obtain the new curr drone local position respect the frame placed in point P1

  drone->rotate_drone_position_respect_new_origin_frame(mission.start_local_position);

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
    mission.setGPSTarget(drone->target_offset_respect_HOME_position.x, drone->target_offset_respect_HOME_position.y, drone->z_des, drone->yaw_des_rad_sim);

    //EValuate GPS TARGET WAYPOINT P2 RESPECT DRONE FRAME ORIENTED AS IN HOME POINT

    evaluate_local_offset_between_GPS_target_and_body_frame(P2_lat, P2_lon, drone->current_gps, drone->new_current_local_pos, drone->target_offset_respect_drone_position);
    drone->target_offset_respect_drone_position.x = drone->target_offset_respect_drone_position.y;
    drone->target_offset_respect_drone_position.y = drone->target_offset_respect_drone_position.x;
    //ROTATE POINT P1 RESPECT DRONE BODY FRAME REFERRED TO LOCAL FRAME PLACED IN P1
    //---> BF has a rotation around Y local axis, the yaw start from zero after the rotation

    yaw_rot = change_yaw_for_translation_between_locals_frame(drone);

    drone->Rotation_local_GF_to_BF(new_curr_local_pos, drone->target_offset_respect_local_position.x, drone->target_offset_respect_local_position.y, drone->yaw_in_Rad);
    drone->target_offset_respect_drone_position.x = drone->check_x_b;
    drone->target_offset_respect_drone_position.y = drone->check_y_b;

    //Serve per avere lo yaw allineato con il waypoint di fine pamnello
    // evaluate_des_GPS_WAYPOINT_yaw_angle(drone);

    /*Initialize Kalman Filter 
    KALMAN FILTER initialized with actual position of the drone and the point P2 at end of the panel express in local coordinate respect the new frame placed in point P1 
    */

    Kalman_Filter->Kalman_filter_initialization(0.0, 0.0, drone->target_offset_respect_local_position.x, drone->target_offset_respect_local_position.y);
    Kalman_Filter->pass_to_class_initialization_matrices(mission.Px, mission.R);
    cout << "[NAVIGATION] a GPS: " << Kalman_Filter->Obtain_Kalman_filter_GPS_state()[0] << " c GPS: " << Kalman_Filter->Obtain_Kalman_filter_GPS_state()[1] << endl;
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
  drone->target_offset_respect_drone_position.x = drone->target_offset_respect_drone_position.y; //devo invertirli perche li sto considerando con new_current_local_pos!!!!!!! --> con current_local_pos invece non necesario perche allineati a GPS
  drone->target_offset_respect_drone_position.y = drone->target_offset_respect_drone_position.x;

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
  drone->Rotation_BF_to_local_GF_des_pos_original(drone->new_current_local_pos, drone->control_thermo_point1_x, drone->control_thermo_point1_y, drone->yaw_in_Rad);
  drone->thermo_control_obs_P1_x_local = drone->check_x_local;
  drone->thermo_control_obs_P1_y_local = drone->check_y_local;

  drone->Rotation_BF_to_local_GF_des_pos_original(drone->new_current_local_pos, drone->control_thermo_point2_x, drone->control_thermo_point2_y, drone->yaw_in_Rad);
  drone->thermo_control_obs_P2_x_local = drone->check_x_local;
  drone->thermo_control_obs_P2_y_local = drone->check_y_local;

  //Evaluate line passing trhough observation in GF
  a_GF = (drone->thermo_control_obs_P2_y_local - drone->thermo_control_obs_P1_y_local) / (drone->thermo_control_obs_P2_x_local - drone->thermo_control_obs_P1_x_local);
  c_GF = (-1 * a_GF * drone->thermo_control_obs_P1_x_local) + drone->thermo_control_obs_P1_y_local;

  drone->obs_thermo_GF << a_GF, c_GF;
  // // ###################### Obtain Information from camera RGB and THERMAL camera #####################

  //NB: Frame ardrone in gazebo presenta la y nel senso opposto a quello in DJI
  //---> Le osservazioni positive sulla Y nel BF del drone in Gazebo in realta sono negative ion quelle di DJI
  if (drone->flagDroneThermoControlPoint1 == true && drone->flagDroneThermoControlPoint2 == true && mission.panel_array_initialization == false || drone->THERMO_image_control_count < 500)
  {
    Kalman_Filter->pass_to_KF_class_OBS_in_GF(a_GF, c_GF);
    //Kalman_Filter->Kalman_Filter_calculate(drone->control_thermo_point1_x,  drone-> control_thermo_point1_y,  drone->control_thermo_point2_x ,  drone->control_thermo_point2_y);
    //-----> EKF
    //Necessario Usare yaw_in_Rad_sim perche faccio riferimento al frame current local pose, allineato con il frame HOME con asse x verso est e asse Y verso Nord

    Kalman_Filter->EKF_calculate(drone->control_thermo_point1_x, drone->control_thermo_point1_y, drone->control_thermo_point2_x, drone->control_thermo_point2_y,
                                 drone->new_current_local_pos.x, drone->new_current_local_pos.y, drone->yaw_in_Rad);
    drone->xh_ = Kalman_Filter->Obtain_Kalman_filter_estimated_state();
    drone->obs = Kalman_Filter->Obtain_Kalman_filter_observation();
    drone->yh_ = Kalman_Filter->Obtain_Kalman_filter_estimated_observation();

    cout << "[EKF THERMO  ------->] Observations in BF : a  " << drone->obs[0] << " c: " << drone->obs[1] << endl;
    cout << "[EKF THERMO  ------->] Estimated states: a   " << drone->xh_[0] << " c: " << drone->xh_[1] << endl;
    cout << "[EKF THERMO  ------->] Estimated Observation in BF:  a_BF " << drone->yh_[0] << " c_BF: " << drone->yh_[1] << endl;
    cout << "[EKF THERMO  ------->]  Observations in GF : a  " << a_GF << " c: " << c_GF << endl;
    from_image_Thermo = true;
    from_image = true;

    if (drone->flagDroneThermoControlPoint1 == true && drone->flagDroneThermoControlPoint2 == true && abs(waypoints.error_from_GPS_line) < 15) //drone.image_control_count: permette la guida guidata dal filtro anche quando non sono rileavate piu osservazioni,
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
  drone->Rotation_BF_to_local_GF_des_pos_original(drone->new_current_local_pos, drone->control_RGB_point1_x, drone->control_RGB_point1_y, drone->yaw_in_Rad);
  drone->RGB_control_obs_P1_x_local = drone->check_x_local;
  drone->RGB_control_obs_P1_y_local = drone->check_y_local;

  //cout << "[KALMAN FILTER RGB] P1 X^L: " << drone-> RGB_control_obs_P1_x_local << "  P1 Y^L: " << drone-> RGB_control_obs_P1_y_local << endl;

  drone->Rotation_BF_to_local_GF_des_pos_original(drone->new_current_local_pos, drone->control_RGB_point2_x, drone->control_RGB_point2_y, drone->yaw_in_Rad);
  drone->RGB_control_obs_P2_x_local = drone->check_x_local;
  drone->RGB_control_obs_P2_y_local = drone->check_y_local;

  //Evaluate line passing trhough observation in GF
  a_GF = (drone->RGB_control_obs_P2_y_local - drone->RGB_control_obs_P1_y_local) / (drone->RGB_control_obs_P2_x_local - drone->RGB_control_obs_P1_x_local);
  c_GF = (-1 * a_GF * drone->RGB_control_obs_P1_x_local) + drone->RGB_control_obs_P1_y_local;

  drone->obs_RGB_GF << a_GF, c_GF;
                               
  if (drone->flagDroneRGBControlPoint1 == true && drone->flagDroneRGBControlPoint2 == true)
  {
    mission.counter_RGB_detection = mission.counter_RGB_detection + 1;
  }

  if (drone->flagDroneRGBControlPoint1 == true && drone->flagDroneRGBControlPoint2 == true && mission.panel_array_initialization == false || drone->RGB_image_control_count < 500)
  {
    Kalman_Filter->pass_to_KF_class_OBS_in_GF(a_GF, c_GF);
    //In ground frame
    //Kalman_Filter->Kalman_Filter_calculate(drone->RGB_control_obs_P1_x_local ,  drone->RGB_control_obs_P1_y_local ,  drone->RGB_control_obs_P2_x_local ,  drone->RGB_control_obs_P2_y_local);
    //--> IN BF
    //Kalman_Filter->Kalman_Filter_calculate(drone->control_RGB_point1_x , drone->control_RGB_point1_y , drone->control_RGB_point2_x, drone-> control_RGB_point2_y);
    Kalman_Filter->EKF_calculate(drone->control_RGB_point1_x, drone->control_RGB_point1_y, drone->control_RGB_point2_x, drone->control_RGB_point2_y,
                                 drone->new_current_local_pos.x, drone->new_current_local_pos.y, drone->yaw_in_Rad);
    drone->xh_ = Kalman_Filter->Obtain_Kalman_filter_estimated_state();
    drone->obs = Kalman_Filter->Obtain_Kalman_filter_observation();
    drone->yh_ = Kalman_Filter->Obtain_Kalman_filter_estimated_observation();

    cout << "[EKF RGB  ------->] Observations : a  " << drone->obs[0] << " c: " << drone->obs[1] << endl;
    cout << "[EKF RGB  ------->] Estimated states: a   " << drone->xh_[0] << " c: " << drone->xh_[1] << endl;
    cout << "[EKF RGB  ------->] Estimated Observation in BF:  a_BF " << drone->yh_[0] << " c_BF: " << drone->yh_[1] << endl;
    cout << "[EKF RGB  ------->] Observations in GF : a  " << a_GF << " c: " << c_GF << endl;
    from_image = true;
    from_image_RGB = true;

    if (drone->flagDroneRGBControlPoint1 == true && drone->flagDroneRGBControlPoint2 == true && abs(waypoints.error_from_GPS_line) < 15) //drone.image_control_count: permette la guida guidata dal filtro anche quando non sono rileavate piu osservazioni,
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

    if (from_image_Thermo == true)
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
    drone->x_target = mission.target_GPS_offset_x;     // mission.start_local_position.x; //mission.target_GPS_offset_x  --> L'offset è calcolato dopo
    drone->y_target = mission.start_local_position.y;  //drone->check_y_local;   ---> per quandfo il drone si allinea al waypoint utilizzo il punto generato dal controllo visivo per allineare il drone
    drone->z_target = -1 * drone->last_altitude - 100; //drone->last_position_z;
    drone->last_altitude = drone->current_local_pos.z; //drone->altitude;

    mission.P1_drone_lat = drone->current_gps.latitude;
    //Take ultrasonic data about altitude
    //drone->reference_altitude_value = drone->ultrasonic_altitude_value;
    mission.panel_array_initialization = true;
    from_image = false;
  }
  else
  {

    /* GPS EVALUATION BACKGROUND */

    /* ########## PUBLISH CONTOL POINT D IN BF TO ARDRONE IN GAZEBO ############# */
    drone->publish_control_D_point_to_GAZEBO(mission.height_above_ground , 0.05*drone->offset);
    mission.panel_array_initialization = false;
    mission.inNavigation = true;
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
  mission.cartesian_distance_2D = sqrt(pow(mission.x_GPS_offset_remaining, 2) + pow(mission.y_GPS_offset_remaining, 2));


   //Conntrol heigth via laser sensor if the flag enable_heigth_control_via_sensor is true
  float heigth_offset = 0.0;
  if (mission.enable_heigth_control_via_sensor == true)
  {
    //Flag per scelto da utente basato sulla scelta dell'altitudeine sempre fissa al valore desiderato 
    //oppure variabile in base alla scelta dell'utente prima dell'inizio della navigazione lungo la vela 
    if (mission.altitude_fixed_flag == true)
    {
      heigth_offset = mission.desired_laser_navigation_heigth - drone->ultrasonic_altitude_value;
    }
    else
    {
      if (mission.counter_altitude_sensor == 0)
      {
       
        mission.desired_laser_navigation_heigth = drone->current_local_pos.z;
        heigth_offset = mission.desired_laser_navigation_heigth - drone->ultrasonic_altitude_value;
        mission.counter_altitude_sensor = 1;
      }
      else
      {
         heigth_offset = mission.desired_laser_navigation_heigth - drone->ultrasonic_altitude_value;
        
    
      }
    }
  
    //if altitude is controlled respect the ultrasonic value 
    if (drone->ultrasonic_altitude_value > 1.5)
    {        
      drone->z_des = drone->current_local_pos.z + heigth_offset;
      cout << "[NAVIGATION] heigth_offset: " << heigth_offset << endl;
    }
    else
    {
      drone->z_des = drone->current_local_pos.z;
      cout << "[NAVIGATION] HEIGTH TOO LOW!! Z POS USED: " <<endl;
    }
   
  }

  // //IF the altitude is controlled respect the images 
  if (mission.height_above_ground == true)
  {
    if  (0.01 * drone->offset > 0.6)
    {
       drone->z_des  = drone->current_local_pos.z  + 0.01 * drone->offset;  //Nella realtà sara meno perche nella simulazione i pannelli RGB sono sopra il drone 
      drone->publish_control_D_point_to_GAZEBO(mission.height_above_ground ,0.01 * drone->offset );
    }
    else
    {
      drone->z_des  = drone->current_local_pos.z;
      drone->publish_control_D_point_to_GAZEBO(mission.height_above_ground ,0.0 );
    }
    cout << "drone->z_des : " << drone->z_des << endl;
    cout << "###################### ------------- >>offset: " << 0.05 * drone->offset << endl;
   
  }
   

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
  //Chreck on the Yaw desired 
  // if (drone->yaw_des_rad > M_PI)
  // {
  //   drone->yaw_des_rad = M_PI;
  // }
   
  
  //VELOCITY CONTROL
  drone->x_vel_cmd = pid_x->position_control_knowing_velocity(drone->x_target, drone->current_local_pos.x, drone->desired_velocity, drone->current_local_velocity.vector.x); //mission.target_GPS_offset_y
  drone->y_vel_cmd = pid_y->position_control_knowing_velocity(drone->y_target, drone->current_local_pos.y, 0, drone->current_local_velocity.vector.y); //mission.target_GPS_offset_x
  drone->z_vel_cmd = pid_z->position_control_knowing_velocity( drone->z_des, drone->current_local_pos.z, 0, drone->current_local_velocity.vector.z);

  // Evaluate Yaw_rfate for control cheking the direction of rotation.
  evaluate_yaw_rate_for_control(drone, pid_yaw);
 // drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_des_rad , drone->yaw_in_Rad, 0, drone->yaw_vel);
 
  
 
  // drone->x_vel_cmd = pid_x->calculate(drone->x_target, drone->current_local_pos.x); //mission.target_GPS_offset_y
  // drone->y_vel_cmd = pid_y->calculate(drone->y_target, drone->current_local_pos.y); //mission.target_GPS_offset_x
  // drone->z_vel_cmd = pid_z->calculate( drone->z_des, drone->current_local_pos.z);
  // drone->yaw_rate_cmd = pid_yaw->calculate(drone->yaw_des_rad, drone->yaw_in_Rad);
  
  //POSITION CONTROL OFFSET
  drone->x_pos_cmd = drone->x_target - drone->current_local_pos.x;
  drone->y_pos_cmd = drone->y_target - drone->current_local_pos.y;
  drone->z_pos_cmd = drone->z_des;

 
  

  //Il comando di velocità viene dato rispetto al frame di partenza, devo ruotarlo rispetto a quel frame

  // drone->yaw_des_rad = atan2(drone->target_offset_respect_drone_position.x, drone->target_offset_respect_drone_position.y);
 
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
    drone->THERMO_image_control_count = drone->THERMO_image_control_count + 1;
  }

  if (from_image_RGB == true)
  {
    //FInche flag rimane true continuo ad incrementare counter.
    // verra resettato solo quando i flag callback sono ture
    drone->RGB_image_control_count = drone->RGB_image_control_count + 1;
  }

  /* ARRIVO AL GPS WAYPOINT DI FINE PANNELLO 
Compute derivative of the distance function step by step in orde to see a change in variation:
Se la derivata ha segno negativo la distaza con il waypoint diminuisce 
Se la derivata ha segno positivo la disatnza con il waypoint aumenta.
--> rilevo cambio di pendenza per capire quando raggioungere il waypoint se la distanza non è rispettata.
*/
  // float derivative = mission.compute_distance_derivative();

  // if (mission.cartesian_distance_2D < 25.0)
  // {
  //   mission.compute_path_difference(drone->new_current_local_pos.x, drone->new_current_local_pos.x, drone->target_offset_respect_local_position);
  // }

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
    drone->y_pos_cmd = 0.0;
  }

 

  /* PUBLISH COMMAND VELOCITY TO DRONE */

  // drone->sat_x_vel = 3;
  // drone-> sat_y_vel = 3;
  // drone->saturation(drone->x_vel_cmd , drone->y_vel_cmd, drone->sat_x_vel, drone->sat_y_vel);
  //publish_command_velocity_to_drone(drone);
  mission.sat_x_off = 1;
  mission.sat_y_off = 1;
  if (mission.enable_navigation_velocity_control_flag == true)
  {
    mission.sat_x_off = drone->sat_x_vel ;
    mission.sat_y_off = 1.5;
    
    //Saturo le velocita se accelerazione troppo cosnistente: 
    //mission.velocity_saturation_given_acceleration(drone->x_ddot_GF, drone->y_ddot_GF );
   

    //Gia qua è negativa amnche se si muove da ovest verso est (assse x world posotivo)
                                                                //D[0] è positiva sempre perche espresso lungo asse x dronje 

    //Check YAW drone in order to give a correct velocity expressed in World frame on axle x 
    if (drone->yaw_in_Rad >= -M_PI/2 && drone->yaw_in_Rad <= M_PI/2 && drone->x_vel_cmd < 0)
    {
      //Vel X in GF Positiva 
       drone->x_vel_cmd = -1*drone->x_vel_cmd;
    }  
    else if (drone->yaw_in_Rad > M_PI/2 && drone->yaw_in_Rad <= M_PI && drone->x_vel_cmd > 0)
    {
        drone->x_vel_cmd = -1*drone->x_vel_cmd;
    }
    else if ( drone->yaw_in_Rad >= -M_PI && drone->yaw_in_Rad < -M_PI/2 && drone->x_vel_cmd > 0)
    {
         drone->x_vel_cmd = -1*drone->x_vel_cmd;
    }

     //Same check on the saturation velocity 
    if (drone->yaw_in_Rad >= -M_PI/2 && drone->yaw_in_Rad <= M_PI/2 &&drone->sat_x_vel< 0)
    {
      //Vel X in GF Positiva 
       drone->sat_x_vel= -1*drone->sat_x_vel;
    }  
    else if (drone->yaw_in_Rad > M_PI/2 && drone->yaw_in_Rad <= M_PI && drone->sat_x_vel > 0)
    {
        drone->sat_x_vel = -1*drone->sat_x_vel;
    }
    else if ( drone->yaw_in_Rad >= -M_PI && drone->yaw_in_Rad < -M_PI/2 && drone->sat_x_vel > 0)
    {
         drone->sat_x_vel = -1*drone->sat_x_vel;
    }

    
    if (drone->sat_x_vel > 0 && mission.sat_x_off < 0)
    {
      mission.sat_x_off = -1*mission.sat_x_off;
    }

    if (drone->sat_x_vel < 0 && mission.sat_x_off > 0)
    {
      mission.sat_x_off = -1*mission.sat_x_off;
    } 
    
    drone->saturation(drone->x_vel_cmd, drone->y_vel_cmd,  mission.sat_x_off , mission.sat_y_off);
    publish_command_velocity_to_drone(drone);
  
  }
  else
  {

    drone->sat_x_vel = 1.0;
    drone->sat_y_vel = 1.0;
    drone->saturation_offset(drone->x_pos_cmd, drone->y_pos_cmd,  drone->sat_x_vel , drone->sat_y_vel);
    publish_command_position_to_drone(drone);
  }




  mission.distance_to_target = 2.0;
  mission.inNavigation = true;
  if (mission.target_reached == true)
  {
    cout << "[INFO NAVIGATION] End Panel Waypoint Reached" << endl;
    mission.state = 1;       //2--> Reaching starting_waypoint  //1--> Allineo Yaw con Waypoint sucessivo
    mission.enter_state = 4; //1 --> permette di raggiungere Reaching starting position //4---> Permette di passare allo stato Jump_panels da stato align Yaw, perche viene sommato + 1
    mission.target_point = 1;
    mission.inbound_counter = 0;
    mission.target_reached = false;
    mission.flag_navigate_to_waypoints = false;
    mission.breaking_counter = 0;
    mission.panel_array_initialization = true;
    mission.end_panel_reached = true; //Necessario per camera in Gazebo per orientarsi su nuovo pannello
    mission.inNavigation = false;
    mission.desired_yaw_flag = true;
    //Publish zero control point to ARDRONE
    drone->control_x_coo = 0.0;
    drone->control_y_coo = 0.0;
    drone->publish_control_D_point_to_GAZEBO(mission.height_above_ground , 0.0);

    drone->last_altitude = drone->current_local_pos.z; //drone->altitude;
    drone->last_position_z = drone->current_local_pos.z;
    drone->z_des =  drone->last_position_z ; //drone->current_local_pos.z;
    //Vai in stato Waiting se drone raggiunge il target finale in modalità test lungo una singola vela 
    if (mission.use_two_single_fixed_waypoints_flag == true)
    {
      //Il controllo viene ridato all'user 
      cout << "[NAVIGATION] GIVE CONTROL TO THE USER" << endl;
      mission.state = 8;
    }

    //Place New Frame in Waypoint
    mission.start_gps_location = drone->current_gps;
    mission.start_local_position = drone->current_local_pos;
    mission.referred_local_yaw = drone->yaw_in_Rad_sim; //Salvo lo yaw del drone nel nuovo local frame in modo da poter avere uno yaw di riferimento per le trasformazioni
    drone->image_control_count = 0;
    drone->THERMO_image_control_count = 6000;
    drone->RGB_image_control_count = 6000;
    waypoints.counter = waypoints.counter + 1;
    mission.counter_altitude_sensor = 0;
    ros::Duration(1.0).sleep();
  }
  else
  {
    mission.inbound_counter = mission.inbound_counter + 1;
    mission.cartesian_distance_2D_old = mission.cartesian_distance_2D;
  }

  //Ottengo controllo manuale del drone
  if (drone->key_for_control_is_pressed == true)
  {
    
    mission.inbound_counter = 0;
    mission.target_reached = false;
    mission.flag_navigate_to_waypoints = false;
    mission.breaking_counter = 0;
    mission.panel_array_initialization = true;
    mission.end_panel_reached = true; //Necessario per camera in Gazebo per orientarsi su nuovo pannello
    mission.inNavigation = false;
    //Publish zero control point to ARDRONE
    drone->control_x_coo = 0.0;
    drone->control_y_coo = 0.0;
    //drone->publish_control_D_point_to_GAZEBO();

    drone->last_altitude = drone->current_local_pos.z;//drone->altitude;
    drone->last_position_z = drone->current_local_pos.z;
    drone->z_des =  drone->last_position_z ;
    drone->image_control_count = 0;
    drone->THERMO_image_control_count = 6000;
    drone->RGB_image_control_count = 6000;
    mission.counter_altitude_sensor = 0;
    drone->key_for_control_is_pressed = false;
    mission.state = 8;
  }
}

void evaluate_GPS_offset_for_new_array_starting_point(Drone *drone)
{
  //GPS coo End Point previous Panel
  double P1_lat = waypoints.GPS_waypoints_lat[waypoints.counter - 1];
  double P1_lon = waypoints.GPS_waypoints_lon[waypoints.counter - 1];
  // cout << " P1_lat: " << P1_lat << " P1_lon:  " << P1_lon << endl;

  //GPS coo Start Point Next Panel
  double P2_lat = waypoints.GPS_waypoints_lat[waypoints.counter];
  double P2_lon = waypoints.GPS_waypoints_lon[waypoints.counter];
  // cout << " P2_lat: " << P2_lat << " P2_lon:  " << P2_lon << endl;

  //GPS coo  of start poit of the last navigated array

  double previous_x_start_lat = waypoints.GPS_waypoints_lat[waypoints.counter - 2];
  double previous_y_start_lon = waypoints.GPS_waypoints_lon[waypoints.counter - 2];
  // cout << " previous_x_start_lat: " << previous_x_start_lat << " previous_y_start_lon:  " << previous_y_start_lon << endl;

  //evaluate_local_offset_between_GPS_target_and_local_frame(P1_lat, P1_lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_local_position);
  evaluate_local_offset_between_GPS_target_and_body_frame(P1_lat, P1_lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_drone_position);
  //Obtain GPS waypoints of previous panel end point --> Since GPS coo expressed in NED frame i have to invert axis to express the position respect local frame
  mission.x_target_P1 = drone->current_local_pos.x + drone->target_offset_respect_drone_position.y;
  mission.y_target_P1 = drone->current_local_pos.y + drone->target_offset_respect_drone_position.x;
  cout << " mission.x_target_P1_B: " << drone->target_offset_respect_drone_position.y << " mission.y_target_P1_B " << drone->target_offset_respect_drone_position.x << endl;

  //evaluate_local_offset_between_GPS_target_and_local_frame(P2_lat, P2_lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_local_position);
  evaluate_local_offset_between_GPS_target_and_body_frame(P2_lat, P2_lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_drone_position);
  //Obtain GPS waypoints of previous panel end point --> Since GPS coo expressed in NED frame i have to invert axis to express the position respect local frame
  mission.x_target_P2 = drone->current_local_pos.x + drone->target_offset_respect_drone_position.y;
  mission.y_target_P2 = drone->current_local_pos.y + drone->target_offset_respect_drone_position.x;
  cout << " mission.x_target_P2_B: " << drone->target_offset_respect_drone_position.y << " mission.y_target_P2_B: " << drone->target_offset_respect_drone_position.x << endl;

  //evaluate_local_offset_between_GPS_target_and_local_frame(previous_x_start_lat, previous_y_start_lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_local_position);
  evaluate_local_offset_between_GPS_target_and_body_frame(previous_x_start_lat, previous_y_start_lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_drone_position);
  //Obtain GPS waypoints of previous panel end point --> Since GPS coo expressed in NED frame i have to invert axis to express the position respect local frame
  mission.previous_x_start = drone->current_local_pos.x + drone->target_offset_respect_drone_position.y;
  mission.previous_y_start = drone->current_local_pos.y + drone->target_offset_respect_drone_position.x;

  //Distance from the previous starting waypoint in the panel just inspected
  // cout << " mission.previous_x_start: " << drone->target_offset_respect_drone_position.y << " mission.previous_y_start: " << drone->target_offset_respect_drone_position.x << endl;

  // cout << " mission.x_target_P1: " << mission.x_target_P1 << " mission.y_target_P1: " << mission.y_target_P1 << endl;
  // cout << " mission.x_target_P2: " << mission.x_target_P2 << " mission.y_target_P2: " << mission.y_target_P2 << endl;
  // cout << " mission.previous_x_start: " << mission.previous_x_start << " mission.previous_y_start: " << mission.previous_y_start << endl;

  //Evaluate angle between previous_start, P1, P2
  float a = sqrt(pow(mission.x_target_P1 - mission.previous_x_start, 2) + pow(mission.y_target_P1 - mission.previous_y_start, 2)); //distance between previous start e P1
  float b = sqrt(pow(mission.x_target_P2 - mission.x_target_P1, 2) + pow(mission.y_target_P2 - mission.y_target_P1, 2));           //distance between P2 and  P1
  float c = sqrt(pow(mission.x_target_P2 - mission.previous_x_start, 2) + pow(mission.y_target_P2 - mission.previous_y_start, 2)); //distance between opreviosu start e P2

  
  float el1 = c * c + a * a - b * b;
  float el2 = 2.0 * a * c;
  float el3 = el1 / el2;
  float alfa = acos(el3);

  el1 = b * b + c * c - a * a;
  el2 = 2.0 * b * c;
  el3 = el1 / el2;
  float beta = acos(el3);

  el1 = a * a + b * b - c * c;
  el2 = 2.0 * a * b;
  el3 = el1 / el2;
  float gamma = acos(el3); //angle relative to the panel jump

 

  //ROtate the three points in the UAV body frame
  //Obtain B coo of the next start array waypoint

  if (mission.inbound_counter < 5)
  {
    drone->Rotation_local_GF_to_BF(drone->current_local_pos, mission.x_target_P2, mission.y_target_P2, drone->yaw_in_Rad);
    mission.x_target_P2_B = drone->check_x_b;
    mission.y_target_P2_B = drone->check_y_b;

    //Obtain B coo of the previous start wayooint
    drone->Rotation_local_GF_to_BF(drone->current_local_pos, mission.previous_x_start, mission.previous_y_start, drone->yaw_in_Rad);
    float x_target_P1_B = drone->check_x_b;
    float y_target_P1_B = drone->check_y_b;
  }

  //Distance done from drone last position --->  mission.start_local_position = drone position taken at the end of the solar array
  float distance_done = sqrt(pow(drone->current_local_pos.x - mission.start_local_position.x, 2) + pow(drone->current_local_pos.y - mission.start_local_position.y, 2));
  cout << "distance_done: " << distance_done << endl;

  //Find the next point to reach, which is the next starting point, based on the angle gamma
  
  if (gamma >= M_PI / 2)
  {
    //Il target P2 forma con il targetP1 un angolo > 90
    //Il triangolo da considerare è ottuso, quindi l'angolo non gamma è maggiore di 90 e va considerato Beta.
    //Guardare Ipad
    if (mission.y_target_P2_B >= 0)
    {
      mission.y_target_P2_BF = (b - distance_done) * sin(beta);
    }
    else
    {
      mission.y_target_P2_BF = -1 * (b - distance_done) * sin(beta);
    }

    if (mission.x_target_P2_B >= 0)
    {
      mission.x_target_P2_BF = (b - distance_done) * cos(beta);
    }
    else
    {
      mission.x_target_P2_BF = -1 * (b - distance_done) * cos(beta);
    }
  }
  else
  {

    // Il triangolo da considerare è acuto
    if (mission.y_target_P2_B >= 0)
    {
      mission.y_target_P2_BF = (b - distance_done) * sin(gamma);
    }
    else
    {
      mission.y_target_P2_BF = -1 * (b - distance_done) * sin(gamma);
    }

    if (mission.x_target_P2_B >= 0)
    {
      mission.x_target_P2_BF = (b - distance_done) * cos(gamma);
    }
    else
    {
      mission.x_target_P2_BF = -1 * (b - distance_done) * cos(gamma);
    }
  }

  //Reassign a new value to  mission.x_target_P2 and  mission.x_target_P3 relative to the evaluted position of new panel array starting point

 
  cout << "[JUMP PANELS] x_target_P2_BF: " << mission.x_target_P2_BF << " y_target_P2_BF: " << mission.y_target_P2_BF << endl;
  cout << "[JUMP PANELS] drone x: " << drone->current_local_pos.x << "drone y: " << drone->current_local_pos.y << endl;

  //Rotate Target Point in Local Frame
  drone->Rotation_BF_to_local_GF_des_pos_original(drone->current_local_pos, mission.x_target_P2_BF, mission.y_target_P2_BF, drone->yaw_in_Rad);
  mission.x_target_P2 = drone->check_x_local;
  mission.y_target_P2 = drone->check_y_local;

  mission.setTarget(mission.x_target_P2, mission.y_target_P2, drone->z_des, 0);
  cout << "[JUMP PANELS] x_target_P2: " << mission.x_target_P2 << " y_target_P2: " << mission.y_target_P2 << endl;
}

void jump_panels(Drone *drone, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{

  cout << "[JUMP PANELS] Reaching NEW PANELS ARRAY" << endl;
  if (mission.inbound_counter < 10)
  {
    //Finde new target poition relative to new waypoint
    evaluate_GPS_offset_for_new_array_starting_point(drone);
  }
  else
  {
    //Update new target position relative to new starting Waypoints
    localOffsetFromGpsOffset(drone->local_drone_position, drone->current_gps, mission.HOME_gps_location);
    mission.x_offset_remaining = mission.target_offset_x - drone->local_drone_position.y;
    mission.y_offset_remaining = mission.target_offset_y - drone->local_drone_position.x;
    mission.z_offset_remaining = mission.target_offset_z - drone->local_drone_position.z; //localOffset è la oposizione attuale del drone rispetto al frame di riferimento dopo la trasformazione da GPS coo

    cout << "[JUMP PANELS] x_off: " << mission.x_offset_remaining << endl;
    cout << "[JUMP PANELS] y_off: " << mission.y_offset_remaining << endl;
    cout << "[JUMP PANELS] mission.target_offset_x: " << mission.target_offset_x << " mission.target_offset_y: " << mission.target_offset_y << endl;
    cout << "[JUMP PANELS] drone->local_drone_position.x: " << drone->local_drone_position.x << " drone->local_drone_position.y: " << drone->local_drone_position.y << endl;

    mission.cartesian_distance_2D = sqrt(pow(mission.x_offset_remaining, 2) + pow(mission.y_offset_remaining, 2));
    // mission.cartesian_distance_3D = sqrt(pow(mission.target_offset_x - drone->current_local_pos.x,2) + pow(mission.target_offset_y)

    // VELOCITY CONTROL  -->
    /* NB: INVERTO LE COO DI TARGET DI X E Y PERCHE L'OFFSET LO CALCOLA RISPETTO AL FRAME LOCALE, IL QUALE PRESENTA X  E Y INVERTITE RISPETTO A QUELLO OTTENUTO 
    TRAMITE LA TRASFORMAZIONE DELLA LAT LON DEL GPS A DELTA X DELTA Y */

    drone->x_vel_cmd = pid_x->position_control_knowing_velocity(mission.target_offset_y, drone->current_local_pos.x, 0, drone->current_local_velocity.vector.x);
    drone->y_vel_cmd = pid_y->position_control_knowing_velocity(mission.target_offset_x, drone->current_local_pos.y, 0, drone->current_local_velocity.vector.y);
    drone->z_vel_cmd = pid_z->position_control_knowing_velocity(mission.target_offset_z, drone->current_local_pos.z, 0, drone->current_local_velocity.vector.z);

    drone->yaw_rate_cmd = pid_yaw->position_control_knowing_velocity(drone->yaw_in_Rad_sim, drone->yaw_des_rad_sim, 0, drone->yaw_vel);

    // POSITION CONTROL --> Sempre pubblicato rispetto local_position frame
    drone->x_pos_cmd = 0.3 * mission.x_offset_remaining;
    drone->y_pos_cmd = 0.3 * mission.y_offset_remaining;
    drone->z_pos_cmd = drone->z_des; //drone->last_altitude ; //drone->last_position_z; //drone->z_des;

    publish_command_position_to_drone(drone);
    //publish_command_velocity_to_drone(drone);
  }

  //Continue to sent Command to Gazebo.
  //Sent x = 0.0 e y = 0.0 --> Arddrone sstop in its last position
  drone->control_x_coo = 0.0;
  drone->control_y_coo = 0.0;
  drone->publish_control_D_point_to_GAZEBO(mission.height_above_ground ,0.0);
  mission.distance_to_target = 0.5;
  if (mission.target_reached == true)
  {
    mission.state = 4;       //Allineo Yaw con Waypoint finale
    mission.enter_state = 3; //Permette di passare a navigation dopo Yaw  alignment state
    mission.target_point = 2;

    mission.inbound_counter = 0;
    mission.target_reached = false;
    mission.breaking_counter = 0;
    drone->z_des =  drone->last_position_z ; //drone->current_local_pos.z;

    mission.start_gps_location = drone->current_gps;
    mission.start_local_position = drone->current_local_pos;
    mission.referred_local_yaw = drone->yaw_in_Rad_sim; //Salvo lo yaw del drone nel nuovo local frame in modo da poter avere uno yaw di riferimento per le trasformazioni
    waypoints.counter = waypoints.counter + 1;
    ros::Duration(1.0).sleep();
  }
  else
  {
    mission.state = 5;
    mission.inbound_counter = mission.inbound_counter + 1;
  }

  //Ottengo controllo manuale del drone
  if (drone->key_for_control_is_pressed == true)
  {
    mission.inbound_counter = 0;
    mission.target_reached = false;
    mission.flag_navigate_to_waypoints = false;
    mission.breaking_counter = 0;
    mission.panel_array_initialization = true;
    mission.end_panel_reached = true; //Necessario per camera in Gazebo per orientarsi su nuovo pannello
    mission.inNavigation = false;
    //Publish zero control point to ARDRONE
    drone->control_x_coo = 0.0;
    drone->control_y_coo = 0.0;
    drone->publish_control_D_point_to_GAZEBO(mission.height_above_ground ,0.0);

    drone->last_altitude = drone->current_local_pos.z;
    drone->last_position_z = drone->current_local_pos.z;
    drone->z_des =  drone->last_position_z ; //-drone->current_local_pos.z;

    drone->image_control_count = 0;
    drone->THERMO_image_control_count = 6000;
    drone->RGB_image_control_count = 6000;

    drone->key_for_control_is_pressed = false;
    mission.state = 8;
  }
}

/* Waiting for parameters optimization in previous position*/
void waiting_for_parameters_optimization(Drone *drone, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
  /*
  This function if enabled is called after the moment where the drone is aligned by the user.
  */
  //Waiting in last positiom for Parameters Optimization
  cout << "[PARAM OPT]  Waiting For Parameters Optimization" << endl;

  drone->x_pos_cmd = mission.target_offset_x - drone->current_local_pos.x;
  drone->y_pos_cmd = mission.target_offset_y - drone->current_local_pos.y;
  drone->z_pos_cmd = mission.target_offset_z - drone->current_local_pos.z;

  drone->yaw_des_rad = mission.target_yaw;
  publish_command_position_to_drone(drone);
  mission.start_optimization_task = true;

  cout << "[PARAM OPT] OPTIMIZATION COMPLETED: " << drone->Optimization_completed << endl;
  if (drone->Optimization_completed == true)
  {
    mission.state = 4;
    mission.target_point = 2;

  
    mission.inbound_counter = 0;
    mission.target_reached = false;
    mission.end_panel_reached = false;
    mission.breaking_counter = 0;
    //Initialize Drone Position --> Sort of transformation respect world frame placed in HOME POINT
    //The yaw evaluation is done automatically by the drone.

    //Save new current altitude
  
   
    // cout << "drone->z_des: " << drone->z_des << endl;

    //clear the array
    waypoints.distance_from_GPS_coo.clear();
    waypoints.angle_vector.clear();

    //Save Last Z altitude
    drone->last_altitude = drone->current_local_pos.z;
    drone->last_position_z = drone->current_local_pos.z;
   
    drone->z_des = drone->current_local_pos.z; //- drone->last_position_z; //drone->altitude;
    mission.start_gps_location = drone->current_gps;
    mission.start_local_position = drone->current_local_pos;
    mission.referred_local_yaw = drone->yaw_in_Rad_sim;
    drone->yaw_des_rad = drone->yaw_in_Rad;

  }
  else
  {
    mission.state = 6;
  }

  //Ottengo controllo manuale del drone
  if (drone->key_for_control_is_pressed == true)
  {
    mission.inbound_counter = 0;
    mission.target_reached = false;
    mission.flag_navigate_to_waypoints = false;
    mission.breaking_counter = 0;
    mission.panel_array_initialization = true;
    mission.end_panel_reached = true; //Necessario per camera in Gazebo per orientarsi su nuovo pannello
    mission.inNavigation = false;
    //Publish zero control point to ARDRONE
    drone->control_x_coo = 0.0;
    drone->control_y_coo = 0.0;
    //drone->publish_control_D_point_to_GAZEBO();

    drone->last_altitude = drone->current_local_pos.z; //drone->altitude;
    drone->last_position_z = drone->current_local_pos.z;
    drone->z_des =  drone->last_position_z ; //- drone->current_local_pos.z;
    drone->image_control_count = 0;
    drone->THERMO_image_control_count = 6000;
    drone->RGB_image_control_count = 6000;

    drone->key_for_control_is_pressed = false;
    mission.state = 8;
  }
}





/*
Funzione che permette di prendere il controllo del drone in qualun uqe fase durate la navigaszione.
Se vien epremuto s il controllo del drone è passato all'utenete.
Se viene premuto s una seconda volta il controllo viene ripreso dal drone e passa in moldalità navigation<
*/
void obtain_user_control(Drone *drone)
{
  bool obtain_control_result = false;
  bool release_user_control = false;

  if (mission.counter_case_8 == 0)
  {
    obtain_control_result = drone->release_control();
    if (!obtain_control_result)
    {
      ROS_ERROR("Impossible to Release Drone Control by OSDK. Exiting.");
    }
    else
    {
      ROS_INFO("Drone Control Released!");
    }
    drone->key_for_control_is_pressed = false;
  }
  // Drone rimane in questo caso finche il pulsante non viene premuto nuovamente
  //drone.key_for_control_is_pressed --> flag relativo al fatto se un tasto viene premuto
  cout << "[USER CONTROL TASK] THE CONTROL IS ON THE USER! KEY PRESSED:  " << drone->array_number<< endl;
  
  if (drone->key_for_control_is_pressed == true && mission.counter_case_8 > 100)
  {
    release_user_control = true;
    obtain_control_result = drone->obtain_control();

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
    /*
        Necessario inizializzare il sistema con i waypoint corretti:
        il drone deve capire in quale waypoint della mappa si trova,
        e inizializzare il filtro con il waypoint sucessivo.
        Calcolare con la distanza 
        */

    if (mission.use_two_single_fixed_waypoints_flag == true)
    {
      
      //Clear GPS_waypoint arrays:
      waypoints.GPS_waypoints_lat.clear();
      waypoints.GPS_waypoints_lon.clear();
      

      //Evaluate longitude and latitude offset between the two fixed waypoints
      float lat_offset = abs(waypoints.GPS_fixed_waypoints_lat[1] - waypoints.GPS_fixed_waypoints_lat[0]);
      float lon_offset = abs(waypoints.GPS_fixed_waypoints_lon[1] - waypoints.GPS_fixed_waypoints_lon[0]);
    
      float P2_lat = 0.0;
      float P1_lat = 0.0;
      float P2_lon = 0.0;
      float P1_lon = 0.0;

      P2_lat = waypoints.GPS_fixed_waypoints_lat[1];
      P1_lat = waypoints.GPS_fixed_waypoints_lat[0];

      P2_lon = waypoints.GPS_fixed_waypoints_lon[1];
      P1_lon = waypoints.GPS_fixed_waypoints_lon[0];

      if (P2_lat > P1_lat)
      {
        //Verifico yaw drone per capire se sommare o sottrarre l'offset in longitudine.
        //IL concetto funziona sapendo che longitudione di P2 è sempre maggiore della longitudine di P1.
        //--> Quindi faccia alla cartina orientata con il nord verso l'alto, P2 si trova a destra di P1

        if (drone->yaw_in_Rad >= -M_PI/2 && drone->yaw_in_Rad < M_PI/2)
        {
          //Il drone punta verso est --> direzione di P2

          //Inserisco Coordinata P1 riferita a posizione corrente drone
          waypoints.GPS_waypoints_lat.push_back(drone->current_gps.latitude);
          waypoints.GPS_waypoints_lon.push_back(drone->current_gps.longitude);
         
          if (lon_offset > 0)
          {
            lon_offset = lon_offset + 0.0001;
          }
          else
          {
            lon_offset = lon_offset -  0.0001;
          }

        
          float P2_new_lon = drone->current_gps.longitude + lon_offset;
          float P2_new_lat = drone->current_gps.latitude + lat_offset;

          waypoints.GPS_waypoints_lat.push_back(P2_new_lat);
          waypoints.GPS_waypoints_lon.push_back(P2_new_lon);
           
          cout <<"#################################################" << endl;
          cout <<"                                                 " << endl;
          cout <<"                                                 " << endl;
          cout << "[USER CONTROL TASK] Drone Placed in P1 --> Proceed towards EAST with lon_offset: " << P2_new_lon - drone->current_gps.longitude << " , lat_offset: " << P2_new_lat - drone->current_gps.latitude << endl;
          cout <<"                                                 " << endl;
          cout <<"#################################################" << endl;
       
          
        }
        else
        {
          //Drone orientato verso ovest
          //Inserisco Coordinata P1 riferita a posizione corrente drone
          waypoints.GPS_waypoints_lat.push_back(drone->current_gps.latitude);
          waypoints.GPS_waypoints_lon.push_back(drone->current_gps.longitude);
          
         
          if (lon_offset > 0)
          {
            lon_offset = lon_offset +  0.0001;
          }
          else
          {
            lon_offset = lon_offset - 0.0001;
          }
        
          float P2_new_lon = drone->current_gps.longitude - lon_offset;
          float P2_new_lat = drone->current_gps.latitude - lat_offset;

          waypoints.GPS_waypoints_lat.push_back(P2_new_lat);
          waypoints.GPS_waypoints_lon.push_back(P2_new_lon);
           
          cout <<"#################################################" << endl;
          cout <<"                                                 " << endl;
          cout <<"                                                 " << endl;        
          cout << "[USER CONTROL TASK] Drone Placed in P2 --> Proceed towards OVEST with lon_offset: " << P2_new_lon - drone->current_gps.longitude << " , lat_offset: " << P2_new_lat - drone->current_gps.latitude << endl;
          cout <<"                                                 " << endl;
          cout <<"#################################################" << endl;          
          
        
        }
      }
      else
      {
        //Se P1_lat > P2_lat
        if (drone->yaw_in_Rad >= -M_PI/2 && drone->yaw_in_Rad < M_PI/2)
        {
          //Il drone punta verso ovest --> direzione di P2

          //Inserisco Coordinata P1 riferita a posizione corrente drone
          waypoints.GPS_waypoints_lat.push_back(drone->current_gps.latitude);
          waypoints.GPS_waypoints_lon.push_back(drone->current_gps.longitude);
          
      
          if (lon_offset > 0)
          {
            lon_offset = lon_offset +  0.0001;
          }
          else
          {
            lon_offset = lon_offset - 0.0001;
          }
         

          float P2_new_lon = drone->current_gps.longitude + lon_offset;
          float P2_new_lat = drone->current_gps.latitude - lat_offset;

          waypoints.GPS_waypoints_lat.push_back(P2_new_lat);
          waypoints.GPS_waypoints_lon.push_back(P2_new_lon);

          cout << "[USER CONTROL TASK] Drone Placed in P1 --> Proceed towards EAST with lon_offset: " << P2_new_lon - drone->current_gps.longitude << " , lat_offset: " << P2_new_lat - drone->current_gps.latitude << endl;
      
        }
        else
        {
          //Drone orientato verso est
          //Inserisco Coordinata P1 riferita a posizione corrente drone
          waypoints.GPS_waypoints_lat.push_back(drone->current_gps.latitude);
          waypoints.GPS_waypoints_lon.push_back(drone->current_gps.longitude);
          
          
          if (lon_offset > 0)
          {
            lon_offset = lon_offset +  0.0001;
          }
          else
          {
            lon_offset = lon_offset - 0.0001;
          }
        
          float P2_new_lon = drone->current_gps.longitude - lon_offset;
          float P2_new_lat = drone->current_gps.latitude + lat_offset;

          waypoints.GPS_waypoints_lat.push_back(P2_new_lat);
          waypoints.GPS_waypoints_lon.push_back(P2_new_lon);

          cout << "[USER CONTROL TASK] Drone Placed in P2 --> Proceed towards OVEST with lon_offset: " << P2_new_lon - drone->current_gps.longitude << " , lat_offset: " << P2_new_lat - drone->current_gps.latitude << endl;
       
        }
      }
      

      
      waypoints.counter = 1;
      
    
    }
    else
    {

      //Create a vector containing all the distances between waypoints
      double angle = 0.0;

      for (int i = 0; i < waypoints.GPS_waypoints_lat.size(); i++)
      {
        double lat = waypoints.GPS_waypoints_lat[i];
        double lon = waypoints.GPS_waypoints_lon[i];
        // drone->current_local_pos.x = 10;
        // drone->current_local_pos.y = 21;

        // drone->current_gps.latitude =    44.781706; //44.781131;
        // drone-> current_gps.longitude =  8.640408; //8.641030;

        //evaluate_local_offset_between_GPS_target_and_local_frame(P1_lat, P1_lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_local_position);
        evaluate_local_offset_between_GPS_target_and_body_frame(lat, lon, drone->current_gps, drone->current_local_pos, drone->target_offset_respect_drone_position);
        //Obtain GPS waypoints of previous panel end point --> Since GPS coo expressed in NED frame i have to invert axis to express the position respect local frame
        double x_coo_GPS_considered = drone->current_local_pos.x + drone->target_offset_respect_drone_position.y;
        double y_coo_GPS_considered = drone->current_local_pos.y + drone->target_offset_respect_drone_position.x;

        //Evaluate distance between drone and GPS coordinates
        double drone_GPS_distance = sqrt(pow(x_coo_GPS_considered - drone->current_local_pos.x, 2) + pow(y_coo_GPS_considered - drone->current_local_pos.y, 2));
        //Save all the waypoint distances inside a vector
        waypoints.distance_from_GPS_coo.push_back(drone_GPS_distance);

        //Evaluate angle between drone yaw and the waypoints position
        //-- Yaw rispetto a local frame coincide con yaw
        // Rotation of the GPS coo in body frame
        // drone->yaw_in_Rad = 0.04;
        geometry_msgs::Point drone_pos;
        drone_pos.x = drone->current_local_pos.x;
        drone_pos.y = drone->current_local_pos.y;
        drone->Rotation_local_GF_to_BF(drone_pos, x_coo_GPS_considered, y_coo_GPS_considered, drone->yaw_in_Rad);
      

        angle = atan2(drone->check_y_b, drone->check_x_b);
        double diff = drone->yaw_in_Rad - angle;
        waypoints.angle_vector.push_back(angle);
      }

      //Search for the minimum waypoint distance iside the vectopr--> find nearest waypoint
      double min_distance = *min_element(waypoints.distance_from_GPS_coo.begin(), waypoints.distance_from_GPS_coo.end());

      //Find the correspondant index element inside the vector
      mission.getIndex(waypoints.distance_from_GPS_coo, min_distance);
      //Find the waypoints on the left or on the right of the waypoint with minimum distance from the drone that present an higher distance
      //_--> Find waypoint P2
      double dist_1 = 0.0;
      double dist_2 = 0.0;
      double angle_1 = 0.0;
      double angle_2 = 0.0;
      int index1 = 0;
      int index2 = 0;

      //mission.distance_vector_index ---> WAYPOINT INDEX CON LA MINIMA DISTANZA DA DOVE SI TROVA IL DRONE
      if (mission.distance_vector_index - 1 < 0)
      {
        dist_1 = waypoints.distance_from_GPS_coo[mission.distance_vector_index];
        angle_1 = waypoints.angle_vector[mission.distance_vector_index];
        index1 = mission.distance_vector_index;
      }
      else
      {
        dist_1 = waypoints.distance_from_GPS_coo[mission.distance_vector_index - 1];
        angle_1 = waypoints.angle_vector[mission.distance_vector_index - 1];
        index1 = mission.distance_vector_index - 1;
      }

      if (mission.distance_vector_index + 1 > waypoints.distance_from_GPS_coo.size())
      {
        dist_2 = waypoints.distance_from_GPS_coo[mission.distance_vector_index];
        angle_2 = waypoints.angle_vector[mission.distance_vector_index];
        index2 = mission.distance_vector_index;
      }
      else
      {
        dist_2 = waypoints.distance_from_GPS_coo[mission.distance_vector_index + 1];
        angle_2 = waypoints.angle_vector[mission.distance_vector_index + 1];
        index2 = mission.distance_vector_index + 1;
      }

      //IMPORTANTE!!!! --> verificare che il punto piu lontano sia anche allineato con lo yaw del robot per evitare situazioni
      //tipo la parte superiore dell'impianto di Predosa. Dove la distanza tra i pannelli è maggiore della lunghezza del pannello
      // cout<<"dist1: " << dist_1 << endl;
      // cout <<"dist2: "<< dist_2 << endl;
      // cout <<"angle_1: " << angle_1 << endl;
      // cout <<"angle_2: " << angle_2 << endl;
      // cout<<"index1: " << index1 << endl;
      //  cout<<"index2: " << index2 << endl;
      //Define next GPS P2 waypoint to initialize the Kalman Filter

      cout << "[USER CONTROL TASK] NEAREST WAYPOINT INDEX: " << mission.distance_vector_index << endl;

      if (dist_1 > dist_2 && abs(angle_1) < abs(angle_2))
      {

        waypoints.counter = index1;
        // cout << "dist1: lat " << waypoints.GPS_waypoints_lat[index1] << " lon: " << waypoints.GPS_waypoints_lon[index1] << endl;
        // cout << "dist_1: " << dist_1 << endl;
        cout << "[USER CONTROL TASK] NEXT WAYPOINT INDEX: " << index1 << endl;
      }
      else
      {
        if (dist_1 < dist_2 && abs(angle_1) < abs(angle_2))
        {
          waypoints.counter = index2;
          // cout << "dist1: lat " << waypoints.GPS_waypoints_lat[index1] << " lon: " << waypoints.GPS_waypoints_lon[index1] << endl;
          // cout << "dist_1 in if : " << dist_1 << endl;
          cout << "[USER CONTROL TASK] NEXT WAYPOINT INDEX: " << index1 << endl;
        }
        else
        {
          //Il waypoint sucessivo è definito come quello piu vicino al drone + 1.
          //Quindi se il waypoint piu vicino al drone è il numero 0 quello sucessivo è quello che si trova all'indice 1.
          waypoints.counter = index2;
          // cout << "dist2: lat " << waypoints.GPS_waypoints_lat[index2] << " lon: " << waypoints.GPS_waypoints_lon[index2] << endl;
          // cout << "dist_2: " << dist_2 << endl;
          cout << "[USER CONTROL TASK] NEXT WAYPOINT INDEX: " << index2 << endl;
        }
      }
    }
   
    mission.counter_case_8_exit_value = mission.counter_case_8;
  }

  if (mission.Optimization_enabled == true && release_user_control == true)
  {
    cout << "[USER CONTROL TASK] GO TO WAIT FOR PARAMETERS OPTIMIZATION TASK: " << endl;
    mission.setTarget(drone->current_local_pos.x, drone->current_local_pos.y, drone->current_local_pos.z, drone->yaw_in_Rad_sim);
    mission.state = 6; //Go to wait for parameters Optimization case
    mission.counter_case_8 = 0;
    drone->user_control_flag = false;
    sleep(4);
  }
  else if (mission.Optimization_enabled == false && release_user_control == true)
  {
    if (drone->key_for_control_is_pressed == true)
    {
      //Sleep for 3 second until the flag becomes false, otherwise the code come back here from navigation case
      cout << "[USER CONTROL TASK] WAIT THREE SECONDS TO START EXECUTION. Switch to Nav Mode" << endl;
      sleep(3);
    
      drone->user_control_flag = false;
    }

    mission.target_point = 2;

    mission.inbound_counter = 0;
    mission.target_reached = false;
    mission.end_panel_reached = false;
    mission.breaking_counter = 0;
    //Initialize Drone Position --> Sort of transformation respect world frame placed in HOME POINT
    //The yaw evaluation is done automatically by the drone.



   
    // cout << "drone->z_des: " << drone->z_des << endl;

    //clear the array
    waypoints.distance_from_GPS_coo.clear();
    waypoints.angle_vector.clear();

    //Save Last Z altitude
    drone->last_altitude = drone->current_local_pos.z;
    drone->last_position_z = drone->current_local_pos.z;
  
    drone->z_des = drone->current_local_pos.z; //- drone->last_position_z; //drone->altitude;
    mission.start_gps_location = drone->current_gps;
    mission.start_local_position = drone->current_local_pos;
    mission.referred_local_yaw = drone->yaw_in_Rad_sim;
    drone->yaw_des_rad = drone->yaw_in_Rad;

    //Go to navigation
    mission.state = 4;
    mission.counter_case_8 = 0;
    drone->user_control_flag = false;
  }

  else
  {
    if (mission.counter_case_8 > 100)
    {
      cout << "YOU CAN NOW PRESS A KEY!:  " << endl;
    }
    //Continua  a rimanere in questo caso ficnhe un tasto non viene premuto
    mission.state = 8;
    mission.counter_case_8 = mission.counter_case_8 + 1;
    drone->user_control_flag = true;
  }
}




int main(int argc, char **argv)
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
  mission.enter_state = 0;
  int counter_user_control = 100;
  float gimbal_pitch_des = 0.0;
  
  //Define the vector for lat lon in case the mode with only two waypoints is used 
  XmlRpc::XmlRpcValue fixed_lat;
  XmlRpc::XmlRpcValue fixed_lon;
  std::vector<double> fixed_lat_v; //44.781715, 44.781720,
  std::vector<double> fixed_lon_v;  
 
  string folder_name;
 
  //float gimbal_yaw_des = 0.0;

  bool control_altitude_by_image_flag = false;

  bool enable_take_off = true;

  //LOAD PARAMS
  nh.getParam("/solar_param/z_des", drone.z_des);
  nh.getParam("/solar_param/initial_state", mission.state);
  nh.getParam("/solar_param/initial_enter_state", mission.enter_state);
  nh.getParam("/solar_param/desired_waiting_KF_init_it", desired_KF_waiting);
  nh.getParam("/solar_param/control_altitude_by_image", control_altitude_by_image_flag);
  nh.getParam("/solar_param/counter_user_control", counter_user_control);
  nh.getParam("/solar_param/Detection_param_Optimization_enabled", mission.Optimization_enabled);
  nh.getParam("/solar_param/enable_take_off", enable_take_off);
  nh.getParam("/solar_param/Gimbal_pitch", gimbal_pitch_des);
  nh.getParam("/solar_param/control_altitude_on_heigth_sensor", mission.enable_heigth_control_via_sensor);
  nh.getParam("/solar_param/desired_laser_heigth", mission.desired_laser_navigation_heigth);
  nh.getParam("/solar_param/use_only_two_fixed_waypoints_for_a_single_array_inspection", mission.use_two_single_fixed_waypoints_flag);
  nh.getParam("/solar_param/enable_velocity_navigation_control_flag", mission.enable_navigation_velocity_control_flag );
  nh.getParam("/solar_param/desired_navigation_velocity", drone.sat_x_vel );
  nh.getParam("/solar_param/desired_txt_folder_name", folder_name );
  nh.getParam("/solar_param/sensor_fixed_altitude", mission.altitude_fixed_flag);
  //nh.getParam("/solar_param/waypoints_lat", fixed_lat)
  nh.getParam("/solar_param/waypoints_lat", fixed_lat);
  ROS_ASSERT(fixed_lat.getType() == XmlRpc::XmlRpcValue::TypeArray);
  nh.getParam("/solar_param/waypoints_lon", fixed_lon);
  ROS_ASSERT(fixed_lon.getType() == XmlRpc::XmlRpcValue::TypeArray);

  //################### PARAMETERS ONLY FOR TESTING PURPOSES ########################ààà
  float KP_Y;
  float KD_Y;
  float KI_Y;
  nh.getParam("/solar_param/KP_Y", KP_Y);
  nh.getParam("/solar_param/KD_Y", KD_Y);
  nh.getParam("/solar_param/KI_Y", KI_Y);
  
  float KP_Z;
  float KD_Z;
  float KI_Z;
  nh.getParam("/solar_param/KP_Z", KP_Z);
  nh.getParam("/solar_param/KD_Z", KD_Z);
  nh.getParam("/solar_param/KI_Z", KI_Z);
  
  // ######################################
  

  if (mission.use_two_single_fixed_waypoints_flag == true)
  {
    //Cast the XmlRpcValue of fixed_lat and fixed_lon to double value 
    for (int32_t i = 0; i < fixed_lat.size(); ++i) 
    {
       ROS_ASSERT(fixed_lat[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
       ROS_ASSERT(fixed_lon[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
       cout << std::setprecision(precision) << " fixed_lat[i] : " << fixed_lat[i] << endl;
       cout << std::setprecision(precision) << " fixed_lon[i]: " << fixed_lon[i] << endl;
       double a = static_cast<double>(fixed_lat[i]);
       double b = static_cast<double>(fixed_lon[i]);
       fixed_lat_v.push_back(a);
       fixed_lon_v.push_back(b);
       
      }
    load_only_two_fixed_GPS_coordinates(fixed_lat_v, fixed_lon_v);
  }
  else
  {
    //Load GPS Coordinate from file txt
    load_GPS_coordinates();
  }



  // load_GPS_coordinates_with_GAZEBO_OFFSET(HOME_lat, HOME_lon);
  Kalman_Filter.pass_to_class_initialization_waiting_value(desired_KF_waiting);
  Kalman_Filter.Kalman_filter_initialization(0.0, 0.0, 0.0, 0.0);

  mission.GAZEBO_LINK = true; //abilitare quando ho connessione con drone in Gazebo Link per osservazioni

bool  gimbal_down_flag = true;

  //Define Gains Controller
  float Kp_z = KP_Z;//0.5;
  float Kd_z = KD_Z;//0.4;
  float Ki_z = KI_Z;
  float Kp_yaw = 0.6;
  float Kd_yaw = 0.3;
  float Kp_x = 1.0; //0.9;   -------> ORIGINAL 1.0
  float Kp_y = KP_Y; //0.2; ---> ORIGINAL 0.85
  float Kd_x = 0.55;
  float Kd_y = KD_Y; //0.45;
  float Ki_x = 0.0; //0.1; //0.9; //0.1
  float Ki_y = KI_Y; //0.1; //0.55; //0.1
  double integralx(0);
  double integraly(0);
  float dt = 0.01;
  float time = 0.0;
  //Initialize PID controller for x, y, z and yaw direction (dt, max_value, min_value, Kp, Kd, Ki)
  //Output of PID are desired velocities
  
  PID pid_x = PID(dt, 4, -4, Kp_x, Kd_x, Ki_x);
  PID pid_y = PID(dt, 1.5, -1.5, Kp_y, Kd_y, Ki_y);

  PID pid_z = PID(dt, 1, -1, Kp_z, Kd_z, Ki_z);
  PID pid_yaw = PID(dt, 0.3, -0.3, Kp_yaw, Kd_yaw, 0.01);

  // Saturation for only FOLLOW LINW MODE
  drone.sat_x_vel = 0.6;
  drone.sat_y_vel = 1;

  Eigen::Matrix2f R;
  Eigen::Matrix2f Px;

  R << 0.1, 0.0,
      0.0, 1;
  Px << 0.01, 0.0,
      0.0, 1.0;

  mission.R = R;
  mission.Px = Px;

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


//Create Directory for print txt files:
string stringpath = "/home/dji/DATA/dji_ws/simulation_data/Test_in_lab/TEST_1/" + folder_name + "/";
int status = mkdir(stringpath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
if (status != 0)
{
  cout << "[SOLAR CONTROL] Impossible to create folder to store txt output files" << endl;
}


     std::ofstream outFile1(stringpath + "x_vel.txt");
     std::ofstream outFile2(stringpath + "y_vel.txt");
     std::ofstream outFile3(stringpath + "des_x_vel.txt");
     std::ofstream outFile4(stringpath + "des_y_vel.txt");
     std::ofstream outFile5(stringpath + "control_x_coo.txt");
     std::ofstream outFile6(stringpath + "control_y_coo.txt");
     std::ofstream outFile7(stringpath + "a_est_GF.txt");
     std::ofstream outFile8(stringpath + "c_est_GF.txt");
     std::ofstream outFile9(stringpath + "a_obs_BF.txt");
     std::ofstream outFile10(stringpath + "c_obs_BF.txt");
     std::ofstream outFile11(stringpath + "a_obs_est_BF.txt");
     std::ofstream outFile12(stringpath + "c_obs_est_BF.txt");
     std::ofstream outFile13(stringpath + "x_pos.txt");
     std::ofstream outFile14(stringpath + "y_pos.txt");
     std::ofstream outFile15(stringpath + "X_target_setpoint_BF.txt");
     std::ofstream outFile16(stringpath + "Y_target_setpoint_BF.txt");
     std::ofstream outFile17(stringpath + "X_target_setpoint_GF.txt");
     std::ofstream outFile18(stringpath + "Y_target_setpoint_GF.txt");
     std::ofstream outFile19(stringpath + "a_obs_thermo_GF.txt");
     std::ofstream outFile20(stringpath + "c_obs_thermo_GF.txt");
     std::ofstream outFile21(stringpath + "a_obs_RGB_GF.txt");
     std::ofstream outFile22(stringpath + "c_obs_RGB_GF.txt");
     std::ofstream outFile23(stringpath + "vel_z.txt");
     std::ofstream outFile24(stringpath + "error_from_GPS_line.txt");
     std::ofstream outFile25(stringpath + "z_vel_des.txt");
     std::ofstream outFile26(stringpath + "z_pos_cmd.txt");
     std::ofstream outFile27(stringpath + "z_pixel_offset.txt");
     std::ofstream outFile28(stringpath + "Yaw_des.txt");
     std::ofstream outFile29(stringpath + "Yaw.txt");
     std::ofstream outFile30(stringpath + "error_from_vision_line.txt");
     std::ofstream outFile31(stringpath + "drone_current_latitude.txt");
     std::ofstream outFile32(stringpath + "drone_current_longitude.txt");
     std::ofstream outFile33(stringpath + "ultrasonic_offset.txt");
     std::ofstream outFile34(stringpath + "counter_RGB_detection.txt");
     std::ofstream outFile35(stringpath + "z_pos.txt");


  //Initialize Mission Switch
  //mission.state = 0; //0
  //mission.enter_state = 0;
  
   /*
   Timer per comandi YAW e PITCH telecamera 
    */
  auto start = high_resolution_clock::now();
  auto end = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  bool start_timer = true;
  bool save_data_flag = false;

  std_msgs::Bool user_control;
  int acquire_GPS_counter = 0;
  bool acquire_GPS_flag = true;
  int case_3_counter = 0;
  int case_8_counter = 0;
  int counter_altitude_init = 0;
  int counter_for_align_way_DEBUG = 4;
  ros::Rate r(30);
  while (nh.ok())
  {
   
    //Loop local variables
    float vel_x = 0.0;
    float vel_y = 0.0;
    float y = 0.0;

    //drone.z_des = 5.0;

    // cout << "latitude " << drone.current_gps.latitude << endl;
    switch (mission.state)
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
        mission.start_gps_location = mission.HOME_gps_location;
        mission.start_local_position = mission.HOME_local_position;

        //gimbal_yaw_des = 25.0 * M_PI/180;
        // cout << " mission.HOME_gps_location.lat: " << mission.HOME_gps_location.latitude << endl;
        // cout << " mission.HOME_gps_location.lon: " << mission.HOME_gps_location.longitude << endl;
        if (acquire_GPS_counter > 20)
        {
         
          acquire_GPS_flag = false;
          acquire_GPS_counter = 0;
        }
        acquire_GPS_counter = acquire_GPS_counter + 1;
      }
      else
      {
        if (enable_take_off == false)
        {
          //Save last drone altitude
          drone.last_position_z = drone.current_local_pos.z;
          drone.last_altitude =  drone.current_local_pos.z;
          mission.state = 8;
        }
        else
        {
          take_off(&drone, &pid_x, &pid_y, &pid_z, &pid_yaw);
        }
        if (mission.take_off_result == false)
        {
          break;
        }
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
      align_Yaw_with_starting_GPS_waypoint(&drone, &pid_z, &pid_yaw);
      //    }
      break;

    case 2:
      reaching_starting_position(&drone, &pid_x, &pid_y, &pid_z, &pid_yaw);

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

      // obtain_control_result == True quando viene ricevuto flag true dal topic /wait_for_user_control
      obtain_control_result = true;

      if (case_3_counter > counter_user_control)
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
          drone.z_des = drone.current_local_pos.z; //drone.altitude;

          //For safety reason, redeclaration of all the flags required for navigation
          mission.target_point = 2;

          mission.inbound_counter = 0;
          mission.target_reached = false;
          mission.breaking_counter = 0;
          //Initialize Drone Position --> Sort of transformation respect world frame placed in HOME POINT
          //The yaw evaluation is done automatically by the drone.

          //Save Last Z altitude
          drone.last_altitude = drone.current_local_pos.z;
          drone.last_position_z = drone.current_local_pos.z;
          mission.start_gps_location = drone.current_gps;
          mission.start_local_position = drone.current_local_pos;
          mission.referred_local_yaw = drone.yaw_in_Rad_sim; //Salvo lo yaw del drone nel nuovo local frame in modo da poter avere uno yaw di riferimento per le trasformazioni
          drone.yaw_des_rad = drone.yaw_in_Rad;              // waypoints.counter = waypoints.counter + 1; //decommentare solo se si arriva diretti a questo caso d atake off
          mission.target_yaw = drone.yaw_in_Rad;

          if (mission.Optimization_enabled == true)
          {
            mission.setTarget(drone.current_local_pos.x, drone.current_local_pos.y, drone.current_local_pos.z, drone.yaw_in_Rad_sim);
            mission.state = 6; //Go to wait for parameters Optimization case
          }
          else
          {
            // float start_lat = waypoints.GPS_waypoints_lat[counter_for_align_way_DEBUG];
            // float start_lon = waypoints.GPS_waypoints_lon[counter_for_align_way_DEBUG];
            // geometry_msgs::Vector3 target_offset_respect_drone_position_referred_to_local_position;
            // evaluate_local_offset_between_GPS_target_and_body_frame(start_lat, start_lon, drone.current_gps, drone.current_local_pos, target_offset_respect_drone_position_referred_to_local_position);
            // drone.yaw_des_rad = atan2(target_offset_respect_drone_position_referred_to_local_position.x, target_offset_respect_drone_position_referred_to_local_position.y);
            
            // counter_for_align_way_DEBUG = counter_for_align_way_DEBUG + 1;
            mission.state = 4;
          }
        }
      }
      case_3_counter = case_3_counter + 1;
      break;

    case 4:
      obtain_control_result = false;
      /*Navigation WIth KF */
      navigation(&drone, &Kalman_Filter, &pid_x, &pid_y, &pid_z, &pid_yaw);

      break;

    case 5:
      /*Jumping between panels*/
      jump_panels(&drone, &pid_x, &pid_y, &pid_z, &pid_yaw);
      break;

    case 6:
      /*Waiting For parameters Optimization Task */
      waiting_for_parameters_optimization(&drone, &pid_x, &pid_y, &pid_z, &pid_yaw);

      break;

    case 7:
      /*Test OUTDOOR */
      if (acquire_GPS_flag == true)
      {
        //SETTING HOME POSITION
        mission.HOME_gps_location = drone.current_gps;
        mission.HOME_local_position = drone.current_local_pos;
        mission.start_gps_location = mission.HOME_gps_location;
        mission.start_local_position = mission.HOME_local_position;

        cout << " mission.HOME_gps_location.lat: " << mission.HOME_gps_location.latitude << endl;
        cout << " mission.HOME_gps_location.lon: " << mission.HOME_gps_location.longitude << endl;
        if (acquire_GPS_counter > 20)
        {
          acquire_GPS_flag = false;
          acquire_GPS_counter = 0;
        }
        acquire_GPS_counter = acquire_GPS_counter + 1;
        mission.state = 5;
      }
      //*****************************************************************
      else
      {
        mission.state = 1;
      }
      break;

      //Caso relativo ai test nel quali è necessario stoppare il drone e farlo passare in modalità manuale
    case 8:
      obtain_user_control(&drone);

      break;

    

    }

    //----  ALTITUDE CONTROL BY IMAGES -------- //
    //Publish KF INIT FLAG for reset of the altitude width array
    std_msgs::Bool KF_initialization;
    if (mission.panel_array_initialization == true)
    {
      mission.flag_altitude_init = true;
    }

    if (mission.flag_altitude_init == true && control_altitude_by_image_flag == true)
    {
      cout << "Initialize PIXEL OFFSET for altitude control " << counter_altitude_init << endl;
      if (counter_altitude_init > 200)
      {
        mission.flag_altitude_init = false;
        mission.height_above_ground = true;
        counter_altitude_init = 0;
      }

      counter_altitude_init = counter_altitude_init + 1;
    }

    KF_initialization.data = mission.flag_altitude_init;
    drone.publish_KF_init(KF_initialization);

    //Publish OPT start flag
    if (mission.Optimization_enabled == true)
    {
      //Publish Flag to Pythons script fpr detection to start the detection
      std_msgs::Bool OPT_start;
      OPT_start.data = mission.start_optimization_task;
      drone.publish_OPT_START(OPT_start);
    }
    

    //Publish gimbal angle cmd
    if (start_timer == true)
    {
       start = high_resolution_clock::now();
       start_timer = false;
    }

    end =  high_resolution_clock::now();
    duration = duration_cast<microseconds>(end - start)/1000000;
    
    if (duration.count() > 1)
    {
       gimbal_pitch_des = -M_PI / 2;
       drone.publish_cmd_angle_gimbal(gimbal_pitch_des);
       start_timer = true;
    }
 

    
    
    
    //Publish a flag whe the drone is under user control
    std_msgs::Bool flag_user_control;
    flag_user_control.data = drone.user_control_flag;
    drone.publish_drone_under_user_control(flag_user_control);
    
    if (mission.enable_heigth_control_via_sensor == true)
    {
       cout<<"[INFO] ALTITUDE BASED ON HEIGTH SENSOR"  << endl;
    }
   
   //Evaluate acceleration on GF 
    drone.acceleration_GF();

    cout << "                               ##### TELEMETRY ####                                                               " << endl;
    cout << "                                                                                               " << endl;
    cout << " X_W : " << drone.current_local_pos.x << " Y_W: " << drone.current_local_pos.y << " Z_W:  " << drone.current_local_pos.z << endl;
    cout << "X_vel: " << drone.current_local_velocity.vector.x << " Y_vel: " <<  drone.current_local_velocity.vector.y <<endl;
    cout << "Des X Vel: " << drone.x_vel_cmd << " Des Y Vel: " << drone.y_vel_cmd  << " Sat X Vel: " <<   drone.sat_x_vel  << endl;
    cout << "ultrasonic_altitude_value: " << drone.ultrasonic_altitude_value << endl;
    cout << "YAW DES: " << drone.yaw_des_rad << " YAW: " << drone.yaw_in_Rad << endl;
    cout << "Drone Latitude: " << drone.current_gps.latitude << " Drone Longitude: " << drone.current_gps.longitude << endl;
    cout << "ERRROR FROM VISION: " << mission.error_from_vision_line << endl;
    // cout <<"target_x : " <<  mission.target_GPS_offset_x << " target_y: " << mission.target_GPS_offset_y << " target_z: " << mission.target_GPS_offset_z << endl;
    cout << "Cartesian Distance to NEXT GPS WAYPOINT : " << mission.cartesian_distance_2D << endl;

    cout << "                                   ####################                                             " << endl;
    

  
    outFile1 << drone.current_local_velocity.vector.x << "\n";
    outFile2 << drone.current_local_velocity.vector.y << "\n";
    outFile3 << drone.x_vel_cmd << "\n";
    outFile4 << drone.y_vel_cmd << "\n";
    outFile5 << drone.control_x_coo << "\n";
    outFile6 << drone.control_y_coo << "\n";
    outFile7 << drone.xh_[0] << "\n";
    outFile8 << drone.xh_[1] << "\n";
    outFile9 << drone.obs[0] << "\n";
    outFile10 << drone.obs[1] << "\n";
    outFile11 << drone.yh_[0] << "\n";
    outFile12 << drone.yh_[1] << "\n";
    outFile13 << drone.current_local_pos.x << "\n";
    outFile14 << drone.current_local_pos.y << "\n";
    outFile15 << mission.x_target << "\n";
    outFile16 << mission.y_target << "\n";
    outFile17 << drone.x_target << "\n";
    outFile18 << drone.y_target << "\n";
    outFile19 << drone.obs_thermo_GF[0] << "\n";
    outFile20 << drone.obs_thermo_GF[1] << "\n";
    outFile21 << drone.obs_RGB_GF[0] << "\n";
    outFile22 << drone.obs_RGB_GF[1] << "\n";
    outFile23 << drone.current_local_velocity.vector.z << "\n";
    outFile24 << waypoints.error_from_GPS_line << "\n";
    outFile25 << drone.z_vel_cmd << "\n";
    outFile26 << drone.z_pos_cmd << "\n";
    outFile27 << drone.offset << "\n";

    outFile28 << drone.yaw_des_rad << "\n";
    outFile29 << drone.yaw_in_Rad << "\n";
    outFile30 << mission.error_from_vision_line << "\n";

    outFile31 << drone.current_gps.latitude << "\n";
    outFile32 << drone.current_gps.longitude << "\n";
    outFile33 << drone.ultrasonic_altitude_value << "\n";
    outFile34 << mission.counter_RGB_detection << "\n";
    outFile35 << drone.current_local_pos.z << "\n";

    //Publish To gezebo Simm
    drone.publish_new_current_local_position(mission.end_panel_reached);
    //Publish flag to script detection python for define the user control
    drone.publish_obtain_control_flag(obtain_control_result);

    //Publish Mission Flag To script "take_overlapped_photos"
    std_msgs::Bool inNav;
    inNav.data = mission.inNavigation;
    drone.publish_flag_inNavigation(inNav);

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
    drone.flagParametersOptimization = false;
    drone.flagGimbalAngle = false;
    drone.flagKeyIsPressed = false;
    drone.flagAltitude = false;
    drone.flagInNavigation = false;
    drone.flagKFINIT = false;
    drone.flagUserDesiredVelocity = false;

    ros::spinOnce();
    r.sleep();
  }

  //  // Kalman_Filter.Kalman_Filter_calculate(0.0, 0.0, 0.0, 0.0,  0.0 ,  0.0 , 0.0 ,  0.0);

  //   ros::spinOnce();
  //   r.sleep();
  // }
  return 0;
}
 
