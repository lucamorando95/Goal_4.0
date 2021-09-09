
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>

#include "solar_flight_control.h"
#include "image_converter.h"
#include "Drone.h"
#include "KF.h"
#include <dji_sdk/dji_sdk_node.h>

using namespace dji_osdk_ros;
using namespace std;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

float x_respect_frame = 0.0;
float y_respect_frame = 0.0;
//For the streaming from the main or fpv camera launch the start_camera_stream robot


//Define Services required to safe fly
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;
ros::ServiceClient setup_camera_stream_client;

dji_osdk_ros::SetupCameraStream setupCameraStream_;


ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos; //--> ritorna una posizione xyz riferita alla posizione di home del drone 
geometry_msgs::Point drone_RGB_control_point1;
geometry_msgs::Point drone_RGB_control_point2;



Mission square_mission;

//Instantiate KALMAN FILTER class 
float a_target = 0.0; //Valore di m della retta vera del pannello --> stato reale non conosciuto nella realta 
float c_target = 0.3; //Valore di traslazione pannello reale rispetto asse y non conosciuto nella realtà


bool set_local_position()
{
  SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}


bool takeoff()
{
  DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = DroneTaskControl::Request ::TASK_TAKEOFF;
  drone_task_service.call(droneTaskControl);
  
  return droneTaskControl.response.result;
}

bool land()
{
   DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = DroneTaskControl::Request ::TASK_LAND;
  drone_task_service.call(droneTaskControl);

  return droneTaskControl.response.result;
}


//Function to rotate frame --> Rotate panels respect frame and Camera position respect frame 
void frame_rotation(float x_pos, float y_pos, float alfa)
 {
    x_respect_frame = x_pos * cos(alfa) - y_pos * sin(alfa); //drone_kf.drone_x;
    y_respect_frame = x_pos * sin(alfa) + y_pos * cos(alfa); //drone_kf.drone_y;
  }

//Function to rotate frame --> Rotate panels respect frame and Camera position respect frame 
void rotation_from_GF_to_BF(Eigen::Vector2f P, float x_pos_I, float y_pos_I, float alfa)
 {
    x_respect_frame = P[0] * cos(alfa) - P[1] * sin(alfa) - x_pos_I*cos(alfa) -y_pos_I * sin(alfa); //drone_kf.drone_x;
    y_respect_frame = -P[0] * sin(alfa) +  P[1]* cos(alfa) + x_pos_I * sin(alfa) - y_pos_I * cos(alfa); //drone_kf.drone_y;
 }





//Publish Estimated line to meke it visible on visual frame 
 void publish_estimated_line(float a_estimated,  float c_estimated, float x_pos_I, float y_pos_I, Drone drone)
 {
   //Find two points on Inertial frame lies to the estimated line 
   Eigen::Vector2f P1(0.0,0.0);
   Eigen::Vector2f P2(1.0,0.0);
  
   P1[1] =  c_estimated;
   P2[1] = a_estimated* P2[0] + c_estimated;

   //Rotate in body/camera frame the points
  rotation_from_GF_to_BF(P1, x_pos_I, y_pos_I, 0.0);
  rotation_from_GF_to_BF(P2, x_pos_I, y_pos_I, 0.0);
   
   //Publish Point 1 
   drone.P1_B = P1;
   drone.P2_B = P2;

   drone.publish_estimated_control_points();
  }
 

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/

void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
	//Calcolo offset tra posizone iniziale e attuale drone 
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  //Ottengo Offset in metri dato offset in latitudine e longitudine 
  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude );
  deltaNed.z = target.altitude - origin.altitude;  //--> Target altitude è l'altuitudine attuale del drone 
  ROS_INFO("##### target.altitude:  %f , origin.altitude : %f....",  target.altitude,  origin.altitude);
  
 // ROS_INFO("##### deltaLon %f ....", deltaLon);
  //ROS_INFO("##### deltaLat %f ....", deltaLat); 
 // ROS_INFO("##### deltaNed.x  %f ....", deltaNed.x );
  //ROS_INFO("##### deltaNed.y  %f ....", deltaNed.y );
  //ROS_INFO("##### deltaNed.z  %f ....", deltaNed.z );
  //Salvo in Vettore LKocalOffset offset in metri nel NEDframe
}


bool obtain_control()
{
  SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}


int main(int argc, char** argv)
{
 
  ros::init(argc, argv, "solar_fligth_control");
  ros::NodeHandle nh;
  

  //Call class for image converter and publish image over the air OTA
  ImageConverter ic;
  Drone drone;
  KalmanFilter KF;
 
  //Call for Drone class and drone related variables and subscribers 
 // Basic services
 
  drone_task_service         = nh.serviceClient<DroneTaskControl>("dji_osdk_ros/drone_task_control");
  query_version_service      = nh.serviceClient<QueryDroneVersion>("dji_osdk_ros/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<SetLocalPosRef> ("dji_osdk_ros/set_local_pos_ref");
  setup_camera_stream_client = nh.serviceClient<SetupCameraStream> ("dji_osdk_ros/setup_camera_stream");
  

  

  //ros::Subscriber attitudeSub = nh.subscribe("dji_osdk_ros/attitude", 10, &attitude_callback);
  //ros::Subscriber gpsSub      = nh.subscribe("dji_osdk_ros/gps_position", 10, &gps_callback);
  //ros::Subscriber flightStatusSub = nh.subscribe("dji_osdk_ros/flight_status", 10, &flight_status_callback);
 // ros::Subscriber displayModeSub = nh.subscribe("dji_osdk_ros/display_mode", 10, &display_mode_callback);
 // ros::Subscriber localPosition = nh.subscribe("dji_osdk_ros/local_position", 10, &local_position_callback);

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_osdk_ros/flight_control_setpoint_ENUposition_yaw", 10);
 
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_osdk_ros/flight_control_setpoint_generic", 10);
  

   /*
  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

 */

 
  /* 
   if (takeoff() ) 
   {
       ROS_INFO_STREAM("Take OFF Successfull");
       ros::Duration(3.0).sleep(); 
       takeoff_result = true;
   }
 */



 /*
  if(takeoff_result)
  {
    square_mission.reset();
	std::cout<<"current_gps: "<< current_gps << std::endl;
		
    square_mission.start_gps_location = current_gps;
	std::cout<<"start_gps_location: "<< square_mission.start_gps_location << std::endl;
    square_mission.start_local_position = current_local_pos; //Qui current_local_pos inizializzata come 0 0 0
    square_mission.setTarget(0, 20, 3, 0); // --> x, y, z, yaw
    square_mission.state = 1;
    ROS_INFO("##### Start route %d ....", square_mission.state);
  }
*/

  //Required to make the sdk able to control the drone 
  //bool obtain_control_result = obtain_control();
  //bool takeoff_result;
  

  //Set Position as Reference Frame ---> Posiziona frame per GO_HOME
  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }
 // #########################à INITIALIZATION OF KALMAN FILTER VARIABLES ###################
  //Define the position of the start and end of each panel respect the inertial frame --> expressed in meters respect the 
  //inertuial frame. Does not known in reality but only in simulation
  //---> Queste variabili potrfloat x_target_P1 = 0.0;
  
  //Le posizioni dei pannelli target e della camera rispetto al frqame fittizio sono sempre date con rotazione frame supossta  di 0.0.
  //Posso ruotare tutto sucessivamente 

   //Supponendo che script venga lanciato quando drone sul punto P1 dis tart con yaw allineato, 
   //Le coordinate del punto P1 sono le coordinate attuali del drone
  
  geometry_msgs::Vector3     localOffset;
  sensor_msgs::NavSatFix start_gps_location;
  geometry_msgs::Point start_local_position;
  
  start_gps_location = drone.current_gps;
  start_local_position = drone.current_local_pos;
  
  
  
  //Suppongo che il frame mondo venga piazzato esattamente sul punto P1 (supponendo che lo script venga lanciato appunto quando il drone in P1 e yaw allineato)
  float x_target_P1 = drone.current_local_pos.x - start_local_position.x;//0.35;  //Le coordinate sono del punto P1 = (0.3, 0.2) rispetto al world frame fittizio
  float y_target_P1 = drone.current_local_pos.y - start_local_position.y;
  float x_target_P2 =x_target_P1 + 10;  //Suppongo che il punto P2 si trovi a 10 metri lungo la x del frame mondo, che è allineato al frame del drone quando si trova in P1. 
  float y_target_P2 = drone.current_local_pos.y - start_local_position.y;
  float x_target_P1_rotated = 0.0; //Solamente in esperimenti in lab pr far finat che i pannelli siano ruotati rispetto al frame
  float y_target_P1_rotated = 0.0;
  float x_target_P2_rotated = 0.0;
  float y_target_P2_rotated = 0.0;
  float frame_rotation_rad = 0.0 * M_PI/180; //angolo va definito in radianti: + rotazione in basso, - rotazione in alto--_> guardare note ipad
   
  
  int target_point = 1; //Permette di definire la direzione del pannello

//Initialize drone position in world frame 

  //Initialize KF 
  float a_init = (y_target_P2 - y_target_P1)/(x_target_P2 - x_target_P1);
  float c_init = ((-1*a_init * x_target_P1) + y_target_P1);
  KF.yaw = drone.yaw_in_Rad; //-0.0 * M_PI/180;
  KF.xh << a_init, c_init; //Inizializzare con valori retta a e c passante per i waypoint di inizio e fine pannello
  KF.Px << 4.0, 0.0, .0, 4.0; //Inizializzare con valore relativamente alto
  KF.R <<  0.2, 0.0,   //Inizializzo Con varianza relativa all'errore di misurazione --> nela relalta puo essere dato su base sperimentale dell'errore del GPS
          0.0, 0.25;
  

  //Old variables P1 P2 RGB and P1 P2 Thermal
  float control_RGB_point1_x_old = 0.0;
  float control_RGB_point1_y_old = 0.0;
  float control_RGB_point2_x_old = 0.0;
  float control_RGB_point2_y_old = 0.0;
  
  
  float control_thermo_point1_x_old = 0.0;
  float control_thermo_point1_y_old = 0.0;
  float control_thermo_point2_x_old = 0.0;
  float control_thermo_point2_y_old = 0.0;

 
 


  bool control_P1_RGB_updated = false;
  bool control_P2_RGB_updated = false;
  bool control_P1_Thermo_updated = false;
  bool control_P2_Thermo_updated = false;
  
  int RGB_detection_count = 0;
  int Thermo_detection_count = 0;

  ros::Rate r(20); 
  while(nh.ok()) 
  {
  
  //Local offset ritorna distanza percorsa da punto di start
  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);
   drone.local_pos_I.x =  localOffset.x;  //drone position respect start_gps_location
  drone.local_pos_I.y =  localOffset.y; //drone.drone_local_pos_y - start_local_position_y; //1.56;
  
  //Calcolo distanza rimanenente da target P2
  float distance_from_P2 = sqrt(pow(x_target_P2 - drone.local_pos_I.x,2) + pow(y_target_P2 - drone.local_pos_I.y,2));
  //Calcolo distanza percorsa da target P1
  float distance_from_P1 = sqrt(pow(drone.local_pos_I.x,2) + pow(drone.local_pos_I.y,2));
  cout<<"Distance to P2: " << distance_from_P2 << endl;
  cout<<"Distance from P1: " << distance_from_P1 << endl;

  //Take observation from RGB camera 
  if (drone.control_RGB_point1_x != control_RGB_point1_x_old or drone.control_RGB_point1_y !=  control_RGB_point1_y_old)
  {
      KF.control_point1_x_b = drone.control_RGB_point1_x;
      KF.control_point1_y_b = drone.control_RGB_point1_y;
      RGB_detection_count = 0;
      control_P1_RGB_updated = true;
     
  }

  if (drone.control_RGB_point2_x != control_RGB_point2_x_old or drone.control_RGB_point2_y !=  control_RGB_point2_y_old)
  {
      KF.control_point2_x_b = drone.control_RGB_point2_x;
      KF.control_point2_y_b = drone.control_RGB_point2_y;
      RGB_detection_count = 0; // required to increase the frequency of updating
      control_P2_RGB_updated = true;
      
  }


     
  // cout<<"KF.control_point1_x_b: "<< KF.control_point1_x_b <<"   KF.control_point1_y_b  " <<  KF.control_point1_y_b << endl;

  //If required rotate inertial frame--> Rotate target P1

  //  frame_rotation( x_target_P1, y_target_P1, frame_rotation_rad);
  //  x_target_P1_rotated = x_respect_frame;
  //  y_target_P1_rotated = y_respect_frame;
  //  cout<<"x_target_P1: "<<x_target_P1_rotated <<"  y_target_P1: " << y_target_P1_rotated<< endl;
   
  //  frame_rotation( x_target_P2, y_target_P2,  frame_rotation_rad);
  //  x_target_P2_rotated = x_respect_frame;
  //  y_target_P2_rotated = y_respect_frame;
  //  cout<<"x_target_P2: "<<x_target_P2_rotated <<"  y_target_P2: " << y_target_P2_rotated<< endl;
  

   //Rotate robot position respect Inertial frame
   frame_rotation( x_drone_I,   y_drone_I, frame_rotation_rad);
   KF.translation_x = x_drone_I;// x_respect_frame; //Translation camera respect Inertial frame
   KF.translation_y = y_drone_I; //y_respect_frame;
  
  
  //KF_RGB.yaw = frame_rotation_rad; //Se frame è ruotato e drone allineato con pannelli è come se fosse ruotato nel senso opposto
 
 //Update KF with RGB observations 
  if  (control_P1_RGB_updated == true and control_P1_RGB_updated  == true or RGB_detection_count < 20)
  {
    //Update only if detected something
      KF.Kalman_Filter(x_target_P1, y_target_P1, x_target_P2, y_target_P2, target_point); //nel caso reale lo stato vero non è conosciuto quindi miodificare funzione 
      cout << "[KF RGB UPDATING] #############################################à" << endl;
      cout<< "[KALMAN FILTER RGB] a_target^I : "<< KF.a_target << " c_target^I: "<< KF.c_target << endl;
      cout<< "[KALMAN FILTER RGB] a_obs^I : "<< KF.a_obs <<"    c_obs^I: "<< KF.c_obs << endl;
  }

  
  //Take observation from Thermal camera 
  if (drone.control_thermo_point1_x != control_thermo_point1_x_old or drone.control_thermo_point1_y !=  control_thermo_point1_y_old)
  {
      KF.control_point1_x_b = drone.control_thermo_point1_x;
      KF.control_point1_y_b = drone.control_thermo_point1_y;
      Thermo_detection_count = 0;
      control_P1_Thermo_updated = true;
  }

  //Take observation from Thermal camera 
  if (drone.control_thermo_point2_x != control_thermo_point2_x_old or drone.control_thermo_point2_y !=  control_thermo_point2_y_old)
  {
      KF.control_point2_x_b = drone.control_thermo_point2_x;
      KF.control_point2_y_b = drone.control_thermo_point2_y;
      Thermo_detection_count = 0;
      control_P2_Thermo_updated = true;
  }

//Update KF with Thermal  observations if available
  if  (control_P1_Thermo_updated == true and control_P2_Thermo_updated  == true or Thermo_detection_count < 20)
  {
    //Update only if detected something
      KF.Kalman_Filter(x_target_P1, y_target_P1, x_target_P2, y_target_P2, target_point);
      cout << "[KF THERMAL UPDATING] #############################################" << endl;
      cout<< "[KALMAN FILTER THERMAL ] a_obs^I : "<< KF.a_obs <<"    c_obs^I: "<< KF.c_obs << endl;
  }

 //Print Kalman Filter estimated state value
  cout<< "[KALMAN FILTER] a_target^I : "<< KF.a_target << " c_target^I: "<< KF.c_target << endl;
  std::cout<<"[KALMAN FILTER] ---> a estimated: "<< KF.xh[0]<< std::endl;
  std::cout<<"[KALMAN FILTER] ---> c estimated: "<< KF.xh[1]<< std::endl;
  
  
  //Publish estimated state value 
  publish_estimated_line(KF.xh[0],  KF.xh[1], x_drone_I, y_drone_I, drone);
 
  


  //Update variables 
  control_RGB_point1_x_old = drone.control_RGB_point1_x;
  control_RGB_point1_y_old = drone.control_RGB_point1_y;
  control_RGB_point2_x_old = drone.control_RGB_point2_x;
  control_RGB_point2_y_old = drone.control_RGB_point2_y;

  control_thermo_point1_x_old = drone.control_thermo_point1_x;
  control_thermo_point1_y_old = drone.control_thermo_point1_y;
  control_thermo_point2_x_old = drone.control_thermo_point2_x;
  control_thermo_point2_y_old = drone.control_thermo_point2_y;

  //Flags
  drone.flagDroneRGBControlPoint1 = false;
  drone.flagDroneRGBControlPoint2 = false;
 
  control_P1_RGB_updated = false;
  control_P2_RGB_updated = false;
  control_P1_Thermo_updated = false;
  control_P2_Thermo_updated = false;


  RGB_detection_count = RGB_detection_count + 1;
  Thermo_detection_count = Thermo_detection_count + 1;
  
  ros::spinOnce();
  r.sleep();
  }
  return 0;
}




//La parte da quii in poi viene chiamata da funzioni commentate che al momento non mi servono







// geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
// {
//   geometry_msgs::Vector3 ans;

//   tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  // R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
//   return ans;
// }

// void Mission::step()
// {
//   static int info_counter = 0;
//   geometry_msgs::Vector3     localOffset; //Alloco un vettore x,y,z che servirà per definire il localOffset rispetto la poszione di home 

  // float speedFactor         = 2;
  // float yawThresholdInDeg   = 2;

  // float xCmd, yCmd, zCmd;

  // localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location); //Passo alla funzione il vettore vuoto dove salvare localOffset, la current_gps location e la start_gps_location
  Offset rimanenente tra l'offset richiesto per raggiungere il target (target_offset_x -- per esempio su x) e la distanza percorsa dal punto di partenza (localOffset.x)
  // double xOffsetRemaining = target_offset_x - localOffset.x; 
  // double yOffsetRemaining = target_offset_y - localOffset.y;
  // double zOffsetRemaining = target_offset_z - localOffset.z;

  // double yawDesiredRad     = deg2rad * target_yaw;
  // double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  // double yawInRad          = toEulerAngle(current_atti).z;

  // info_counter++;
  // if(info_counter > 25)
  // {
  //   info_counter = 0;
  //   ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    // ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
//   }
//  //Sorta di saturazione 
//   if (abs(xOffsetRemaining) >= speedFactor)
//     xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  // else
  //  //Il comando sulla x è relativo all'offset remaining
  //   xCmd = xOffsetRemaining;

  // if (abs(yOffsetRemaining) >= speedFactor)
  //   yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  // else
  //   yCmd = yOffsetRemaining;

  // zCmd = start_local_position.z + target_offset_z;


  // /*!
  //  * @brief: if we already started breaking, keep break for 50 sample (1sec)
  //  *         and call it done, else we send normal command
  //  */

  // if (break_counter > 50)
  // {
  //   ROS_INFO("##### Route %d finished....", state);
  //   finished = true;
  //   return;
  // }
  // else if(break_counter > 0)
  // {
  //   sensor_msgs::Joy controlVelYawRate;
  //   uint8_t flag = (Control::VERTICAL_VELOCITY   |
  //               Control::HORIZONTAL_VELOCITY |
  //               Control::YAW_RATE            |
  //               Control::HORIZONTAL_GROUND   |
  //               Control::STABLE_ENABLE);
  //   controlVelYawRate.axes.push_back(0);
  //   controlVelYawRate.axes.push_back(0);
  //   controlVelYawRate.axes.push_back(0);
  //   controlVelYawRate.axes.push_back(0);
  //   controlVelYawRate.axes.push_back(flag);

  //   ctrlBrakePub.publish(controlVelYawRate);
  //   break_counter++;
  //   return;
  // }
  // else //break_counter = 0, not in break stage
  // {
    
  //  //Pubblico sul topic controlPosYaw
  //   sensor_msgs::Joy controlPosYaw;

    
  //   controlPosYaw.axes.push_back(xCmd);
  //   controlPosYaw.axes.push_back(yCmd);
  //   controlPosYaw.axes.push_back(zCmd);
  //   controlPosYaw.axes.push_back(yawDesiredRad);
  //   ctrlPosYawPub.publish(controlPosYaw);
  // }

  // if (std::abs(xOffsetRemaining) < 0.5 &&
      // std::abs(yOffsetRemaining) < 0.5 &&
      // std::abs(zOffsetRemaining) < 0.5 &&
      // std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
  // {
  //   //! 1. We are within bounds; start incrementing our in-bound counter
  //   inbound_counter ++;
  // }
  // else
  // {
  //   if (inbound_counter != 0)
  //   {
  //     //! 2. Start incrementing an out-of-bounds counter
  //     outbound_counter ++;
  //   }
  // }

  // //! 3. Reset withinBoundsCounter if necessary
  // if (outbound_counter > 10)
  // {
  //   ROS_INFO("##### Route %d: out of bounds, reset....", state);
//     inbound_counter  = 0;
//     outbound_counter = 0;
//   }

//   if (inbound_counter > 50)
//   {
//     ROS_INFO("##### Route %d start break....", state);
//     break_counter = 1;
//   }

// }

// bool takeoff_land(int task)
// {
//   DroneTaskControl droneTaskControl;

//   droneTaskControl.request.task = task;

//   drone_task_service.call(droneTaskControl);

//   if(!droneTaskControl.response.result)
//   {
//     ROS_ERROR("takeoff_land fail");
//     return false;
//   }

//   return true;
// }

// bool obtain_control()
// {
//   SDKControlAuthority authority;
//   authority.request.control_enable=1;
//   sdk_ctrl_authority_service.call(authority);

//   if(!authority.response.result)
//   {
//     ROS_ERROR("obtain control failed!");
//     return false;
//   }

//   return true;
// }



// void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
// {
//   current_atti = msg->quaternion;
// }

// void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
// {
//   current_local_pos = msg->point;
// }

// /*

// //Ogni volta che vien chiamata la gps callback viene aggiornato lo stato della missione
// void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
// {
//   static ros::Time start_time = ros::Time::now();
  // ros::Duration elapsed_time = ros::Time::now() - start_time;
  // current_gps = *msg; //Da informazioni riguardo lat e lon posizione attuale drone 

  // // Down sampled to 50Hz loop
  // if(elapsed_time > ros::Duration(0.02))
  // {
  //   start_time = ros::Time::now();
  //   switch(square_mission.state)
  //   {
  //     case 0:
  //       break;

  //     case 1:
  //       if(!square_mission.finished)
  //       {
  //        //Chiamo la funzione step() se siamo in caso 1 e la funzione è diversa da finished
        //   square_mission.step();
        // }
        // else
        // {
        //   square_mission.reset();
        //   square_mission.start_gps_location = current_gps;
        //   square_mission.start_local_position = current_local_pos;
        //   square_mission.setTarget(20, 0, 0, 0);
      //     square_mission.state = 2;
      //     ROS_INFO("##### Start route %d ....", square_mission.state);
      //   }
      //   break;

      // case 2:
      //   if(!square_mission.finished)
      //   {
      //     square_mission.step();
      //   }
      //   else
      //   {
      //     square_mission.reset();
      //     square_mission.start_gps_location = current_gps;
      //     square_mission.start_local_position = current_local_pos;
      //     square_mission.setTarget(0, -20, 0, 0);
      //     square_mission.state = 3;
      //     ROS_INFO("##### Start route %d ....", square_mission.state);
      //   }
      //   break;
      // case 3:
      //   if(!square_mission.finished)
      //   {
      //     square_mission.step();
      //   }
      //   else
      //   {
      //     square_mission.reset();
      //     square_mission.start_gps_location = current_gps;
      //     square_mission.start_local_position = current_local_pos;
      //     square_mission.setTarget(-20, 0, 0, 0);
//           square_mission.state = 4;
//           ROS_INFO("##### Start route %d ....", square_mission.state);
//         }
//         break;
//       case 4:
//         if(!square_mission.finished)
//         {
//           square_mission.step();
//         }
//         else
//         {
//           ROS_INFO("##### Mission %d Finished ....", square_mission.state);
//           square_mission.state = 0;
//         }
//         break;
//     }
//   }
// }
// */
// void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
// {
//   flight_status = msg->data;
// }

// void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
// {
//   display_mode = msg->data;
// }


// /*!
//  * This function demos how to use the flight_status
//  * and the more detailed display_mode (only for A3/N3)
//  * to monitor the take off process with some error
//  * handling. Note M100 flight status is different
//  * from A3/N3 flight status.
//  */



// bool set_local_position()
// {
// //Chiama il servizio nel quale la posizione attuale GPS del drone viene fissata come quella di home
//   SetLocalPosRef localPosReferenceSetter;
//   set_local_pos_reference.call(localPosReferenceSetter);
//   return localPosReferenceSetter.response.result;
// }

