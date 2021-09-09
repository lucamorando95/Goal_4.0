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
#include "pid.h"


using namespace dji_osdk_ros;
using namespace std;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;


struct Mission
{
  int state = 0;
  //Target : GPS o P ogni inizio pannello
  float x_target_P1 = 0.0;
  float y_target_P1 = 0.0; 
  float x_target_P2 = 0.0;
  float y_target_P2 = 0.0; 
  vector<float> P1_target;
  vector<float> P2_target;

  //Waypoints GPS convertite in local frame
  Eigen::Vector2f  P1; //quando arrivo in P2 prendo una nuova coo GPS dal vettore la quale diventa la nuova P2 , mentre la precedente diventa P1
  Eigen::Vector2f P2;
  //Eigen::Vector2f Target;

//vettori per aggiornare posizini
  sensor_msgs::NavSatFix start_gps_location;
  geometry_msgs::Point start_local_position;
  geometry_msgs::Vector3 localOffset;   
  
  float x_target_I = 0.0;
  float y_target_I = 0.0;
  float z_target_I = 0.0;
  float yaw_I = 0.0;

  int target_point = 0.0;
  float cartesian_distance_err = 0.0;

  int count = 0;
  int navigation_iteration_count = 0;

  bool flag_end_point = false;
  bool flag_create_setpoint = false;
  bool panel_array_initialization = false;
  bool KF_Initialization = false;

} mission;

struct Waypoint_GPS {
    float delta = 0.0; //Fattore di traslazione rispetto panel frame (locato nel cnetro pannello 1) su asse x
    float eta = 0.0; ///Fattore di traslazione rispetto panel frame (locato nel cnetro pannello 1) su asse y
    float gamma = 0.0; //rotazione sistema di waypoint gps rispetto panel frame 
    //Coordinata punto 1 (riferimento) del GPS. Da inizializzare in main con coordinata P1_x P1_y del primo pannello
    float GPS1_x = 0.0;
    float GPS1_y = 0.0;

    float error_from_GPS_line = 0.0;
    
    vector<float> waypoints_x_coo_world_frame;
    vector<float> waypoints_y_coo_world_frame;
    vector<float> waypoints_x_coo_gps_frame;
    vector<float> waypoints_y_coo_gps_frame;

    float GPS_P1_lat = 0.0;
    float GPS_P1_lon = 0.0;
    float GPS_P2_lat = 0.0;
    float GPS_P2_lon = 0.0;


    //Vector to initialize the Kalamn filter state with the equation of the line passing through the GPS
    //Waypoints 
    vector<float> GPS_P1_waypoint; //Start P1 Waypoint 
    vector<float> GPS_P2_waypoint; //End P2 waypoint


} waypoints;


//Define Services required to safe fly
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;
ros::ServiceClient setup_camera_stream_client;

dji_osdk_ros::SetupCameraStream setupCameraStream_;



void convert_GPS_coo_to_local_coo(geometry_msgs::Vector3&  deltaNed, sensor_msgs::NavSatFix& start_GPS, float target_lat, float target_lon)
{
    //Calcolo offset tra posizone iniziale e attuale drone 
  double deltaLon = target_lon - start_GPS.longitude;
  double deltaLat = target_lat - start_GPS.latitude;

  //Ottengo Offset in metri dato offset in latitudine e longitudine 
  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude );
  //deltaNed.z = target.altitude - origin.altitude;  //--> Target altitude è l'altuitudine attuale del drone 
  ROS_INFO("##### target.altitude:  %f .",  target.altitude);
  
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

bool set_local_position()
{
  SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}


void set_target(float x_I, float y_I, float z_I, float Yaw)
{
    mission.x_target_I = x_I;
    mission.y_target_I = y_I;
    mission.z_target_I = z_I;
    mission.yaw_I = Yaw;

}

void step(Drone * drone, PID * pid_x, PID * pid_y, PID * pid_z, PID *pid_yaw)
{
    static int info_counter = 0;
    float speedFactor         = 2;
    float yawThresholdInDeg   = 2;

    float xCmd, yCmd, zCmd;
     
    //Calcolo Offset tra posizione iniziale di start e posizione attuale drone
    convert_GPS_coo_to_local_coo(localOffset, mission.start_gps_location, drone.current_gps.lat, drone.current_gps.lon); //Passo alla funzione il vettore vuoto dove salvare localOffset, la current_gps location e la start_gps_location
    //Offset rimanenente tra l'offset richiesto per raggiungere il target (target_offset_x -- per esempio su x) e la distanza percorsa dal punto di partenza (localOffset.x)
    double xOffsetRemaining = mission.x_target_I - mission.localOffset.x; 
    double yOffsetRemaining = mission.y_target_I- mission.localOffset.y;
    double zOffsetRemaining = mission.z_target_I- mission.localOffset.z;
  
    double yawDesiredRad     = deg2rad * mission.yaw_I;
    double yawThresholdInRad = deg2rad * yawThresholdInDeg;
    double yawInRad          = toEulerAngle(current_atti).z;
    
    //Cartesian distance respect target Point
    mission.cartesian_distance_err = sqrt(pow(xOffsetRemaining,2) + pow(yOffsetRemaining,2) + pow(zOffsetRemaining, 2));
    std::cout<<"Cartesian Distance: " << mission.cartesian_distance_er << std::endl;

    ROS_INFO("##### Target Psoition: x: %f, y: %f, z: %f ", drone.Drone_Target_x, drone.Drone_Target_y, drone.Drone_Target_z);

   //Calcolo Offset su ogni asse del frame mondo tra la posizone percorsa dal drone e quella che rimane per raggiu ngere il target
    std::cout<<"xOffsetRemaining: " << xOffsetRemaining << std::endl;
    std::cout<<"yOffsetRemaining: " << yOffsetRemaining << std::endl;
    std::cout<<"zOffsetRemaining: " << zOffsetRemaining << std::endl;

    //Rotate x_target, y_target, z_target in world frame
    
    //calcolare velocità con PID o direttamente dare la posizione di target nel controllo di posizione del drone 

    //pubblicare il messaggio 
    
  
    
    



    
}

int main(int argc, char** argv)
{
 
  ros::init(argc, argv, "solar_fligth_control_real");
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
  
  
  //##Load GPS Waypoints from txt file 
  //load_GPS_waypoints()
  //In questo contesto simulo delle coordinate del punto P1 e P2 che salvo in queste variabili. Nella versione vera i punti vanno salvati nei vettori waypoints
//   waypoints.GPS_P1_lat = ...;
//   waypoints.GPS_P1_lon = ...;
//   waypoints.GPS_P2_lat = ...;
//   waypoints.GPS_P2_lon = ...;
  //####

  //Define Gains Controller
    float Kp_z = 1.5;
    float Kd_z = 0.5;
    float Kp_yaw = 0.6;
    float Kd_yaw = 0.3;
    float Kp_x = 0.45;
    float Kp_y = 0.3;//0.2;
    float Kd_x = 0.01;
    float Kd_y = 0.02;
    float Ki_x = 0.15;//0.9; //0.1
    float Ki_y = 0.15;//0.55; //0.1

    double integralx(0);
    double integraly(0);
    float dt = 0.01;
    float time = 0.0;

    //Initialize PID controller for x, y, z and yaw direction (dt, max_value, min_value, Kp, Kd, Ki)
    //Output of PID are desired velocities
    
    PID pid_x = PID(dt, 3, -3, Kp_x, Kd_x, Ki_x);
    PID pid_y = PID(dt, 3, -3, Kp_y, Kd_y, Ki_y);
     
    PID pid_z = PID(dt, 3, -3, Kp_z, Kd_z, 0.01);
    PID pid_yaw = PID(dt, 1, -1, Kp_yaw, Kd_yaw, 0.01);


  
  
  //Required to make the sdk able to control the drone 
  bool obtain_control_result = obtain_control();
  
  

  //Set Position as Reference Frame ---> Posiziona frame per GO_HOME
  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

  // Converto coo target P1 e P2 da lat e lon nel local frame
   mission.start_gps_location = drone.current_gps;
   mission.start_local_position = drone.current_local_pos;
   
   //Convert GPS P1 to local pos
   convert_GPS_coo_to_local_coo(mission.localOffset, mission.start_gps_location, waypoints.GPS_P1_lat, waypoints.GPS_P1_lon);
   mission.P1 << mission.localOffset.x, mission.localOffset.y; //Ottengo coordinate locali punto P1 rispetto local frame
   
   //Convert GPS P1 to local pos
   convert_GPS_coo_to_local_coo(mission.localOffset, mission.start_gps_location, waypoints.GPS_P2_lat, waypoints.GPS_P2_lon);
   mission.P2 << mission.localOffset.x, mission.localOffset.y;

   //Initialize KF con l'equazione della retta passante tra P1 e P2
  float a_init = (mission.P2[1] -mission.P1[1])/(mission.P2[0] - mission.P1[0]);
  float c_init = ((-1*a_init * mission.P1[0]) + mission.P1[1]);
  
  KF.yaw = drone.yaw_in_Rad; //-0.0 * M_PI/180;
  KF.xh << a_init, c_init; //Inizializzare con valori retta a e c passante per i waypoint di inizio e fine pannello
  KF.Px << 4.0, 0.0, .0, 4.0; //Inizializzare con valore relativamente alto
  KF.R <<  0.2, 0.0,   //Inizializzo Con varianza relativa all'errore di misurazione --> nela relalta puo essere dato su base sperimentale dell'errore del GPS
          0.0, 0.25;


  mission.state = 0;

//Before entering the central loop command for take off
  if (takeoff() ) 
   {
       ROS_INFO_STREAM("Take OFF Successfull");
       ros::Duration(3.0).sleep(); 
       takeoff_result = true;
   }




while(nh.ok()) 
  {
    switch(mission.state)
        {
            case 0:
            //Take Off 
                float desired_altitude = 10;
                 if(takeoff_result)
                 {
                    set_target(0,0,desired_altitude, 0);

                    step(&drone, &pid_x, &pid_y, &pid_z, &pid_yaw);
                 }
               //start lo metto nel caso 1, raggiungere posizine iniziale 
                start(&panel, &pid_x, &pid_y, &pid_z, &pid_yaw);
            break;

            case 1: 
            //Nel caso 2 navigaziobe lungo il pannello
            //Reaching end of panel point P2 from point P1
               navigation(&panel, &pid_x, &pid_y, &pid_z, &pid_yaw);
            break;

            case 2:
            //Cambio di vela 
               jump_panels_array(&panel,  &pid_x, &pid_y, &pid_z, &pid_yaw);
            break;
        }
  
  }

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
