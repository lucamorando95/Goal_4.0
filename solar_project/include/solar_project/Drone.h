

#ifndef DRONE_H
#define DRONE_H



// ROS includes
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
//#include "sensor_msgs/msg/Joy.hpp"
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h> 
//#include <dji_sdk/dji_sdk_node.h>
#include <dji_osdk_ros/Gimbal.h>
#include <solar_project/KEY_PRESSED.h>
// DJI SDK includes
#include <dji_osdk_ros/DroneTaskControl.h>
#include <dji_osdk_ros/SDKControlAuthority.h>
#include <dji_osdk_ros/QueryDroneVersion.h>
#include <dji_osdk_ros/SetLocalPosRef.h>
#include <dji_osdk_ros/SetupCameraStream.h>
#include <dji_osdk_ros/VOPosition.h> 
#include <dji_osdk_ros/CameraAction.h>

#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/mission.h"
// DJI SDK includes
/*
#include <dji_osdk_ros/DroneTaskControl.h>
#include <dji_osdk_ros/SDKControlAuthority.h>
#include <dji_osdk_ros/QueryDroneVersion.h>
#include <dji_osdk_ros/SetLocalPosRef.
*/

/* 
Classe nella quale sono presenti tutti i metodi e le funioni relativi alla struct drone
*/
#define C_PI (double)3.141592653589793
using namespace dji_osdk_ros;

class Drone
{
  ros::NodeHandle nh_;
  //Services
  //Define Services required to safe fly
  ros::ServiceClient set_local_pos_reference;
  ros::ServiceClient sdk_ctrl_authority_service;
  ros::ServiceClient drone_task_service;
  ros::ServiceClient query_version_service;
  ros::ServiceClient setup_camera_stream_client;
  ros::ServiceClient GimbalAction;
  ros::ServiceClient take_photo_service;
  
  //SetupCameraStream setupCameraStream_;
  


  ros::Subscriber control_RGB_point_1;
  ros::Subscriber control_RGB_point_2;
  ros::Subscriber control_thermo_point_1;
  ros::Subscriber control_thermo_point_2;
  ros::Subscriber height_above_take_off;

  //Subscribers to Drone Telemetry callback For real flight
  ros::Subscriber flight_statusSub;
  ros::Subscriber display_modeSub;
  ros::Subscriber attitudeSub ;
  ros::Subscriber gpsSub;
  ros::Subscriber localPosition;
  ros::Subscriber local_velocity;
  ros::Subscriber angular_velocity;
  ros::Subscriber gimbal_angle;
  ros::Subscriber vo_position;

  //Subscribe to altitude control offset from RGB script
  ros::Subscriber altitude_offset;
  ros::Subscriber ultrasonic_data;
   
  //Subscribe to receive Optimization completed flag
  ros::Subscriber parameter_opt; 
  
  //Subscribe to keyboard input to start stop the drone 
  ros::Subscriber keyboard_input;
  ros::Subscriber inNavigation;

   
  //Subscribe to the array number topic every time the control by the user is rerleased 
  ros::Subscriber Array_Number; 
  // Subscribe to the desired velocity selected by the user 
  ros::Subscriber user_des_vel;
  //TEST ONLY
  ros::Subscriber user_des_vel_y;

  ros::Subscriber KFInit;
  ros::Publisher GimbalAngle_cmd;  
  ros::Publisher pub_P1_estimated;
  ros::Publisher pub_P2_estimated;
   
  // Publish the control signal
  ros::Publisher ctrlPosYawPub; 
  ros::Publisher ctrlVelYawPub;
  
  ros::Publisher new_local_position;
  ros::Publisher user_joy_control_obtained;
  //#### Gazebo link Publish and Subscribe
  ros::Publisher Gazebo_P1_thermo_pub;
  ros::Publisher GAZEBO_Point_D_in_BF;

  //Publish to RGB image detector The  KF initialization time to take the  reference for the altitude control by images 
  ros::Publisher KF_init; 
  
  ros::Publisher inNav;
  //Publish Optimization Start
  ros::Publisher start_OPT_task;

  //Publish a flag == True when the drone is under user control 
  ros::Publisher under_user_control;
  //Private Variables rlative to the subscribers selected
  //Variables to verify if new values are received from the control point RGB topic
 

public:
  
  //Acquire drone control points expressed in body frame from RGB images 
  float control_RGB_point1_x;
  float control_RGB_point1_y;
  float control_RGB_point2_x;
  float control_RGB_point2_y;

  float control_thermo_point1_x;
  float control_thermo_point1_y;
  float control_thermo_point2_x;
  float control_thermo_point2_y;  

  //THERMO AND RGB observation rotated to local drone frame 
  float thermo_control_obs_P1_x_local;
  float thermo_control_obs_P1_y_local;
  float thermo_control_obs_P2_x_local;
  float thermo_control_obs_P2_y_local;

  float RGB_control_obs_P1_x_local;
  float RGB_control_obs_P1_y_local;
  float RGB_control_obs_P2_x_local;
  float RGB_control_obs_P2_y_local;


   
// General variable for Observation rotation in drone body frame
  float check_x_local;
  float check_y_local; 

  float check_x_b;
  float check_y_b;

// KF state estimation in local and body frame 
  Eigen::Vector2f xh_;
  Eigen::Vector2f obs;
  Eigen::Vector2f xh_body;
  Eigen::Vector2f yh_;
  Eigen::Vector2f obs_thermo_GF;
  Eigen::Vector2f obs_RGB_GF;

  float z_des;
  float offset;
  float last_position_z;
  std_msgs::Float32 ultrasonic_offset; //Offset given by the ultrasonic data
  std_msgs::Float32 reference_altitude_value; //reference value for the ultrasonic data
  
  //Command for position control expressed respect local offset 
  float x_pos_cmd;
  float y_pos_cmd;
  float z_pos_cmd;
  //std_msgs::Float32 ultrasonic_altitude_value; //make global the value obtained from the ultrasonic sensors
  float ultrasonic_altitude_value;

  float x_vel_cmd;
  float y_vel_cmd;
  float z_vel_cmd;

  float yaw_rate_cmd;
 
  float sat_x_vel;
  float sat_y_vel;
  
  //Acceleration 
  float x_ddot_GF = 0.0;
  float y_ddot_GF = 0.0;
  float z_ddot_GF = 0.0;
 
  float old_x_dot_GF = 0.0;
  float old_y_dot_GF  = 0.0;
  float old_z_dot_GF  = 0.0;
  //Drone Telemetry variables 
  uint8_t flight_status_data = 255;
  uint8_t display_mode_data = 255;
  
  float yaw_in_Rad;
  float yaw_in_Rad_sim; //Come in simulazione DJI, con zero allineato a punto di decollo --> vedi IPAD note
  float yaw_des_rad; //usare in ENU setpoint control
  float yaw_des_rad_sim; //Des yaw coerente con yaw_in_Rad_sim
  float yaw_vel;
  float yaw_err_rad;
  float yaw_in_rad_rel;

  float altitude = 0.0;
  float last_altitude = 0.0;

  //Control point D coordinates from GPS 
  float control_x_coo = 0.0;
  float control_y_coo = 0.0;

  float x_target = 0.0;
  float y_target = 0.0;
  float z_target = 0.0;

//Coo referrred from new_local_frame to HOME Local
  float x_target_HOME = 0.0;
  float y_target_HOME = 0.0;

//Define variable to store array number 
 int array_number = 0;
 float desired_velocity = 0;
 float desired_velocity_y = 0.0;

  //Topic message 
  geometry_msgs::Quaternion current_atti;
  sensor_msgs::NavSatFix current_gps;
  geometry_msgs::Point current_local_pos; //Posizione local ripsetto home frame 
  geometry_msgs::Point local_pos_I; //posizione rispetto frame fissato tra un target e il sucessivo
  geometry_msgs::Point new_current_local_pos;
  geometry_msgs::Vector3 local_drone_position; //Posizione attuale del drone rispetto a frame di riferimento dopo trasformazione da coo GPS
  geometry_msgs::Vector3Stamped current_local_velocity;
  geometry_msgs::Vector3Stamped current_angular_velocity;
  geometry_msgs::Vector3Stamped gimbalAngle;
  geometry_msgs::Vector3 target_offset_respect_drone_position;
  geometry_msgs::Vector3 target_offset_respect_local_position;
  geometry_msgs::Vector3 target_offset_respect_HOME_position;

  dji_osdk_ros::VOPosition vo_position_msg;
  //point lies on the estimated line expresse in body frame 
  Eigen::Vector2f P1_B;
  Eigen::Vector2f P2_B;
  
  //Counter 
  int image_control_count = 0; //---> Permette al drone di seguire la linea stimata da KF per tot iterazioni anche se non riceve piu osservazioni.
  int THERMO_image_control_count = 6000;
  int RGB_image_control_count = 6000;
  

  bool user_control_flag = false;
  bool Optimization_completed = false;
  bool init_from_KF = false;
  bool key_for_control_is_pressed = false;
  bool Navigation_flag = false;
  bool KF_init_flag = false;
  bool flagDroneRGBControlPoint1 = false; 
  bool flagDroneRGBControlPoint2 = false;
  bool flagDroneThermoControlPoint1 = false;
  bool flagDroneThermoControlPoint2 = false;
  bool flagDroneAttitude = false;
  bool flagDroneGPSpos = false;
  bool flagDroneLocalpos = false;
  bool flagDroneLocalvel = false;
  bool flagDroneAngularVel = false; 
  bool flagAltitudeOffset = false;
  bool flagUltrasonic = false;
  bool flagParametersOptimization = false;
  bool flagGimbalAngle = false;
  bool flagKeyIsPressed = false;
  bool flagAltitude = false;
  bool flagInNavigation = false;
  bool flagKFINIT = false;
  bool flagArraynumber = false;
  bool flagUserDesiredVelocity = false;
  //TEST ONLY
  bool flagUserDesiredVelocity_y = false;

  Drone()
  {
    //Services 
    drone_task_service         = nh_.serviceClient<DroneTaskControl>("dji_osdk_ros/drone_task_control");
    query_version_service      = nh_.serviceClient<QueryDroneVersion>("dji_osdk_ros/query_drone_version");
    set_local_pos_reference    = nh_.serviceClient<SetLocalPosRef> ("dji_osdk_ros/set_local_pos_ref");
    setup_camera_stream_client = nh_.serviceClient<SetupCameraStream> ("dji_osdk_ros/setup_camera_stream");
    sdk_ctrl_authority_service = nh_.serviceClient<SDKControlAuthority> ("dji_osdk_ros/sdk_control_authority");
    take_photo_service         = nh_.serviceClient<CameraAction> ("dji_osdk_ros/camera_action");

    //Topics 
    control_RGB_point_1 = nh_.subscribe("/RGB_control_point_1", 10, &Drone::drone_RGB_control_point1_callback, this);
    control_RGB_point_2 = nh_.subscribe("/RGB_control_point_2", 10, &Drone::drone_RGB_control_point2_callback, this);
    control_thermo_point_1 = nh_.subscribe("/Thermo_control_point_1", 10, &Drone::drone_thermo_control_point1_callback, this); //da cambiare 
    control_thermo_point_2 = nh_.subscribe("/Thermo_control_point_2", 10, &Drone::drone_thermo_control_point2_callback, this);
    
    flight_statusSub = nh_.subscribe("dji_osdk_ros/flight_status", 10, &Drone::flight_status_callback,this);
    display_modeSub = nh_.subscribe("dji_osdk_ros/display_mode", 10, &Drone::display_mode_callback, this);
    attitudeSub = nh_.subscribe("/dji_osdk_ros/attitude", 10, &Drone::drone_attitude_callback, this);
    gpsSub      = nh_.subscribe("dji_osdk_ros/gps_position", 10, &Drone::drone_gps_callback, this);
    localPosition = nh_.subscribe("dji_osdk_ros/local_position", 10, &Drone::drone_local_position_callback, this);
    local_velocity = nh_.subscribe("dji_osdk_ros/velocity", 10, &Drone::drone_velocity_callback,this);
    angular_velocity = nh_.subscribe("/dji_osdk_ros/angular_velocity_fused",10, &Drone::drone_angular_velocity_callback,this);
    altitude_offset= nh_.subscribe("/dji_osdk_ros/altitude_offset", 5, &Drone::altitude_offset_callbak,this);
    parameter_opt = nh_.subscribe("/dji_osdk_ros/parameter_opt_completed", 5,  &Drone::parameter_opt_callback, this);
    gimbal_angle = nh_.subscribe("/dji_osdk_ros/gimbal_angle", 5,  &Drone::gimbalangle_callback, this);
    vo_position = nh_.subscribe("/dji_osdk_ros/vo_position", 5, &Drone::VoPosition_callback, this);

    height_above_take_off = nh_.subscribe("/dji_osdk_ros/height_above_takeoff", 5,  &Drone::ultrasonic_callback, this);
    keyboard_input = nh_.subscribe("/dji_osdk_ros/user_keyboard_input", 5,  &Drone::wait_user_keyboard_input, this);
    
    inNavigation = nh_.subscribe("/dji_osdk_ros/inNav_flag", 5, &Drone::inNavigation_flag_callback, this);
    KFInit =  nh_.subscribe("/dji_osdk_ros/KF_init", 5, &Drone::KF_init_flag_callback, this);

    //Flag relative to array number 
    Array_Number = nh_.subscribe("/dji_osdk_ros/key_pressed", 5,  &Drone::array_number_callback, this);

    //Receive desired drone veolocity 
    user_des_vel =  nh_.subscribe("/dji_osdk_ros/user_desired_velocity", 5,  &Drone::user_desired_velocity_callback, this);
    //TEST ONLY
    user_des_vel_y =  nh_.subscribe("/dji_osdk_ros/user_desired_velocity_y", 5,  &Drone::user_desired_velocity_y_callback, this);

    //ultrasonic_data = nh_.subscribe("/dji_osdk_ros/height_above_takeoff", 10, &Drone::altitude_ultrasonic_data,this);
    GimbalAngle_cmd = nh_.advertise<dji_osdk_ros::Gimbal>("/dji_osdk_ros/gimbal_angle_cmd",1);
    pub_P1_estimated  = nh_.advertise<geometry_msgs::Point>("/P1_estimated_control_point",1);
    pub_P2_estimated =  nh_.advertise<geometry_msgs::Point>("/P2_estimated_control_point",1);

    ctrlPosYawPub = nh_.advertise<sensor_msgs::Joy>("dji_osdk_ros/flight_control_setpoint_ENUposition_yaw", 10);
    ctrlVelYawPub = nh_.advertise<sensor_msgs::Joy>("dji_osdk_ros/flight_control_setpoint_ENUvelocity_yawrate", 10);

    new_local_position =  nh_.advertise<geometry_msgs::Point>("/dji_osdk_ros/new_current_local_position",10);
    user_joy_control_obtained = nh_.advertise<std_msgs::Bool>("/dji_osdk_ros/user_joy_control_obtained",10);
    // Optimization Start 
    start_OPT_task =  nh_.advertise<std_msgs::Bool>("/dji_osdk_ros/optimization_start",30);
   
   //Drone under user control
   under_user_control = nh_.advertise<std_msgs::Bool>("/dji_osdk_ros/drone_is_under_user_control",30);
  //########### GAZEBO LINK #####
    GAZEBO_Point_D_in_BF =  nh_.advertise<geometry_msgs::Point>("Gazebo_link/point_D_for_control_in_BF", 10);
    KF_init =  nh_.advertise<std_msgs::Bool>("/dji_osdk_ros/KF_init",30);

  //Publish flag to take photo script when the drone is in  navigation 
    inNav =  nh_.advertise<std_msgs::Bool>("/dji_osdk_ros/inNav_flag",30);
  }

    
//Define Drone Class Desctructor
  ~Drone()
  {
  
  }


// ####################### Services  ################################
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

bool release_control()
{
  SDKControlAuthority authority;
  authority.request.control_enable=0;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}



bool setup_camera_stream_main_cam()
{
  SetupCameraStream setupCameraStream_;
  setupCameraStream_.request.cameraType = setupCameraStream_.request.MAIN_CAM;
  setupCameraStream_.request.start = 1;
  return setup_camera_stream_client.call(setupCameraStream_);
}

bool setup_camera_stream_fpv_cam()
{
  SetupCameraStream setupCameraStream_;
  setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
  setupCameraStream_.request.start = 1;
  return setup_camera_stream_client.call(setupCameraStream_);
}

bool camera_take_photo()
{
  /* Service to take photo with main camera */
  CameraAction take_photo_;
  take_photo_.request.camera_action = take_photo_.request.CAMERA_ACTION_TAKE_PICTURE;
  return take_photo_service.call(take_photo_);


}

bool set_local_position()
{
//Chiama il servizio nel quale la posizione attuale GPS del drone viene fissata come quella di home
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


//################ Callback #################

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{

  flight_status_data = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{

  
  display_mode_data =  msg->data;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

//Subscribers to RGB  image control acquired points referred to drone body frame 
void drone_RGB_control_point1_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point drone_RGB_control_point1;
    drone_RGB_control_point1 = *msg;
    control_RGB_point1_x = drone_RGB_control_point1.x;
    control_RGB_point1_y= drone_RGB_control_point1.y;
    flagDroneRGBControlPoint1 = true;
}

void drone_RGB_control_point2_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point drone_RGB_control_point2;
    drone_RGB_control_point2 = *msg;
    control_RGB_point2_x = drone_RGB_control_point2.x;
    control_RGB_point2_y= drone_RGB_control_point2.y;
    flagDroneRGBControlPoint2 = true;
}

void drone_thermo_control_point1_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point drone_thermo_control_point1;
    drone_thermo_control_point1 = *msg;
    control_thermo_point1_x = drone_thermo_control_point1.x;
    control_thermo_point1_y= drone_thermo_control_point1.y;
    flagDroneThermoControlPoint1 = true;     
}

void drone_thermo_control_point2_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point drone_thermo_control_point2;
    drone_thermo_control_point2 = *msg;
    control_thermo_point2_x = drone_thermo_control_point2.x;
    control_thermo_point2_y= drone_thermo_control_point2.y;
    flagDroneThermoControlPoint2 = true;
}

void drone_attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
 
  current_atti = msg->quaternion; //msg->quaternion;
  yaw_in_Rad = toEulerAngle(current_atti).z;
 
  /*
  FRAME AL DECOLLO:
  allineato con NED ma asse Y verso NORD e asse X verso EST
  --> YAW = 0 se drone allineato con asse X 
  ---> Yaw = pi/2 se drone allineato con Asse Y local, quindi col NORD.
  */

    //yaw_in_Rad = yaw_in_Rad - C_PI/2;
  //Transformation Yaw to reflect simulation ---> 
  //Trasformazione necessaria per avere yaw allineato con asse x drone e con valore 0.0 se drone allineato con frame HOME---> VEDI NOTE IPAD
  if(yaw_in_Rad <= +C_PI && yaw_in_Rad > -C_PI/2)
  {
      yaw_in_Rad_sim = -yaw_in_Rad + C_PI/2;
  }
  else
  {
      yaw_in_Rad_sim = -C_PI - yaw_in_Rad -C_PI/2;
  }
  
  flagDroneAttitude = true;
}

void drone_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  
  current_gps = *msg;
  // gps_latitude = current_gps.latitude;
  // gps_longitude = current_gps.longitude;

  //Condivido variabile
  flagDroneGPSpos = true;
}

void drone_local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
 
  current_local_pos = msg->point;
  // drone_local_pos_x = current_local_pos.x;
  // drone_local_pos_y = current_local_pos.y;
  
  flagDroneLocalpos = true;
}

void drone_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  current_local_velocity.vector = msg->vector;
  //std::cout<<"drone_velocity: "<<drone_velocity<<std::endl;
  
  // drone.drone_vel_x = drone->control_x_coo, drone->control_y_coo drone_velocity.vector.x;
  // drone.drone_vel_y =  drone_velocity.vector.y;
  // drone.drone_vel_z =  drone_velocity.vector.z;
  flagDroneLocalvel = true;
}

void drone_angular_velocity_callback(const  geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  current_angular_velocity.vector = msg ->vector;
  yaw_vel = current_angular_velocity.vector.z;
  flagDroneAngularVel = true;
}


void altitude_offset_callbak(const std_msgs::Float32::ConstPtr& msg)
{
  //Take information of the altitude respect the shape pf the panels in the image detetcted 
    std_msgs::Float32 offset_value;
    offset_value = *msg;
    offset = offset_value.data;
    flagAltitudeOffset = true;

}




void ultrasonic_callback(const std_msgs::Float32::ConstPtr& msg) //Vedere che messaggio è 
{
    std_msgs::Float32 ultrasonic_altitude;
     ultrasonic_altitude =  *msg;
    
     ultrasonic_altitude_value = ultrasonic_altitude.data;

     //ultrasonic_offset = reference_altitude_value - ultrasonic_altitude_value.data;
     flagUltrasonic = true;

}


//Callback to paramster optimization callabck from RGB detection algorithm

void parameter_opt_callback(const std_msgs::Bool::ConstPtr& msg)
{
    //Callback for parameters optimization callback
    //False if the Optimization is completed 
    //True if teh optimization is done 
    //Cosi perche ho utilizatto un flag gia presente nel codice 
    std_msgs::Bool param;
    param = *msg;
    if (param.data == false)
    {
      Optimization_completed = false;
    }
    else
    {
      Optimization_completed = true;
    }
    flagParametersOptimization = true;

}


void gimbalangle_callback( const geometry_msgs::Vector3Stamped::ConstPtr& angle)
{
  //OBTAIN gimbal angle 
  gimbalAngle = *angle;
 
  flagGimbalAngle = true;
  
}


void VoPosition_callback(const dji_osdk_ros::VOPosition::ConstPtr& vo_pos)
{
  //Obtain teh absolute altitude of the drone, not the offset 
  vo_position_msg = *vo_pos;
  altitude = -1*vo_position_msg.z-100;
  
 
  flagAltitude = true;
}


void wait_user_keyboard_input(const std_msgs::Bool::ConstPtr& msg)
{
    std_msgs::Bool key_input;
    key_input = *msg;

    key_for_control_is_pressed = key_input.data;
    flagKeyIsPressed = true;
   
}

/*############### Callback for the script which took photos ########### */
//Subscrfibe to missipn flag since also the script take_overlapped_photos use the calss Drone 
void inNavigation_flag_callback(const std_msgs::Bool::ConstPtr& msg)
{
    std_msgs::Bool inNavigation;
    inNavigation = *msg;

    Navigation_flag = inNavigation.data;
    
    flagInNavigation = true;
}


//Obtain array number from wait_user_keyboard_input.py
//The control of the drone is released only when the number of the array to be inspected is inserted 
void array_number_callback(const solar_project::KEY_PRESSED::ConstPtr& msg)
{
   solar_project::KEY_PRESSED key;
    key = *msg;

    array_number = key.data;
    
    flagArraynumber = true;
}


//Select user desired velocity callback 
void user_desired_velocity_callback(const std_msgs::Float32::ConstPtr& msg)
{
  std_msgs::Float32 vel;
  vel = *msg;

  desired_velocity = vel.data;
    
  flagUserDesiredVelocity = true;
}



//TEST ONLY 
void user_desired_velocity_y_callback(const std_msgs::Float32::ConstPtr& msg)
{
  std_msgs::Float32 vel;
  vel = *msg;

  desired_velocity_y = vel.data;
    
  flagUserDesiredVelocity_y = true;
}




void KF_init_flag_callback(const std_msgs::Bool::ConstPtr& msg)
{
  std_msgs::Bool KF_init;
    KF_init = *msg;

    KF_init_flag = KF_init.data;
    
    flagKFINIT= true;
}




// ########################################à

//###############à Publish Command Position ########

void publish_cmd_angle_gimbal( float gimbal_pitch_des)
{
  dji_osdk_ros::Gimbal cmd;
  
  cmd.mode = 1; //Yaw riferito alla prua del drone 
  cmd.roll = 0.0; // gimbal_pitch_des;
  cmd.pitch = gimbal_pitch_des;  //gimbal_pitch_des;
  //cmd.yaw = 10; 
  cmd.ts = 1;
  GimbalAngle_cmd.publish(cmd);
}


void publish_cmd_position( sensor_msgs::Joy& cmd)
{
  ctrlPosYawPub.publish(cmd);
}





void publish_estimated_control_points()
{
  geometry_msgs::Point point1;
  point1.x = P1_B[0];
  point1.y = P1_B[1];
  
  geometry_msgs::Point point2;
  point2.x = P2_B[0];
  point2.y = P2_B[1];

  pub_P1_estimated.publish(point1);
  pub_P2_estimated.publish(point2);

}

void publish_new_current_local_position(bool end_panel_reached)
{
  geometry_msgs::Point point;
  point.x = new_current_local_pos.x;
  point.y = new_current_local_pos.y;

  if (end_panel_reached == true)
  {
    point.x = 1000;
    point.y = 1000;
  }

  new_local_position.publish(point);
}



void publish_cmd_velocity( sensor_msgs::Joy& cmd)
{
   ctrlVelYawPub.publish(cmd);
}


void publish_control_D_point_to_GAZEBO(bool image_heigth_flag, float offset)
{
  //PUBLISH CONTROL D POINT FROM EVALUATE CONTROL TO GAZEBO ARDRONE ---> EXPRESSED IN BF
  geometry_msgs::Point point_D;
  //Normalize Vector 
  if (control_x_coo == 0 || control_y_coo == 0)
  {
     point_D.x = 0.0;
     point_D.y = 0.0;
     point_D.z = 0.0;
  }
  else
  {
    float i = sqrt(pow(control_x_coo,2) + pow(control_y_coo,2));
   
    point_D.x = 0.5*(control_x_coo/i);
    point_D.y = 0.5*(control_y_coo/i);
    if (image_heigth_flag == true)
    {
      point_D.z = offset; //only when controlled by image 
    }
    else
    {
      point_D.z = 0.0;
    }
    
  }
  
  

  // Rotate points To gazebo using yaw_in_Rad
  //point_D.y = sin(yaw_in_Rad)*point_D.x + cos(yaw_in_Rad)*point_D.y;
  GAZEBO_Point_D_in_BF.publish(point_D);



}


//Publish flag to RGB detection when user takles control
void publish_obtain_control_flag(bool obtain_control_result)
{
   std_msgs::Bool flag;
  
  if (obtain_control_result == true)
  {
    flag.data = obtain_control_result;
  }
  else
  {
    flag.data = obtain_control_result;
  }

  user_joy_control_obtained.publish(flag);
}


//Publish flag relative to Optimization start 
void publish_OPT_START(std_msgs::Bool OPT_start)
{
  start_OPT_task.publish(OPT_start);
}


//Publish KF_init flag 
void publish_KF_init(std_msgs::Bool KF_Init)
{
  //serve per terminare la l'inizializazzione della maschera per l'0altitudeone 
 KF_init.publish(KF_Init);

}


//Publish flag inNavigation to script which take mission photo 
void publish_flag_inNavigation(std_msgs::Bool inNavigation_flag)
{
   inNav.publish(inNavigation_flag);

}


//Publish a flag when the done is under user control
void publish_drone_under_user_control(std_msgs::Bool user_control)
{
   under_user_control.publish(user_control);
}


//#########################À SERVICE FUNCTIONS #############################
void rotate_drone_position_respect_new_origin_frame(geometry_msgs::Point new_start_position)
{
 /*
    new_start_position --> Nuovo Frame locale 
    current_local_pos --> Posizione calcolata rispetto frame HOME
    NB: IL FRAME LOCALE SI SUPPONE ALLINEATO CON IL FRAME HOME 
 */ 
   new_current_local_pos.x = current_local_pos.x - new_start_position.x;
   new_current_local_pos.y = current_local_pos.y - new_start_position.y;
   new_current_local_pos.z = current_local_pos.z;
}

void rotate_target_position_from_new_local_to_HOME_local(float X_Target_new, float Y_Target_new)
{
  //Rtiotyate TARGET to LOCAL HOME 
  x_target_HOME = X_Target_new + current_local_pos.x;
  y_target_HOME = Y_Target_new + current_local_pos.y;

}

void Rotation_BF_to_local_GF_des_pos(geometry_msgs::Point drone_local_position,  float P1_x_obs_body, float P1_y_obs_body, float yaw_in_rad_rel)
{
  //La matrice R è considerata come trasposta 
 
  check_x_local = P1_x_obs_body * cos(yaw_in_rad_rel) + P1_y_obs_body * sin(yaw_in_rad_rel) +drone_local_position.x;
  check_y_local = -1*P1_x_obs_body * sin(yaw_in_rad_rel) + P1_y_obs_body * cos(yaw_in_rad_rel) + drone_local_position.y;
}

void Rotation_BF_to_local_GF_des_pos_original(geometry_msgs::Point drone_local_position,  float P1_x_obs_body, float P1_y_obs_body, float yaw_in_rad_rel)
{
  check_x_local = P1_x_obs_body * cos(yaw_in_rad_rel) - P1_y_obs_body * sin(yaw_in_rad_rel) + drone_local_position.x;
  check_y_local = P1_x_obs_body * sin(yaw_in_rad_rel) + P1_y_obs_body * cos(yaw_in_rad_rel) + drone_local_position.y;
}

void Rotation_local_GF_to_BF(geometry_msgs::Point drone_local_position, float P1_x_local, float P1_y_local, float yaw_in_rad_rel)
{

  check_x_b = P1_x_local*cos(yaw_in_rad_rel) + P1_y_local * sin(yaw_in_rad_rel) -cos(yaw_in_rad_rel)* drone_local_position.x - sin(yaw_in_rad_rel)*drone_local_position.y; //Checkpoint coordinate in drone body frame
  check_y_b = -P1_x_local*sin(yaw_in_rad_rel) + P1_y_local * cos(yaw_in_rad_rel) +sin(yaw_in_rad_rel)* drone_local_position.x - cos(yaw_in_rad_rel)*drone_local_position.y;
}


void acceleration_GF()
{
  float dt = 0.05;
  x_ddot_GF = (current_local_velocity.vector.x - old_x_dot_GF)/dt;
  y_ddot_GF =  (current_local_velocity.vector.y - old_y_dot_GF)/dt;
  z_ddot_GF =  (current_local_velocity.vector.z - old_z_dot_GF)/dt;
  old_x_dot_GF = current_local_velocity.vector.x;
  old_y_dot_GF = current_local_velocity.vector.y;
  old_z_dot_GF =  current_local_velocity.vector.z;

  cout << " X ddot: " << x_ddot_GF << " Y ddot: " << y_ddot_GF << " Z ddot: "  << z_ddot_GF;

}


void saturation(float x_vel, float y_vel, float sat_x_vel, float sat_y_vel)
{
  /*
  Saturate Linear Velocities only when the drone is in Viusual or GPS Navigation 
  and it Follow The Estimated Line 
  */

     if (x_vel > sat_x_vel)
    {
       x_vel_cmd = sat_x_vel;
    }
    else if (x_vel < -sat_x_vel)
    {
       x_vel_cmd= -sat_x_vel;
    }
    
    if (y_vel > sat_y_vel)
    {
        y_vel_cmd = sat_y_vel; 
    }
    else if (y_vel < -sat_y_vel)
    {
        y_vel_cmd = -sat_y_vel;
    }
    
}


void saturation_offset(float x_off, float y_off, float sat_x_off, float sat_y_off)
{

     if (x_off > sat_x_off)
    {
       x_pos_cmd = sat_x_off;
    }
    else if (x_off < -sat_x_off)
    {
       x_pos_cmd= -sat_x_off;
    }
    
    if (y_off > sat_y_off)
    {
        y_pos_cmd = sat_y_off; 
    }
    else if (y_off < -sat_y_off)
    {
        y_vel_cmd = -sat_y_off;
    }
    

}

};


#endif
