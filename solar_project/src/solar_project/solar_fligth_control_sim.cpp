#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "gazebo_msgs/SetModelState.h"
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
//#include "image_converter.h"


#include "/home/lucamora/catkin_ws/src/solar_panel_project/include/solar_panel_project/pid.h"
#include "/home/lucamora/catkin_ws/src/solar_panel_project/include/solar_panel_project/waypoints.h"
#include "/home/lucamora/catkin_ws/src/solar_panel_project/include/solar_panel_project/Panel.h"
#include "/home/lucamora/catkin_ws/src/solar_panel_project/include/solar_panel_project/KF.h"

using namespace std;
#define N 2 

// Solar Panel Position and Size
unsigned int sleep(unsigned int seconds);


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

//IN GF 
    float GPS_P1_x = 0.0;
    float GPS_P1_y = 0.0;
    float GPS_P2_x = 0.0;
    float GPS_P2_y = 0.0;
//IN BF
    float GPS_P1_x_BF = 0.0;
    float GPS_P1_y_BF = 0.0;
    float GPS_P2_x_BF = 0.0;
    float GPS_P2_y_BF = 0.0;


    //Vector to initialize the Kalamn filter state with the equation of the line passing through the GPS
    //Waypoints 
    vector<float> GPS_P1_waypoint; //Start P1 Waypoint 
    vector<float> GPS_P2_waypoint; //End P2 waypoint
} waypoints;




struct Drone
{
    //Struct that link drone variables store in class drone outside of main 
   //Drone Position, Orientation variables 
  float drone_x, drone_y, drone_z;
  double drone_Yaw, drone_roll;
  float drone_ang_vel_z;
  float drone_lin_vel_x, drone_lin_vel_y, drone_lin_vel_z;

  geometry_msgs::Twist drone_vel_msg;

 
  float x_vel, y_vel, z_vel, x_angular_vel, y_angular_vel, z_angular_vel; //Desired Velocites
  float x_des, y_des, z_des, z_des_navigation; //Desired positions
  float yaw_des;
  float yaw_err;

  
  float drone_x_b = 0.0;
  float drone_y_b = 0.0;

  float offset = 0.0;
  //Drove Control points obtained from camera expressed in drone body frame
  float control_Thermo_point1_x, control_Thermo_point1_y;
  float control_Thermo_point2_x, control_Thermo_point2_y;
  float control_RGB_point1_x, control_RGB_point1_y;
  float control_RGB_point2_x, control_RGB_point2_y;
 
   float RGB_control_obs_P1_x_world = 0.0;
   float RGB_control_obs_P1_y_world = 0.0;
   float RGB_control_obs_P2_x_world = 0.0;
   float RGB_control_obs_P2_y_world = 0.0;

  float thermo_control_obs_P1_x_world = 0.0;
  float thermo_control_obs_P1_y_world = 0.0;
  float thermo_control_obs_P2_x_world = 0.0;
  float thermo_control_obs_P2_y_world = 0.0 ;

  //OBS THERMO LINE parameters in BF 
  float THERMO_M_BF = 0.0;
  float THERMO_C_BF = 0.0;

  //OBS GPS LINE parameters in BF 
  float RGB_M_BF = 0.0;
  float RGB_C_BF = 0.0;

  //Control point D coordinates from GPS 
  float control_x_coo = 0.0;
  float control_y_coo = 0.0;

  float x_target = 0.0;
  float y_target = 0.0;
  
  float last_position_x = 0.0;
  float last_position_y = 0.0;
  float last_position_z = 0.0;

  float MATRICE_local_position_x = 0.0;
  float MATRICE_local_position_y = 0.0;
  float MATRICE_local_position_z = 0.0;
 //KF counter 
  int thermo_detection_count = 0;
  int RGB_detection_count = 0;

  Eigen::Vector2f xh_; //KF estimated state
  Eigen::Vector2f xh_b; //KF estimated state in body frame
  Eigen::Vector2f obs; //Obtain from class KF observation vector
  Eigen::Vector2f yh_; //Obtain estimated Observation from EKF
  Eigen::Vector2f P1_B;
  Eigen::Vector2f P2_B;
  Eigen::Vector2f obs_thermo_GF;
  Eigen::Vector2f obs_RGB_GF;

  int image_control_count = 0;
  int RGB_image_control_count = 0;
  int THERMO_image_control_count = 0;

  bool flagDroneOdom = false;
  bool flagDroneImu = false;
  bool flagDroneFix_vel = false;
  bool flagDroneThermoControlPoint1 = false;
  bool flagDroneThermoControlPoint2 = false;
  bool flagDroneRGBControlPoint1 = false;
  bool flagDroneRGBControlPoint2 = false;
  bool flagDroneMatricePointD = false;
  bool flagMatriceLocalPos = false;
  bool flagAltitudeOffset = false;

} drone;


struct Mission
{
  int state = 0;
  //Target : GPS o P ogni inizio pannello
  float x_target_P1 = 0.0;
  float y_target_P1 = 0.0; 
  float x_target_P2 = 0.0;
  float y_target_P2 = 0.0; 

  //Target GPS in BF
  float x_target_P1_BF = 0.0;
  float y_target_P1_BF = 0.0;
  float x_target_P2_BF = 0.0;
  float y_target_P2_BF = 0.0;
  
  float line_setpoint_distance = 0.0;
  vector<float> P1_target;
  vector<float> P2_target;
  //Eigen::Vector2f Target;

  //Target Vision Point in BF from MAtrice
  geometry_msgs::Point target_from_MATRICE;
    
  float x_target = 0.0;
  float y_target = 0.0;
  int target_point = 0.0;
  float cartesian_distance_err = 0.0;
  
  int count = 0;
  int navigation_iteration_count = 0;
  
  bool GAZEBO_LINK = false;
  bool Optimization_completed = false;
  bool flagParametersOptimization = false;
  bool start_optimization_task = false;
  bool flag_end_point = false;
  bool flag_create_setpoint = false;
  bool panel_array_initialization = false;
  bool KF_Initialization = false;
  bool Optimization_enabled = false;

} mission;

//Initialize Kalman FIlter  
KF Kalman_Filter = KF();



nav_msgs::Odometry drone_odom;
sensor_msgs::Imu drone_imu;
geometry_msgs::Vector3Stamped drone_fix_vel;
geometry_msgs::Point drone_Thermo_control_point1;
geometry_msgs::Point drone_Thermo_control_point2;
geometry_msgs::Point drone_Thermo_control_point11;
geometry_msgs::Point drone_Thermo_control_point22;
geometry_msgs::Point drone_RGB_control_point1;
geometry_msgs::Point drone_RGB_control_point2;
std_msgs::Float32 offset;
std_msgs::Empty myMsg;
std_msgs::Bool param;


float check_x_b = 0.0;
float check_y_b = 0.0;
float check_x_w = 0.0;
float check_y_w = 0.0;

bool from_image = false;
bool from_image_RGB = false;
bool from_image_Thermo = false;

bool coming_back_to_GPS_LINE = false;

void drone_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    drone_odom = *msg;
    drone.drone_x = drone_odom.pose.pose.position.x;
    drone.drone_y = drone_odom.pose.pose.position.y;
    drone.drone_z = drone_odom.pose.pose.position.z;
   

    // quaternion to RPY conversion
    tf::Quaternion q(drone_odom.pose.pose.orientation.x, drone_odom.pose.pose.orientation.y,
        drone_odom.pose.pose.orientation.z, drone_odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    double drone_roll, drone_pitch, drone_yaw;
    m.getRPY(drone_roll, drone_pitch, drone_yaw);
    drone.drone_roll = drone_roll;
    drone.drone_Yaw = drone_yaw;
    // angular position
    drone.flagDroneOdom = true;
}

void drone_Imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    drone_imu = *msg;
    drone.drone_ang_vel_z = drone_imu.angular_velocity.z;
    drone.flagDroneImu = true;
}

void drone_fix_Vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    drone_fix_vel = *msg;
    drone.drone_lin_vel_x = drone_fix_vel.vector.x;
    drone.drone_lin_vel_y = drone_fix_vel.vector.y;
    drone.drone_lin_vel_z = drone_fix_vel.vector.z;
    drone.flagDroneFix_vel = true;
}




// OBTAIN CONTROL POINTS FROM THERMAL AND RGB IMAGES //
//#############  Desired Control POint 1 from Thermal images  ---> Usati per ottenere la retta esportata nel body vrame dall'image frame 
void drone_Thermo_control_point1_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    drone_Thermo_control_point1 = *msg;
   
    drone.control_Thermo_point1_x = drone_Thermo_control_point1.x;
    drone.control_Thermo_point1_y= drone_Thermo_control_point1.y;

    drone.thermo_detection_count = 0;
    drone.flagDroneThermoControlPoint1 = true;
     
}

//Desired COntrol Point 2 from thermal Images 
void drone_Thermo_control_point2_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    drone_Thermo_control_point2 = *msg;
    drone.control_Thermo_point2_x = drone_Thermo_control_point2.x;
    drone.control_Thermo_point2_y= drone_Thermo_control_point2.y;
    drone.flagDroneThermoControlPoint2 = true;
     
}



// Desired Control POint 1 from RGB images ---> Usati per ottenere la retta esportata nel body vrame dall'image frame 
void drone_RGB_control_point1_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    drone_RGB_control_point1 = *msg;
    drone.control_RGB_point1_x = drone_RGB_control_point1.x;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    drone.control_RGB_point1_y= drone_RGB_control_point1.y;

    drone.RGB_detection_count = 0;
    drone.flagDroneRGBControlPoint1 = true;
     
}



//Desired COntrol Point 2 from thermal Images 
void drone_RGB_control_point2_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    drone_RGB_control_point2 = *msg;
    drone.control_RGB_point2_x = drone_RGB_control_point2.x;
    drone.control_RGB_point2_y= drone_RGB_control_point2.y;
    drone.RGB_detection_count = 0;
    drone.flagDroneRGBControlPoint2 = true;
     
}

void altitude_offset_callbak(const std_msgs::Float32::ConstPtr& msg)
{
    offset = *msg;
    drone.offset = offset.data;
    drone.flagAltitudeOffset = true;

}

void parameter_opt_callback(const std_msgs::Bool::ConstPtr& msg)
{
    //Callback for parameters optimization callback
    //False if the Optimization is completed 
    //True if teh optimization is done 
    //Cosi perche ho utilizatto un flag gia presente nel codice 
    param = *msg;
    if (param.data == false)
    {
        mission.Optimization_completed = true;
    }
    else
    {
        mission.Optimization_completed = false;
    }
    mission.flagParametersOptimization = true;

}


void Matrice_point_D_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point take_MATRICE_points;
    take_MATRICE_points = *msg;
    mission.target_from_MATRICE.x = take_MATRICE_points.x;
    mission.target_from_MATRICE.y = take_MATRICE_points.y;

    drone.flagDroneMatricePointD = true;
}

void DJI_local_pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
   
    drone.MATRICE_local_position_x = msg->point.x;;
    drone.MATRICE_local_position_y = msg->point.y;
    drone.MATRICE_local_position_z = msg->point.z;
    drone.flagMatriceLocalPos = true;
}


/// ROTATION AND TRASFORMATION FUNCTION
void Rotation_GF_to_BF_des_pos(float x_pos, float y_pos, float alfa)
{
     check_x_b = x_pos*cos(alfa) + y_pos * sin(alfa) -cos(alfa)*drone.drone_x - sin(alfa)*drone.drone_y; //Checkpoint coordinate in drone body frame
     check_y_b = -x_pos*sin(alfa) + y_pos * cos(alfa) +sin(alfa)*drone.drone_x - cos(alfa)*drone.drone_y;
}


void Rotation_GF_to_BF_drone_pos(float alfa)
{
     drone.drone_x_b = drone.drone_x*cos(alfa) + drone.drone_y * sin(alfa) - drone.drone_x*cos(alfa) - drone.drone_y * sin(alfa); //Checkpoint coordinate in drone body frame
     drone.drone_y_b = -drone.drone_x*sin(alfa) + drone.drone_y * cos(alfa) +sin(alfa)*drone.drone_x - cos(alfa)*drone.drone_y;
}

void Rotation_BF_to_GF_des_pos(float x_pos, float y_pos, float alfa)
{
    check_x_w = x_pos * cos(alfa) - y_pos * sin(alfa) + drone.drone_x;
    check_y_w = x_pos * sin(alfa) + y_pos * cos(alfa) + drone.drone_y;
}

void saturation(float vel_x, float vel_y, float vel_z, float vel_yaw)
{
     if (vel_x > 1)
    {
        drone.drone_vel_msg.linear.x = 1;
    }
    else if (vel_x < -1)
    {
       drone.drone_vel_msg.linear.x = -1;
    }
    
    if (vel_y > 1)
    {
        drone.drone_vel_msg.linear.y = 1; 
    }
    else if (vel_y < -1)
    {
        drone.drone_vel_msg.linear.y = -1;
    }
    if (vel_z > 1)
    {
       drone.drone_vel_msg.linear.z = 1; 
    }
    else if (vel_z < -1)
    {
       drone.drone_vel_msg.linear.z = -1;
    }

    if (vel_yaw > 0.4)
    {
       drone.drone_vel_msg.angular.z = 0.4; 
    }
    else if (vel_yaw < -0.4)
    {
        drone.drone_vel_msg.angular.z = -0.4;
    }
}



void GPS_background_line()
{
   float a = 0.0; //equivale a m coefficente angolare
    float c = 0.0;
    float b = -1;
    
    waypoints.GPS_P1_x = waypoints.waypoints_x_coo_world_frame[mission.count - 1];
    waypoints.GPS_P1_y = waypoints.waypoints_y_coo_world_frame[mission.count - 1];
    waypoints.GPS_P2_x = waypoints.waypoints_x_coo_world_frame[mission.count ]; 
    waypoints.GPS_P2_y = waypoints.waypoints_y_coo_world_frame[mission.count ];

    float x_target_P1_body = 0.0;
    float y_target_P1_body = 0.0;
    float x_target_P2_body = 0.0;
    float y_target_P2_body = 0.0;

    //Rotate Point P1 in drone body frame 
    Rotation_GF_to_BF_des_pos( waypoints.GPS_P1_x , waypoints.GPS_P1_y ,  drone.drone_Yaw);
    x_target_P1_body =  check_x_b;
    y_target_P1_body = check_y_b;

    //Rotate Point P2 in drone body frame 
    Rotation_GF_to_BF_des_pos( waypoints.GPS_P2_x,waypoints.GPS_P2_y,  drone.drone_Yaw); // mission.y_target_P2
    x_target_P2_body =  check_x_b;
    y_target_P2_body = check_y_b; 

    //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0
     a =  ((y_target_P2_body  -  y_target_P1_body)/(x_target_P2_body - x_target_P1_body));
     c = ((-(a) * x_target_P1_body) + y_target_P1_body);

    //Find distance point line: point is the body Origin and line is the line r
     double e = (abs(c)/(sqrt(pow(a,2) + pow(b,2))));
     waypoints.error_from_GPS_line = e;


}



void  evaluate_control_point(bool from_image)
{
    //Equation line parameters Passing trhiug P1 and P2
    float a = 0.0; //equivale a m coefficente angolare
    float c = 0.0;
    float b = -1;

    float x_target_P1_body = 0.0;
    float y_target_P1_body = 0.0;
    float x_target_P2_body = 0.0;
    float y_target_P2_body = 0.0;

    waypoints.GPS_P1_x = waypoints.waypoints_x_coo_world_frame[mission.count - 1];
    waypoints.GPS_P1_y = waypoints.waypoints_y_coo_world_frame[mission.count - 1];
    waypoints.GPS_P2_x = waypoints.waypoints_x_coo_world_frame[mission.count ]; 
    waypoints.GPS_P2_y = waypoints.waypoints_y_coo_world_frame[mission.count ];

    //Rotate Point P1 in drone body frame 
    Rotation_GF_to_BF_des_pos( waypoints.GPS_P1_x , waypoints.GPS_P1_y ,  drone.drone_Yaw);
    x_target_P1_body =  check_x_b;
    y_target_P1_body = check_y_b;

    //Rotate Point P2 in drone body frame 
    Rotation_GF_to_BF_des_pos( waypoints.GPS_P2_x, waypoints.GPS_P2_y,  drone.drone_Yaw); // mission.y_target_P2
    x_target_P2_body =  check_x_b;
    y_target_P2_body = check_y_b; 

    
    if (from_image == false)
    {
      
    cout<< "FOllowing GPS line!" << endl;

    //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0
     a =  ((y_target_P2_body  -  y_target_P1_body)/(x_target_P2_body - x_target_P1_body));
     c = ((-(a) * x_target_P1_body) + y_target_P1_body);
    
    }
    else
    {
     cout<< "[IMAGE NAVIGATION] FOllowing Estimated line!" << endl;
     x_target_P1_body = drone.control_Thermo_point1_x;
     y_target_P1_body = drone.control_Thermo_point1_y;
     x_target_P2_body = drone.control_Thermo_point2_x;
     y_target_P2_body = drone.control_Thermo_point2_y;
     
     a = drone.xh_b[0];
     c = drone.xh_b[1];
    //  a = drone.xh_[0];
    //  c = drone.xh_[1];
    
    
    }

    
    

     //Find distance point line: point is the body Origin and line is the line r
     double e = (abs(c)/(sqrt(pow(a,2) + pow(b,2))));
     cout << "[EVALUATE CONTROL] ERROR FROM LINE: " << e << endl;
     //waypoints.error_from_GPS_line = e;

     //Define vector Vx starting from body frame origin and parallel to the line r
     double Vx[2] = {1/a, -1/b};
     
     double Kx = 0.7; //Coefficiente moltiplicativo del vettore parallelo alla retta r
     double Vx_norm[2] = {Kx * ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[0]), Kx* ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[1])};

     //cout<<"target_point: "<< target_point<<endl;
     float rot_Vx = 0.0;
     float rot_Vy = 0.0;

     if (mission.target_point == 1)
     {
         //punto di target è P1:
         if (x_target_P1_body >= 0 &&  Vx_norm[0] < 0)
         {
     
            rot_Vx =  Vx_norm[0] * cos(M_PI) - Vx_norm[1] * sin(M_PI);
            rot_Vy = Vx_norm[0] * sin(M_PI) + Vx_norm[1] * cos(M_PI);
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
            rot_Vx =  Vx_norm[0] * cos(M_PI) - Vx_norm[1] * sin(M_PI);
            rot_Vy = Vx_norm[0] * sin(M_PI) + Vx_norm[1] * cos(M_PI);
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
    Vy_norm[0] = Ky * e *((1/(sqrt(pow(Vy[0],2) + pow(Vy[1],2)))) * Vy[0]); 
    Vy_norm[1] = Ky * e *((1/(sqrt(pow(Vy[0],2) + pow(Vy[1],2)))) * Vy[1]); 
     if (c > 0 && Vy_norm[1] < 0)
     {
         Vy_norm[1] = -1 * Vy_norm[1];
     }
     else if (c < 0 && Vy_norm[1] > 0)
     {
         Vy_norm[1] = -1 * Vy_norm[1];
     }
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

     drone.control_x_coo = abs(D[0]);
     drone.control_y_coo = D[1]; 
     
     cout << "D[0]: " << abs(D[0]) << endl;
     cout << "D[1]: " << D[1] << endl;

     //Se le coordinate sono NAN alla prima iterazione le metto 0.0;
     if (drone.control_x_coo != drone.control_x_coo and drone.control_y_coo != drone.control_y_coo)
     {
         drone.control_x_coo = 0.0;
         drone.control_y_coo = 0.0;
     }
    

    //cout << "[EVALUATE CONTROL LINE] DISTANCE TO LOCAL SETPOINT: " << sqrt(pow( drone.x_target - drone.drone_x,2) + pow( drone.y_target - drone.drone_y ,2))  << endl;
    if (from_image == true && mission.navigation_iteration_count > 80000)
    {
      /*
      EValuate a setpoints placed on the line every tot meters 
      The setpoint is evaluated in BF and then rotated directly to HOME frame for control (NOT EXPRESSED repect frame placed in P1 bu with respcet the TAKE OFF frame)
      */
       
        if (sqrt(pow( drone.x_target - drone.drone_x,2) + pow( drone.y_target - drone.drone_y ,2)) < 0.8 )
        {
       // alfa = target_coo_given_distance_setpoint(mission.line_setpoint_distance,  drone->control_x_coo, drone->control_y_coo);
        mission.x_target = drone.control_x_coo + mission.line_setpoint_distance;
        mission.y_target = drone.control_y_coo;

        cout<<"[EVALUATE CONTROL LINE] -----> BODY FRAME x_target: " <<  mission.x_target << endl;
        cout<<"[EVALUATE CONTROL LINE] -----> BODY FRAME y_target: " <<  mission.y_target << endl; 
        
    
        drone.x_target = mission.x_target;
        drone.y_target = mission.y_target;
        }
      
    } 
    else
    {
       //Permits to follow the line without teh use of setpoints
       //cout << "[EVALUATE CONTROL LINE] Setpoint Disabled " << endl;
       drone.x_target = drone.control_x_coo;
       drone.y_target = drone.control_y_coo;
    }
  
    
   // Velocity control in simulation is applied to Drone Body frame.
}





void KF_estimation_exportation_in_BF_for_control_point_generation()
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

     //Use the RGB KF estimation
      y1_w =  drone.xh_[0]*x1_w + drone.xh_[1];
      y2_w =  drone.xh_[0]*x2_w + drone.xh_[1];
      //cout<<"[KF EXP FUNC] KF RGB estimation exported for control Point Evaluation" << endl;
      //cout<<"[KF EXP FUNC] y1_w: " <<y1_w << ", "<< "y2_w: " << y2_w << endl;
      

      //ROtate both points in body frame 
      Rotation_GF_to_BF_des_pos(x1_w, y1_w, drone.drone_Yaw);
      x1_b = check_x_b;
      y1_b = check_y_b;
      
      Rotation_GF_to_BF_des_pos(x2_w, y2_w,drone.drone_Yaw);
      x2_b = check_x_b;
      y2_b = check_y_b;

      //Evaluate a,c parameters of the line in body frame
       a_b =  ((y2_b  - y1_b)/(x2_b - x1_b));
       c_b =  ((-(a_b) * x1_b) + y1_b);
      
      drone.xh_b << a_b, c_b;
      cout << "[KF EXP FUNC] ------------------____> drone.xh_b: " << drone.xh_b << endl; 
       
}



void start(Panel *panel, PID * pid_x, PID * pid_y, PID * pid_z, PID *pid_yaw)
{
    from_image_RGB = false;
    from_image_Thermo = false;
    from_image = false;
   //In start mission.count = 0
//Le coordinate di target sono le coordinate P1 del nuovo pannello, conosciute alla prima iterzione
   mission.x_target_P1 =  panel -> obtain_xcoo_panel_world_frame()[mission.count];
   mission.y_target_P1 =  panel -> obtain_ycoo_panel_world_frame()[mission.count];        
   mission.x_target_P2 =  panel -> obtain_xcoo_panel_world_frame()[mission.count + 1];
   mission.y_target_P2 =  panel -> obtain_ycoo_panel_world_frame()[mission.count + 1];
   
  
   mission.P1_target.push_back(mission.x_target_P1);
   mission.P1_target.push_back(mission.y_target_P1);
   mission.P2_target.push_back(mission.x_target_P2);
   mission.P2_target.push_back(mission.y_target_P2);
  
   //Point Relative to GPS waypoints 
    waypoints.GPS_P1_waypoint.push_back(waypoints.waypoints_x_coo_world_frame[mission.count]);
    waypoints.GPS_P1_waypoint.push_back(waypoints.waypoints_y_coo_world_frame[mission.count]);
    waypoints.GPS_P2_waypoint.push_back(waypoints.waypoints_x_coo_world_frame[mission.count + 1]); 
    waypoints.GPS_P2_waypoint.push_back(waypoints.waypoints_y_coo_world_frame[mission.count + 1]);
   
   // evaluate_control_point(from_image);

   mission.x_target = panel -> obtain_xcoo_panel_world_frame()[mission.count];
   mission.y_target = panel -> obtain_ycoo_panel_world_frame()[mission.count];
   
   mission.x_target =  mission.x_target + 2;
  //cout << "[START MISSION]--> Reaching P1 panel "<< mission.count +1 << " --> distance --> " <<  mission.cartesian_distance_err<< endl;

   mission.cartesian_distance_err = sqrt(pow(mission.x_target- drone.drone_x,2) + pow(mission.y_target- drone.drone_y,2));
   drone.drone_vel_msg.angular.z  = pid_yaw -> position_control_knowing_velocity(drone.yaw_des , abs(drone.drone_Yaw), 0, drone.drone_ang_vel_z);
   
   if (drone.drone_Yaw < 0)
   {
      drone.drone_vel_msg.angular.z = -1* drone.drone_vel_msg.angular.z;
   }

   drone.yaw_err = drone.yaw_des - abs(drone.drone_Yaw);
   if (abs(drone.yaw_err) < 0.5 )
    {
        Rotation_GF_to_BF_des_pos(mission.x_target , mission.y_target,  drone.drone_Yaw);
       Rotation_GF_to_BF_drone_pos( drone.drone_Yaw);

       drone.drone_vel_msg.linear.x = pid_x->calculate(check_x_b, drone.drone_x_b);
       drone.drone_vel_msg.linear.y = pid_y->calculate(check_y_b, drone.drone_y_b);
       
    }

    //Raggiunto Point P1 mi dirigo verso point P2
    if (mission.cartesian_distance_err < 0.2)
    {
        mission.flag_end_point = true; //Mi dirigo lungo il pannello verso punto P2 di end --> passo all'else sottostante 
        mission.flag_create_setpoint = true;
        if (mission.count == 0)
        {
            drone.yaw_des = panel->obtain_theta();
        }
        drone.last_position_x = drone.drone_x;
        drone.last_position_y = drone.drone_y;
        drone.last_position_z = drone.drone_z;
        drone.yaw_des = drone.drone_Yaw;
        drone.image_control_count = 100; // inizializazzione valore grosso. --> Serve per aggiornare il filtro per un certo numero di iterazioni anche se nin sono state ricevute nuove osservazioni-- > utilizzal'ultima osservazione ricevuta 
        drone.THERMO_image_control_count = 200;
        drone.RGB_image_control_count = 200;
        mission.count = mission.count + 1; //--> Raggiungo punto P2 stesso pannello
        mission.target_point = 2;
        mission.KF_Initialization = true;
        mission.panel_array_initialization = true; //necessaria per la corretta inizializazzione Kalman Filter
        mission.state = 1; //Quando 1 mi muovo lungo il pannello tra punti 1 e 2
        if (mission.Optimization_enabled == true)
        {
            mission.state = 3; //Go to wait for parameters Optimization case
        }
                    

    }
}



int stop_navigation_counter = 0;
float x = 0.0;
float y = 0.0;
int counter_stop_matrice = 0;
void navigation(Panel *panel, PID * pid_x, PID * pid_y, PID * pid_z, PID *pid_yaw)
{

    //Default 
   bool KF_init = false;

    //Reaching point P2 from point P1. P1 ---> P2
    mission.x_target_P1 =  panel->obtain_xcoo_panel_world_frame()[mission.count- 1];
    mission.y_target_P1 =  panel->obtain_ycoo_panel_world_frame()[mission.count - 1];
                
    mission.x_target_P2 =  panel->obtain_xcoo_panel_world_frame()[mission.count];
    mission.y_target_P2 =  panel->obtain_ycoo_panel_world_frame()[mission.count];


    //Point to pass to Kalman FIlter class function
    
    mission.P1_target.push_back(mission.x_target_P1);
   mission.P1_target.push_back(mission.y_target_P1);
   mission.P2_target.push_back(mission.x_target_P2);
   mission.P2_target.push_back(mission.y_target_P2);
  
   //Point Relative to GPS waypoints 
    waypoints.GPS_P1_x = waypoints.waypoints_x_coo_world_frame[mission.count - 1];
    waypoints.GPS_P1_y = waypoints.waypoints_y_coo_world_frame[mission.count - 1];
    waypoints.GPS_P2_x = waypoints.waypoints_x_coo_world_frame[mission.count ]; 
    waypoints.GPS_P2_y = waypoints.waypoints_y_coo_world_frame[mission.count ];

   
/*
   cout << " drone.drone_Yaw: " <<  drone.drone_Yaw << endl;
    //Point relative to GPS waypoint in BF
    Rotation_GF_to_BF_des_pos(waypoints.GPS_P1_x , waypoints.GPS_P1_y ,  drone.drone_Yaw);
    waypoints.GPS_P1_x_BF = check_x_b;
    waypoints.GPS_P1_y_BF = -1*check_y_b;
    
    Rotation_GF_to_BF_des_pos(waypoints.GPS_P2_x , waypoints.GPS_P2_y ,  drone.drone_Yaw);
    waypoints.GPS_P2_x_BF = check_x_b;
    waypoints.GPS_P2_y_BF = -1*check_y_b;


   
   //ROtate START and END points of panels  to BF 
   Rotation_GF_to_BF_des_pos(mission.x_target_P1, mission.y_target_P1,  drone.drone_Yaw);
   mission.x_target_P1_BF = check_x_b;
   mission.y_target_P1_BF = -1*check_y_b;
  
   Rotation_GF_to_BF_des_pos(mission.x_target_P2, mission.y_target_P2,  drone.drone_Yaw);
   mission.x_target_P2_BF = check_x_b;
   mission.y_target_P2_BF = -1*check_y_b;

   cout<< "GPS P1 X BF:  "<< waypoints.GPS_P1_x_BF << " GPS P1 Y BF:  " << waypoints.GPS_P1_y_BF << endl;
   cout<< "GPS P2 X BF:  "<< waypoints.GPS_P2_x_BF << " GPS P2 Y BF:  " << waypoints.GPS_P2_y_BF << endl;

   cout<< "Target P1 X BF:  "<<mission.x_target_P1_BF << " Target P1 Y BF:  " << mission.y_target_P1_BF << endl;
   cout<< "Target P2 X BF:  "<< mission.x_target_P2_BF << " Target P2 Y BF:  " << mission.y_target_P2_BF << endl;
*/
   
   
   //ROtate COntrol observatio,n points obtained from thermo and RGB camera to world frame
   Rotation_BF_to_GF_des_pos(drone.control_Thermo_point1_x, drone.control_Thermo_point1_y, drone.drone_Yaw);
  
   drone.thermo_control_obs_P1_x_world = check_x_w;
   drone.thermo_control_obs_P1_y_world = check_y_w;
   Rotation_BF_to_GF_des_pos(drone.control_Thermo_point2_x, drone.control_Thermo_point2_y, drone.drone_Yaw);
   drone.thermo_control_obs_P2_x_world = check_x_w;
   drone.thermo_control_obs_P2_y_world = check_y_w;
   
   Rotation_BF_to_GF_des_pos(drone.control_RGB_point1_x, drone.control_RGB_point1_y, drone.drone_Yaw);
   drone.RGB_control_obs_P1_x_world = check_x_w;
   drone.RGB_control_obs_P1_y_world = check_y_w;
   Rotation_BF_to_GF_des_pos(drone.control_RGB_point2_x, drone.control_RGB_point2_y, drone.drone_Yaw);
   drone.RGB_control_obs_P2_x_world = check_x_w;
   drone.RGB_control_obs_P2_y_world = check_y_w;
  
   //Obtain Thermo obs in GF for Debugging
   //Evaluate a_obs e c_obs in BF per Matlab
  float a_GF = (drone.thermo_control_obs_P2_y_world- drone.thermo_control_obs_P1_y_world)/(drone.thermo_control_obs_P2_x_world - drone.thermo_control_obs_P1_x_world);
  float c_GF= ((-1*a_GF * drone.thermo_control_obs_P1_x_world) + drone.thermo_control_obs_P1_y_world);

   drone.obs_thermo_GF << a_GF,c_GF;



  /* GPS BACKGROUND LINE */
   GPS_background_line();
   cout << "[NAVIGATION INFO] GPS LINE ERROR: " << waypoints.error_from_GPS_line << endl;
   cout << "[NAVIGATION INFO] Distance to Waypoints : " <<mission.cartesian_distance_err << endl;
  
  //Initilize Kalman Filter
   if (mission.KF_Initialization == true)
   {     //FOR GF FILTER
        Kalman_Filter.Kalman_filter_initialization(mission.x_target_P1,mission.y_target_P1, mission.x_target_P2, mission.y_target_P2, waypoints.GPS_P1_x ,  waypoints.GPS_P1_y,  waypoints.GPS_P2_x,  waypoints.GPS_P2_y);
         //FOR BF FILTER ---> Rotate Points referred to BF
       // Kalman_Filter.Kalman_filter_initialization( mission.x_target_P1_BF , mission.y_target_P1_BF ,  mission.x_target_P2_BF , mission.y_target_P2_BF ,  waypoints.GPS_P1_x_BF ,  waypoints.GPS_P1_y_BF,   waypoints.GPS_P2_x_BF,   waypoints.GPS_P2_y_BF);
        mission.KF_Initialization = false;
        drone.drone_vel_msg.linear.x = 0.0;
        drone.drone_vel_msg.linear.y = 0.0;
        ros::Duration(1).sleep();
   }
   
   //Update Kalman Filter estimation with thermal observation
   /* """"""" "
   COndizioni per entrare nel filtro:
   drone.flagDroneThermoControlPoint1 == true
   drone.flagDroneThermoControlPoint1 == true   ---> flag sono true quando la callback ha ricevuto le nuove osservazionidal topic
   mission.panel_array_initialization == false ---> il flag è true nelle prime 100 iteraioni del drone nella fase di navigazioni . Permette 
                                                    di muovere il drone lungo i waypoints GPS per iniziare a detectare i pannelli e ricevere informazioni con cui aggiornare il filtro
    drone.image_control_count < 50 --> contatore che si aggiorna SOLO quando non sono ricevute ulteriori nuove osservazioni.
                                       Per 50 iterazion da quando i flag      flagDroneThermoControlPoint1 sono falsi il filtro viene aggiornato con le ultime informazioni disponibii.                                           

   */

  
  
   if(drone.flagDroneThermoControlPoint1 == true && drone.flagDroneThermoControlPoint2 ==true && mission.panel_array_initialization == false ||  drone.THERMO_image_control_count < 100)
   {
       Kalman_Filter.pass_to_KF_class_OBS_in_GF(a_GF,c_GF);
       //Kalman_Filter.Kalman_Filter_calculate(mission.x_target_P1,mission.y_target_P1, mission.x_target_P2, mission.y_target_P2, drone.thermo_control_obs_P1_x_world, drone.thermo_control_obs_P1_y_world, drone.thermo_control_obs_P2_x_world, drone.thermo_control_obs_P2_y_world); 
      // Kalman_Filter.Kalman_Filter_calculate(  mission.x_target_P1_BF , mission.y_target_P1_BF ,  mission.x_target_P2_BF ,  mission.y_target_P2_BF , drone.control_Thermo_point1_x , drone.control_Thermo_point1_y , drone.control_Thermo_point2_x , drone.control_Thermo_point2_y  );
       Kalman_Filter.EKF_calculate(mission.x_target_P1,mission.y_target_P1, mission.x_target_P2, mission.y_target_P2, drone.control_Thermo_point1_x, drone.control_Thermo_point1_y, drone.control_Thermo_point2_x, drone.control_Thermo_point2_y,
                                                                                             drone.drone_x, drone.drone_y, drone.drone_Yaw);
       drone.xh_ = Kalman_Filter.Obtain_Kalman_filter_estimated_state();
       drone.obs = Kalman_Filter.Obtain_Kalman_filter_observation();
       drone.yh_ = Kalman_Filter.Obtain_Kalman_filter_estimated_observation();
 
       cout << "[EKF THERMO  ------->] Observations : a  " << drone.obs[0] << " c: " << drone.obs[1] << endl;
       cout << "[EKF THERMO  ------->] Estimated states: a  " << drone.xh_[0] << " c: " << drone.xh_[1] << endl;
       cout << "[EKF THERMO  ------->] Estimated Observation in BF:  a_BF " << drone.yh_[0] << " c_BF: " << drone.yh_[1] << endl; 
       
      cout << "[EKF THERMO ------->] Observation in GF:  a  " << a_GF << " c : " << c_GF << endl; 
      from_image_Thermo = true;
      from_image = true;
      if (drone.flagDroneThermoControlPoint1 == true && drone.flagDroneThermoControlPoint2== true &&  waypoints.error_from_GPS_line < 4.5) //drone.image_control_count: permette la guida guidata dal filtro anche quando non sono rileavate piu osservazioni,
                                                                                  //finche la condizione sull'errore dalla GPS line è rispettata 
      {
        //Resetto counter 
         drone.THERMO_image_control_count  = 0;
      }
       KF_estimation_exportation_in_BF_for_control_point_generation();

       
   }
   else
   {
       //Se non ci sono piu nuove osservazioni anche per oltre il numero concesso di osservazioni 
       //torno alla navigazione via GPS
       from_image_Thermo = false; 
       from_image = false;
        coming_back_to_GPS_LINE = true;
   }
   
   
   
    a_GF = (drone.RGB_control_obs_P2_y_world -drone.RGB_control_obs_P1_y_world)/(drone.RGB_control_obs_P2_x_world -drone.RGB_control_obs_P1_x_world);
    c_GF= (-1*a_GF *drone.RGB_control_obs_P1_x_world) + drone.RGB_control_obs_P1_y_world;
   
    drone.obs_RGB_GF << a_GF,c_GF;
   
  
   //Update Kalamn FIlter Estimation with RGB observation if available
   if(drone.flagDroneRGBControlPoint1 == true && drone.flagDroneRGBControlPoint2 == true && mission.panel_array_initialization == false ||  drone.RGB_image_control_count< 100 )
   {
        Kalman_Filter.pass_to_KF_class_OBS_in_GF(a_GF, c_GF);
        /*
        EKF Prende le osservazioni a_BF e c_BF nel body frame.
        */
       //Kalman_Filter.Kalman_Filter_calculate(mission.x_target_P1,mission.y_target_P1, mission.x_target_P2, mission.y_target_P2, drone.RGB_control_obs_P1_x_world, drone.RGB_control_obs_P1_y_world, drone.RGB_control_obs_P2_x_world, drone.RGB_control_obs_P2_y_world); 
       Kalman_Filter.EKF_calculate(mission.x_target_P1,mission.y_target_P1, mission.x_target_P2, mission.y_target_P2, drone.control_RGB_point1_x, drone.control_RGB_point1_y, drone.control_RGB_point2_x, drone.control_RGB_point2_y,
                                                                                             drone.drone_x, drone.drone_y, drone.drone_Yaw);
       //Kalman_Filter.Kalman_Filter_calculate(mission.x_target_P1_BF,mission.y_target_P1_BF, mission.x_target_P2_BF, mission.y_target_P2_BF, 
       //                                        drone.control_RGB_point1_x, drone.control_RGB_point1_y,drone.control_RGB_point2_x, drone.control_RGB_point2_y);
       drone.xh_ = Kalman_Filter.Obtain_Kalman_filter_estimated_state();
       drone.obs = Kalman_Filter.Obtain_Kalman_filter_observation();
       drone.yh_ = Kalman_Filter.Obtain_Kalman_filter_estimated_observation();
       cout << "[EKF RGB ------->] Observations : a  " << drone.obs[0] << " c: " << drone.obs[1] << endl;
       cout << "[EKF RGB  ------->] Estimated states: a  " << drone.xh_[0] << " c: " << drone.xh_[1] << endl;
       cout << "[EKF RGB ------->] Estimated Observation in BF:  a_BF " << drone.yh_[0] << " c_BF: " << drone.yh_[1] << endl; 

       cout << "[EKF RGB ------->] Observation in GF:  a  " << a_GF << " c : " << c_GF << endl; 

       from_image_RGB = true;
       from_image = true;
       if (drone.flagDroneRGBControlPoint1 == true && drone.flagDroneRGBControlPoint2== true &&  waypoints.error_from_GPS_line < 4.5)
      {
        //Resetto counter 
      drone.RGB_image_control_count = 0;
      }
       KF_estimation_exportation_in_BF_for_control_point_generation();
      
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
           coming_back_to_GPS_LINE = true;
         
       }

   }
   

  
  /*
  KF class return a flag whne the KF is initialized 
*/
   KF_init = Kalman_Filter.Obtain_KF_initialization_flag(); 
 
 
  evaluate_control_point(from_image); 
  
  


  drone.drone_vel_msg.angular.z  = pid_yaw->position_control_knowing_velocity(drone.yaw_des, abs(drone.drone_Yaw), 0, drone.drone_ang_vel_z); //ci andrebbe yaw 
  if (drone.drone_Yaw < 0)
  {
       drone.drone_vel_msg.angular.z = -1* drone.drone_vel_msg.angular.z;
  }




/* ###################    GAZEBO LINK ----> CONNECT TO REAL MATRICE ############
Obtain control Point D from Matrice 
Move the drone in Gazebo with the control point obtained from the MATRICE 
*/


//  /* MULSTI SIM NAVIGATION ---> LINK WITH DJI SIM 
//  -- ARdrone sosta sul pannellio finche non riceve il punto stimato espresso in BF dal MATRICE tramite ROS OVTA.
//  -- AR drone invia osservazione a MATRICE, il quale inizia a leggerle solo quando ha raggiunto il punto P1 di inzio pannello.
//  -- Quando Matrice Invia il suo punto stimato dalle osservaziioni prese da ARDrone in Gazebo, AR drone calcola le velocità in 
//  -- base al punto stimato da KF MATRICE.
//  */

// cout << "[INFO NAVIGATION]  target_from_MATRICE.x: " <<  mission.target_from_MATRICE.x << " target_from_MATRICE.y: " <<  mission.target_from_MATRICE.y << endl;
            
//   if (stop_navigation_counter > 500 && mission.target_from_MATRICE.x == 0.0 && mission.target_from_MATRICE.y == 0.0)
//   {
//       if (counter_stop_matrice == 0)
//       {

//           x = drone.drone_x;
//           y = drone.drone_y;
//          Rotation_GF_to_BF_des_pos(x, y, drone.drone_Yaw);
//          x = check_x_b;
//          y = check_y_b;
//       }
     
//       drone.x_target= x;
//       drone.y_target = y;
//       mission.GAZEBO_LINK = false; 
//       cout << "Waiting in Position: X "<< drone.x_target << "Y " << drone.y_target << endl;
//       counter_stop_matrice = counter_stop_matrice + 1;
//   }
//   else
//   {
//       mission.GAZEBO_LINK = true; 
//       stop_navigation_counter = 0;
//       counter_stop_matrice = 0;
//   }

//  if (mission.GAZEBO_LINK == true && mission.navigation_iteration_count > 70 &&  waypoints.error_from_GPS_line < 1.5)
//  {
//     cout << "[INFO NAVIGATION] NAVIGATION WITH MATRICE POINT D CONTROL POSITION " << endl;
//     drone.x_target = mission.target_from_MATRICE.x;
//     drone.y_target= mission.target_from_MATRICE.y;
//  } 


// PID control on velocity defined on body frame
 drone.drone_vel_msg.linear.x = pid_x->calculate(drone.x_target, drone.drone_x_b);
 drone.drone_vel_msg.linear.y = pid_y->calculate(drone.y_target, drone.drone_y_b);
 
//  drone.drone_vel_msg.linear.x = 0.0;
//  drone.drone_vel_msg.linear.y = 0.0;
 //drone.drone_vel_msg.linear.z = 0.005*drone.offset;
 cout<<"drone.drone_vel_msg.linear.z: " << drone.drone_vel_msg.linear.z << endl;

//IF KF IS NOT INITIALIZE Drone wait in position
if (KF_init == true)
  {
   
    cout << "[KF INIT] WAITING KF INITIALIZATION " << endl;
    drone.drone_vel_msg.linear.x = 0.0;
    drone.drone_vel_msg.linear.y = 0.0;
    drone.drone_vel_msg.linear.z = pid_z->position_control_knowing_velocity(drone.last_position_z, drone.drone_z, 0, drone.drone_lin_vel_z);
    from_image = false;
    mission.panel_array_initialization = false;

  }



 


  //Considero step

//Permette a tutte le osservaziinidel filtri di essere inizializzate correttamente 
//finche non diventa false il flag il drone è guidato via Waypoint lungo la vela 

//   if (mission.navigation_iteration_count > 100)
//   {
//        mission.panel_array_initialization = false;
//       // from_image = false;
//   }
 
if (from_image_RGB == true)
{
    //FInche flag rimane true continuo ad incrementare counter. 
    // verra resettato solo quando i flag callback sono ture 
    drone.RGB_image_control_count = drone.RGB_image_control_count  + 1;
  
}
if (from_image_Thermo == true)
{
    drone.THERMO_image_control_count = drone.THERMO_image_control_count  + 1;
}


mission.navigation_iteration_count = mission.navigation_iteration_count + 1;

mission.cartesian_distance_err = sqrt(pow(mission.x_target_P2- drone.drone_x,2) + pow(mission.y_target_P2- drone.drone_y,2)); //distanza dal raggiungere la fine del oannello

if (mission.cartesian_distance_err < 2.5 || waypoints.error_from_GPS_line > 2.5)
{
    from_image = false;
    from_image_RGB = false;
    from_image_Thermo = false;
}



//Passaggio alla mission.state 2--> Cambio di vela 

if (mission.cartesian_distance_err < 2.0)
 {
    cout<<"End Panel Reached: " << endl;
   drone.last_position_z = drone.drone_z;
   from_image = false;
   from_image_RGB = false;
   from_image_Thermo = false;
   //ROtating Yaw of 180 degree 
   drone.yaw_des = drone.drone_Yaw + M_PI; //ROtate drone
   mission.state = 2; //Cambio di array
   mission.count = mission.count + 1;
   stop_navigation_counter = 0;
   mission.navigation_iteration_count = 0;
 } 

 stop_navigation_counter = stop_navigation_counter + 1;

}





void jump_panels_array(Panel *panel,  PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
    cout<< "Reaching NEW PANELS ARRAY" << endl;
    //Reaching point P3 from point P2. P2 ---> P3 --> Cambio pannello
    mission.x_target_P1 =  panel->obtain_xcoo_panel_world_frame()[mission.count - 1]; //P1 diventa il precedente punto P2 di fine pannella
    mission.y_target_P1 =  panel->obtain_ycoo_panel_world_frame()[mission.count - 1];
                
    mission.x_target_P2 =  panel->obtain_xcoo_panel_world_frame()[mission.count];
    mission.y_target_P2 =  panel->obtain_ycoo_panel_world_frame()[mission.count];

    mission.cartesian_distance_err = sqrt(pow(mission.x_target_P2- drone.drone_x,2) + pow(mission.y_target_P2- drone.drone_y,2)); 
    cout << "mission.cartesian_distance_err: " << mission.cartesian_distance_err << endl;
    //cout << "mission.x_target_P2:  " << mission.x_target_P2 << "mission.y_target_P2:  " << mission.y_target_P2 << endl;
    drone.drone_vel_msg.angular.z  = pid_yaw -> position_control_knowing_velocity(drone.yaw_des , abs(drone.drone_Yaw), 0, drone.drone_ang_vel_z);
    cout << "YAW DES: " << drone.yaw_des << " drone.drone_Yaw: "<< abs(drone.drone_Yaw) << endl;
    
    float yaw_err = drone.yaw_des - abs(drone.drone_Yaw);
    Rotation_GF_to_BF_des_pos(mission.x_target_P2, mission.y_target_P2,  drone.drone_Yaw);
    Rotation_GF_to_BF_drone_pos( drone.drone_Yaw);

    drone.drone_vel_msg.linear.x = pid_x->calculate(check_x_b, drone.drone_x_b);
    drone.drone_vel_msg.linear.y = pid_y->calculate(check_y_b, drone.drone_y_b);
    drone.drone_vel_msg.linear.z = pid_z->position_control_knowing_velocity(drone.last_position_z, drone.drone_z, 0, drone.drone_lin_vel_z);
  
    if (mission.cartesian_distance_err < 0.2 && yaw_err < 0.1)
    {
       mission.count = mission.count + 1; //Aggiorno target e waypoints con cui aggiornare anche KF
      drone.image_control_count = 100; // inizializazzione valore grosso. --> Serve per aggiornare il filtro per un certo numero di iterazioni anche se nin sono state ricevute nuove osservazioni-- > utilizzal'ultima osservazione ricevuta 
      drone.THERMO_image_control_count = 200;
      drone.RGB_image_control_count = 200;
      mission.target_point = 2;
      mission.KF_Initialization = true;
      mission.panel_array_initialization = true; //necessaria per la corretta inizializazzione Kalman Filter
      mission.state = 1; // Incomincia nuova navigazione vela
      

   } 
}


  void publish_estimated_line(float a_estimated,  float c_estimated)
 {
   //Find two points on Inertial frame lies to the estimated line 
   Eigen::Vector2f P1(0.0,0.0);
   Eigen::Vector2f P2(1.0,0.0);
  
   P1[1] =  c_estimated; //y_P1 = a P1_x + c
   P2[1] = a_estimated* P2[0] + c_estimated;

//    //Rotate in body/camera frame the points
//   Rotation_GF_to_BF_des_pos(P1[0], P1[1], drone.drone_Yaw);
//   P1[0] = check_x_b;
//   P1[1] = check_y_b;
//  Rotation_GF_to_BF_des_pos(P2[0], P2[1], drone.drone_Yaw);
//   P2[0] = check_x_b;
//   P2[1] = check_y_b;
   
   //Publish Point 1 
   drone.P1_B = P1;
   drone.P2_B = P2;

  }



void waiting_for_parameters_optimization(PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw)
{
    //Waiting in last positiom for Parameters Optimization
    cout << " Waiting For Parameters Optimization" << endl;
    Rotation_GF_to_BF_des_pos(drone.last_position_x, drone.last_position_y,  drone.drone_Yaw);
    drone.drone_vel_msg.linear.x = pid_x->calculate(check_x_b, drone.drone_x_b);
    drone.drone_vel_msg.linear.y = pid_y->calculate(check_y_b, drone.drone_y_b);
    drone.drone_vel_msg.angular.z  = pid_yaw -> position_control_knowing_velocity(drone.yaw_des , abs(drone.drone_Yaw), 0, drone.drone_ang_vel_z);
    mission.start_optimization_task = true;


    if (mission.Optimization_completed == true)
    {
        drone.yaw_des = drone.drone_Yaw;
        drone.image_control_count = 100; // inizializazzione valore grosso. --> Serve per aggiornare il filtro per un certo numero di iterazioni anche se nin sono state ricevute nuove osservazioni-- > utilizzal'ultima osservazione ricevuta 
        drone.THERMO_image_control_count = 200;
        drone.RGB_image_control_count = 200;
        mission.target_point = 2;
        mission.KF_Initialization = true;
        mission.panel_array_initialization = true; //necessaria per la corretta inizializazzione Kalman Filter
        mission.state = 1; //Quando 1 mi muovo lungo il pannello tra punti 1 e 2
    }
    else
    {
      mission.state = 3;
    }
    
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "Solar_fligth_control_sim");

    ros::NodeHandle nh;
     
   
     // Image converter 
    
     //Writing FIle 
    std::ofstream outFile("simulation_data/sim_data_complete/panel_x_point_in_world_frame.txt");
    std::ofstream outFile1("simulation_data/sim_data_complete/panel_y_point_in_world_frame.txt");
    std::ofstream outFile2("simulation_data/sim_data_complete/GPS_x_point_in_world_frame.txt");
    std::ofstream outFile3("simulation_data/sim_data_complete/GPS_y_point_in_world_frame.txt");
    //std::ofstream outFile4("simulation_data/sim_data_complete/altitude");

    bool Take_off = false;
    bool flag_even = false;
    bool flag_end_point = false;
    double pass_to_class_initialization_matrices; 
    
    bool control_altitude_by_image_flag = false;
    //PARAMETRS 
    float n_panels = 4;
    float distance_between_panels = 2.2; //1.84;
    int configuration = 0;  //1 se si vuole la configurazione con panneli a ferro di cavallo
    
    //Initialize GPS class 
    waypoints.gamma = 0.0*M_PI/180; //Rotational error in rad
    waypoints.delta =  0.0; //Traslational error along x respect panel frame located in panel 1
    waypoints.eta = 0.0; //Traslational error along y

    //Per le misure di ciasucna configurazione fare riferimento al file ardrone_testworld.world
    Eigen::Vector2f panel_center_W(12.74,2); //(12.74,2) //6.37, 2)
    float theta = 0; //45.0 * M_PI/180;
    float size = 1.0;
    float length = 22; //22 11

    //Load Parameters from launch file 
    //Take ROS PARAMETERS
    
    nh.getParam("/solar_param/desired_waiting_KF_init_it", pass_to_class_initialization_matrices);
    nh.getParam("/solar_param/z_des", drone.z_des);
    nh.getParam("/solar_param/initial_state", mission.state);  
    nh.getParam("/solar_param/control_altitude_by_image", control_altitude_by_image_flag);
    nh.getParam("/solar_param/Detection_param_Optimization_enabled", mission.Optimization_enabled);
    //Parameters relative to Panels and Waypoints dispositions
    nh.getParam("/solar_param/n_panels", n_panels);
    nh.getParam("/solar_param/distance_between_panels", distance_between_panels);
    nh.getParam("/solar_param/configuration", configuration);
    nh.getParam("/solar_param/gamma",  waypoints.gamma);
    nh.getParam("/solar_param/delta",  waypoints.delta);
    nh.getParam("/solar_param/eta", waypoints.eta);
     
    
    
    waypoints.gamma = waypoints.gamma*M_PI/180;
    
    //Create a vector of structure for each panel
    Panel panel(panel_center_W, theta, size, length);
    
    //Obtain GPS error to define GPS waypoints for point P1 P2 of start and end
     panel.pass_to_class_GPS_error(waypoints.gamma ,  waypoints.eta ,  waypoints.delta );

    //Place panel centers and Start P1 and end P2 in map for each panel
     panel.init(n_panels, distance_between_panels, configuration);
     
      //Write panel points in world frame
     for (int i = 0; i < panel.obtain_xcoo_panel_world_frame().size(); i++)
    {
        outFile << panel.obtain_xcoo_panel_world_frame()[i] << "\n";
        outFile1 << panel.obtain_ycoo_panel_world_frame()[i] << "\n";
        cout << "Panel " << i << " x coo in W: " << panel.obtain_xcoo_panel_world_frame()[i] << "," << panel.obtain_ycoo_panel_world_frame()[i] << "\n";
    }
     //Write GPS waypoints relative to panel points in world frame
     for (int i = 0; i < panel.obtain_waypoints_x_coo_world_frame().size(); i++)
    {
        outFile2 << panel.obtain_waypoints_x_coo_world_frame()[i] << "\n";
        outFile3 << panel.obtain_waypoints_y_coo_world_frame()[i] << "\n";
        
        cout << "Panel " << i << " GPS waypoint in W: " << panel.obtain_waypoints_x_coo_world_frame()[i] << "," << panel.obtain_waypoints_y_coo_world_frame()[i] << "\n";
    }
    
    //Save in struct waypoints the GPS panel waypoints obtained from class Panel
    for  (int i = 0; i < panel.obtain_waypoints_x_coo_world_frame().size(); i++)
    {
        waypoints.waypoints_x_coo_world_frame.push_back(panel.obtain_waypoints_x_coo_world_frame()[i]);
        waypoints.waypoints_y_coo_world_frame.push_back(panel.obtain_waypoints_y_coo_world_frame()[i]);
        waypoints.waypoints_x_coo_gps_frame.push_back(panel.obtain_waypoints_x_coo_gps_frame()[i]);
        waypoints.waypoints_y_coo_gps_frame.push_back(panel.obtain_waypoints_y_coo_gps_frame()[i]);

    }
       
    //Define Gains Controller
    float Kp_z = 1.5;
    float Kd_z = 0.5;
    float Kp_yaw = 0.6;
    float Kd_yaw = 0.3;
    float Kp_x = 0.4;
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
    
    PID pid_x = PID(dt, 1.5, -1.5, Kp_x, Kd_x, Ki_x);
    PID pid_y = PID(dt, 1.5, -1.5, Kp_y, Kd_y, Ki_y);
     
    PID pid_z = PID(dt, 1.5, -1.5, Kp_z, Kd_z, 0.01);
    PID pid_yaw = PID(dt, 1, -1, Kp_yaw, Kd_yaw, 0.01);


     Eigen::Matrix2f R;
    Eigen::Matrix2f Px;

   R << 0.8, 0.0,  
     0.0, 2;
   Px << 4.0, 0.0,
    0.0, 4.0;
     
Kalman_Filter.pass_to_class_initialization_matrices(Px, R, pass_to_class_initialization_matrices);


    //General Variables 
    float disturb = 0.0;
    //error on the drone body frame in PD controller
    float e_x = 0.0;
    float e_y = 0.0;
    float yaw_des = 0.0;
    float yaw_des_old = 0.0;
    float yaw_err = 0.0;
    float cartesian_distance_err = 0.0; //distance between final panel point
    float cartesian_distance_err1 = 0.0; //distance between setpoints
    float drone_yaw_degree = 0.0;
    float distance_threshold = 0.2;
    int checkpoint = 1;

    //Publisher Topics 
    ros::Publisher takeOff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    ros::Publisher Land = nh.advertise<std_msgs::Empty>("/ardrone/land",1);
    ros::Publisher vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher pub_P1_estimated  = nh.advertise<geometry_msgs::Point>("/P1_estimated_control_point",1);
    ros::Publisher pub_P2_estimated =  nh.advertise<geometry_msgs::Point>("/P2_estimated_control_point",1);
    ros::Publisher KF_init =  nh.advertise<std_msgs::Bool>("/dji_osdk_ros/KF_init",30);
    ros::Publisher start_OPT_task =  nh.advertise<std_msgs::Bool>("/dji_osdk_ros/optimization_start",30);

    //Publish drone position to camera in order to create idealistic gimbal
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient client_RGB = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    
    
    ros::Subscriber odom_drone_sub = nh.subscribe("/ground_truth/state", 50, drone_odom_callback);
    ros::Subscriber imu_drone_sub = nh.subscribe("/ardrone/imu", 5, drone_Imu_callback);
    ros::Subscriber vel_drone_sub = nh.subscribe("/fix_velocity", 1, drone_fix_Vel_callback);
    //Subscribe to the control points extracted from Thermal images 
    ros::Subscriber control_point_1 = nh.subscribe("/Thermo_control_point_1", 5, drone_Thermo_control_point1_callback);
    ros::Subscriber control_point_2 = nh.subscribe("/Thermo_control_point_2", 5, drone_Thermo_control_point2_callback);
    //Subscribe to DJI local position 
    ros::Subscriber DJI_local_pos = nh.subscribe("/dji_osdk_ros/local_position", 1, DJI_local_pos_callback);
    
    //Subscribe to the second control points extracted from Thermal images 
    //ros::Subscriber control_point_11 = nh.subscribe("/Thermo_control_point2_1", 1, drone_Thermo_control_point1_callback2);
    //ros::Subscriber control_point_22 = nh.subscribe("/Thermo_control_point2_2", 1, drone_Thermo_control_point2_callback2);
    
    //Subscribe to the control points extracted from RGB images 
    ros::Subscriber control_RGB_point_1 = nh.subscribe("/RGB_control_point_1", 5, drone_RGB_control_point1_callback);
    ros::Subscriber control_RGB_point_2 = nh.subscribe("/RGB_control_point_2", 5, drone_RGB_control_point2_callback);
    
    ros::Subscriber altitude_offset = nh.subscribe("/dji_osdk_ros/altitude_offset", 5, altitude_offset_callbak);
    ros::Subscriber parameter_opt = nh.subscribe("/dji_osdk_ros/parameter_opt_completed", 5, parameter_opt_callback);
    //##### Gazebo LINK with Matrice  
    ros::Subscriber MATRICE_D_point = nh.subscribe("Gazebo_link/point_D_for_control_in_BF", 5, Matrice_point_D_callback);
    // ros::Publisher Gazebo_P1_thermo_pub  = nh.advertise<geometry_msgs::Point>("Gazebo_link/THERMO_control_point_1",10);
    // ros::Publisher Gazebo_P1_thermo_pub  = nh.advertise<geometry_msgs::Point>("Gazebo_link/THERMO_control_point_1",10);
    // ros::Publisher Gazebo_P1_thermo_pub  = nh.advertise<geometry_msgs::Point>("Gazebo_link/THERMO_control_point_1",10);
    // ros::Publisher Gazebo_P1_thermo_pub  = nh.advertise<geometry_msgs::Point>("Gazebo_link/THERMO_control_point_1",10);
    

    //WRITE ON TXT simulation_data/sim_data_complete
        // std::ofstream outFile4("simulation_data/sim_data_complete/des_x_vel.txt");
        // std::ofstream outFile5("simulation_data/sim_data_complete/des_y_vel.txt");
        std::ofstream outFile6("simulation_data/sim_data_complete/des_z_vel.txt");
        std::ofstream outFile7("simulation_data/sim_data_complete/x_vel.txt");
        std::ofstream outFile8("simulation_data/sim_data_complete/y_vel.txt");
        std::ofstream outFile9("simulation_data/sim_data_complete/z_vel.txt");
        // std::ofstream outFile10("simulation_data/sim_data_complete/drone_x_pos_saved.txt");
        // std::ofstream outFile11("simulation_data/sim_data_complete/drone_y_pos_saved.txt");
        // std::ofstream outFile13("simulation_data/sim_data_complete/y_error_VISION_LINE.txt");
        // std::ofstream outFile14("simulation_data/sim_data_complete/roll.txt");
        // std::ofstream outFile15("simulation_data/sim_data_complete/a_GPS.txt");
        // std::ofstream outFile16("simulation_data/sim_data_complete/c_GPS.txt");
        std::ofstream outFile17("simulation_data/sim_data_complete/a_est.txt");
        std::ofstream outFile18("simulation_data/sim_data_complete/c_est.txt");
        // std::ofstream outFile19("simulation_data/sim_data_complete/x_target.txt");
        // std::ofstream outFile20("simulation_data/sim_data_complete/y_target.txt");
        std::ofstream outFile21("simulation_data/sim_data_complete/a_obs_BF.txt");
        std::ofstream outFile22("simulation_data/sim_data_complete/c_obs_BF.txt");
        std::ofstream outFile37("simulation_data/sim_data_complete/a_obs_est.txt");
        std::ofstream outFile38("simulation_data/sim_data_complete/c_obs_est.txt");
        std::ofstream outFile39("simulation_data/sim_data_complete/a_obs_thermo_GF.txt");
        std::ofstream outFile40("simulation_data/sim_data_complete/c_obs_thermo_GF.txt");
        std::ofstream outFile41("simulation_data/sim_data_complete/a_obs_RGB_GF.txt");
        std::ofstream outFile42("simulation_data/sim_data_complete/c_obs_RGB_GF.txt");
        

        //Da cancellare poi 
        // std::ofstream outFile23("simulation_data/sim_data_complete/KF_std_dev.txt");
        
        // std::ofstream outFile24("simulation_data/sim_data_complete/a_RGB_est.txt");
        // std::ofstream outFile25("simulation_data/sim_data_complete/c_RGB_est.txt");
        // std::ofstream outFile26("simulation_data/sim_data_complete/KF_RGB_std_dev.txt");
        // std::ofstream outFile27("simulation_data/sim_data_complete/a_RGB_obs.txt");
        // std::ofstream outFile28("simulation_data/sim_data_complete/c_RGB_obs.txt");
        // std::ofstream outFile29("simulation_data/sim_data_complete/Eigen_Px_KF_RGB.txt");
        // std::ofstream outFile30("simulation_data/sim_data_complete/Eigen_Px_KF_thermal.txt");
        std::ofstream outFile31("simulation_data/sim_data_complete/drone_x_pos.txt");
        std::ofstream outFile32("simulation_data/sim_data_complete/drone_y_pos.txt");
        std::ofstream outFile33("simulation_data/sim_data_complete/drone_z_pos.txt");
        std::ofstream outFile34("simulation_data/sim_data_complete/error_from_GPS_line.txt");
        // std::ofstream outFile35("simulation_data/sim_data_complete/error_from_vision_line.txt");
        std::ofstream outFile36("simulation_data/sim_data_complete/Yaw.txt");

    
    
    mission.state = 0;
    //Counter 
    int count = 0;
    int count_setpoint = 0;
    bool gps_waypoint = false;
    bool flag_create_setpoint = true;
    bool coming_back_to_GPS_path = false;
    bool flag_altitude_init = false;
    int count_hovering = 0;
    int counter_altitude_init = 0;

    ros::Rate r(20);
    while(nh.ok())
    {
     /*##################################################################################################################################*/
    
        //Loop local variables 
        float vel_x = 0.0;
        float vel_y = 0.0;
        float y = 0.0;

        drone.z_des = 5.0 ;
        //Desired_altitude
        
        
        if (Take_off == false)
        {
            takeOff.publish(myMsg);
            //Increasing altitude 
            drone.drone_vel_msg.linear.z = pid_z.position_control_knowing_velocity(drone.z_des, drone.drone_z, 0, drone.drone_lin_vel_z);
            drone.yaw_des = atan2(panel.obtain_ycoo_panel_world_frame()[count + 1],panel.obtain_xcoo_panel_world_frame()[count + 1]); //Define yaw des to poiunt to the first P1 start panel
            drone.drone_vel_msg.angular.z = pid_yaw.position_control_knowing_velocity(drone.yaw_des, drone.drone_Yaw, 0, drone.drone_ang_vel_z); //Yaw PID control
            //cout<<"drone.drone_vel_msg.linear.z: "<< drone.drone_vel_msg.linear.z<< endl;
            // publish the message
            vel.publish(drone.drone_vel_msg);
            

            drone.flagDroneOdom = false;
            drone.flagDroneImu = false;
            drone.flagDroneFix_vel = false;
            
            if (drone.drone_z < drone.z_des - 0.5)
            {
                cout << "Take_Off --> altitude: " << drone.drone_z << endl;
                ros::spinOnce();
                r.sleep();
                continue;
            }
            else
            {
                 //Initialize distance error 
                 
                Take_off = true;
            }
        }
            
       // Maintain desired altitude and attitude
        drone.drone_vel_msg.linear.z = pid_z.position_control_knowing_velocity(drone.z_des, drone.drone_z, 0, drone.drone_lin_vel_z); //Kp_z * (drone.z_des - drone.drone_z) + Kd_z * (0 - drone.drone_lin_vel_z);
        drone_yaw_degree = drone.drone_Yaw * (180/M_PI);
        

        switch(mission.state)
        {
            case 0:
            //Mission Initialization --> Reach point P1 panel after Take Off
                start(&panel, &pid_x, &pid_y, &pid_z, &pid_yaw);
            break;

            case 1: 
            //Reaching end of panel point P2 from point P1
               navigation(&panel, &pid_x, &pid_y, &pid_z, &pid_yaw);
               
            break;

            case 2:
            //Cambio di vela  ----> Testare meccanismo per cambio di vela facendo riferimento alla distanza tra waypoints e la distanza percorsa da drone
               jump_panels_array(&panel,  &pid_x, &pid_y, &pid_z, &pid_yaw);
            break;

            case 3:
            //Waiting for parameters Optimization
               waiting_for_parameters_optimization(&pid_x, &pid_y, &pid_z, &pid_yaw);
        }

     //Publish Estimated Line from KF to add to the elaborated images taken from camera
     publish_estimated_line(drone.xh_[0], drone.xh_[1]);
     geometry_msgs::Point point1;
     point1.x = drone.P1_B[0];
     point1.y = drone.P1_B[1];
  
     geometry_msgs::Point point2;
     point2.x = drone.P2_B[0];
     point2.y = drone.P2_B[1];

     pub_P1_estimated.publish(point1);
     pub_P2_estimated.publish(point2);
  
   // publish velocity message
     saturation(drone.drone_vel_msg.linear.x, drone.drone_vel_msg.linear.y, drone.drone_vel_msg.linear.x, drone.drone_vel_msg.angular.z);
     vel.publish(drone.drone_vel_msg);
    
    //Publish KF INIT FLAG for reset of the altitude width array 
    std_msgs::Bool KF_initialization;
    if  (mission.panel_array_initialization == true )
    {
        flag_altitude_init = true;
       
    }
    
    if (flag_altitude_init == true && control_altitude_by_image_flag == true)
    {
        cout<< "Initialize PIXEL OFFSET for altitude control " << counter_altitude_init << endl;
        if (counter_altitude_init > 200) 
        {
            flag_altitude_init = false;
            counter_altitude_init  = 0;
        }
           
        counter_altitude_init = counter_altitude_init + 1;
    }
    
    KF_initialization.data =  flag_altitude_init;
    KF_init.publish(KF_initialization);


//ANCORA DA AGGIUNGERE NELLO SCRIPT SU MATRICE
    //Publish OPT start flag 
    if (mission.Optimization_enabled == true)
    {
        //Publish Flag to Pythons script fpr detection to start the detection
        std_msgs::Bool OPT_start;
        OPT_start.data = mission.start_optimization_task;
        start_OPT_task.publish(OPT_start);

    }
    
    
    //##########################Export RGB and Thermo Observation to Matrice Simulation ###################################
//    geometry_msgs::Point publish_to_DJI_Thermo_obs_P1;
//    publish_to_DJI_Thermo_obs_P1.x = drone.control_Thermo_point1_x;
//    publish_to_DJI_Thermo_obs_P1.y = drone.control_Thermo_point1_y;
//    Gazebo_P1_thermo_pub.publish(publish_to_DJI_Thermo_obs_P1);
   
   //######################################################



/*
  //Define when it is required to switch control from image navigation to GPS navigation
  if (drone.flagDroneThermoControlPoint1 == false && drone.flagDroneRGBControlPoint1 == false && mission.state > 0 && mission.panel_array_initialization == false)
  {
      from_image = false;
      drone.image_control_count = drone.image_control_count + 1; //aggiorno il contatore quando non ho piu osservazioni quindi nessuna osservazione del filtro
  }
  */
  
 /*
  cout << "drone.image_control_count: " << drone.image_control_count << endl;
  cout << "drone.image_thermo_count: " << drone.thermo_detection_count << endl;
  cout << "drone.RGB_detection_count: " << drone.RGB_detection_count << endl;
  cout<< "mission.cartesian_distance_err : " <<mission.cartesian_distance_err << endl;
  cout << "from_image: " <<from_image << endl;
*/

//Evaluate a_obs e c_obs in BF per Matlab
 float a_BF = (drone.control_Thermo_point2_y - drone.control_Thermo_point1_y)/(drone.control_Thermo_point2_x - drone.control_Thermo_point1_x);
 float c_BF = ((-1*a_BF * drone.control_Thermo_point1_x) + drone.control_Thermo_point1_y);

// Write Drone Position and Yaw on TXT file
if (isnan(drone.yh_[0]) == 0|| isnan(drone.yh_[1]) == 0 || isnan(drone.xh_[0]) == 0 ||isnan(a_BF))
{
 outFile21 << a_BF <<"\n";
 outFile22 << c_BF << "\n";
 outFile31 << drone.drone_x << "\n";
 outFile32 << drone.drone_y << "\n";
 outFile36 << drone.drone_Yaw << "\n";

//Scrivo yh_
 outFile37 << drone.yh_[0] << "\n";
 outFile38 << drone.yh_[1] << "\n";
 //Scrivo xh_
 outFile17 << drone.xh_[0] << "\n";
 outFile18 << drone.xh_[1] << "\n";

 //OBs in GF
 
 outFile39 << drone.obs_thermo_GF[0] << "\n";
 outFile40 << drone.obs_thermo_GF[1] << "\n";
 outFile41 << drone.obs_RGB_GF[0] << "\n";
 outFile42 << drone.obs_RGB_GF[1] << "\n";

 outFile7 << drone.drone_lin_vel_x << "\n";
 outFile8 << drone.drone_lin_vel_y<< "\n";

}

// Save values related to altitude 
outFile9 << drone.drone_lin_vel_z<< "\n";
outFile6 << drone.drone_vel_msg.linear.z<< "\n";
outFile33 << drone.drone_z << "\n";

outFile34 << waypoints.error_from_GPS_line<< "\n";



/* ################################################################################## */

// drone.drone_x = 0.5*drone.MATRICE_local_position_x;
// drone.drone_y = 0.5*drone.MATRICE_local_position_y;
// drone.drone_z = drone.MATRICE_local_position_z;;
// drone.drone_Yaw = 0.0;
// cout << "MATRICE_local_position.x: " << drone.MATRICE_local_position_x << endl;

//################## Define Bottom camera position relative to drone position-> idealistic Gimbal: 
        gazebo_msgs::ModelState bottom_camera_pose;
        bottom_camera_pose.model_name = (std::string) "camera_box";
        bottom_camera_pose.reference_frame = (std::string) "world";
        bottom_camera_pose.pose.position.x = drone.drone_x;
        bottom_camera_pose.pose.position.y = drone.drone_y;
        bottom_camera_pose.pose.position.z = drone.drone_z- 0.15;
        
        //Conversion of eulerian angle to quaternion for model state 
        tf::Matrix3x3 obs_mat;
        obs_mat.setEulerYPR(drone.drone_Yaw,0.0,0.0);
  
        tf::Quaternion q_tf;
        obs_mat.getRotation(q_tf);
        bottom_camera_pose.pose.orientation.x =  q_tf.getX();
        bottom_camera_pose.pose.orientation.y = q_tf.getY(); //Orientazione al fine di avere la camera sempre puntata verso il basso 
        bottom_camera_pose.pose.orientation.z =  q_tf.getZ();
        bottom_camera_pose.pose.orientation.w = q_tf.getW();
       
        //Call Service 
        gazebo_msgs::SetModelState srv;
        srv.request.model_state = bottom_camera_pose;
        
        if(client.call(srv))
        {
        //ROS_INFO("camera box pose updated!!");
        }
        else
        {
        ROS_ERROR("Failed to update camera_box: Error msg:%s",srv.response.status_message.c_str());
        } 
        //##########################################################
        
       
        //############################# Define RGB up camera position of the gimbal --> folow the drone position  ######################
        gazebo_msgs::ModelState bottom_camera_pose_RGB;
        bottom_camera_pose_RGB.model_name = (std::string) "camera_box_RGB";
        bottom_camera_pose_RGB.reference_frame = (std::string) "world";
        bottom_camera_pose_RGB.pose.position.x = drone.drone_x;
        bottom_camera_pose_RGB.pose.position.y = drone.drone_y;
        bottom_camera_pose_RGB.pose.position.z = drone.drone_z + 0.1;
        
        //Conversion of eulerian angle to quaternion for model state 
        
        obs_mat.setEulerYPR(drone.drone_Yaw,0.0,0.0);
  
       
        obs_mat.getRotation(q_tf);
        bottom_camera_pose_RGB.pose.orientation.x =  q_tf.getX();
        bottom_camera_pose_RGB.pose.orientation.y = q_tf.getY(); //Orientazione al fine di avere la camera sempre puntata verso il basso 
        bottom_camera_pose_RGB.pose.orientation.z =  q_tf.getZ();
        bottom_camera_pose_RGB.pose.orientation.w = q_tf.getW();
       
        //Call Service 
        gazebo_msgs::SetModelState srv2;
        srv2.request.model_state = bottom_camera_pose_RGB;
        
        if(client_RGB.call(srv2))
        {
        //ROS_INFO("camera box pose updated!!");
        }
        else
        {
        ROS_ERROR("Failed to update camera_box: Error msg:%s",srv2.response.status_message.c_str());
        }
       
       //################################
        
  






  drone.flagDroneOdom = false;
  drone.flagDroneImu = false;
  drone.flagDroneFix_vel = false;
  drone.flagDroneThermoControlPoint1 = false;
  drone.flagDroneThermoControlPoint2 = false;
  drone.flagDroneRGBControlPoint1 = false;
  drone.flagDroneRGBControlPoint2 = false;
  drone.flagDroneMatricePointD = false;
  drone.flagMatriceLocalPos = false;
  drone.flagAltitudeOffset = false;
  mission.flagParametersOptimization = false;

  time = time + dt;
  count_hovering = count_hovering + 1;

  
  
  ros::spinOnce();
  r.sleep();
 }
 return 0;
}
     
    

