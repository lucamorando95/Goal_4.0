#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "gazebo_msgs/SetModelState.h"
#include <fstream>
#include "/home/lucamora/catkin_ws/src/solar_panel_project/include/solar_panel_project/pid.h"
#include <unistd.h>

#include <eigen3/Eigen/Dense>


/* Questo codice si basa sul sistema di navigazione definito nel body frame del robot:
 * Viene calcolata l'equazione della retta passante per i punti P1 e P2 (inizio e fine pannello) nel body frame.
 * Dall'equazione della retta viene ricavato il vettore parallelo e perpendicolare a quest'ultima dal punto O = (0,0) del body frame
 * Conoscendo i punti nel body frame dove puntano questi vettori:
 * Calcolo intersezione tra vettore perpendicolare alla retta r e la retta r stessa passante per P1 e P2.
 * Calcolo retta parallela alla retta su cui giace il vetore perpendicolare passante per il punto a cui punta il vettore parallelo alla retta.
 * Trovo l'intersezione di questa retta con la retta r ottenendo un punto di controllo D che giace sulla retta r stessa.
 * 
 * Il punto di controllo dipende dall'errore traslazionale tra l'origine del body frame e la retta e dal coeficiente moltiplicativo del vettore parallelo a quest'ultima.
 * 
 * Piu informazioni in IPAD.
 */
 
 
using namespace std;

#define N 2  
// Solar Panel Position and Size
unsigned int sleep(unsigned int seconds);


struct Structure {
    float x_center;
    float y_center;
    float P1_x;
    float P1_y;
    float P2_x;
    float P2_y;
    float length;
    float size;
    float theta;
    
    int target_point = 1; //Permeytte di definire la direzione del pannello
    //Coordinates P1, P2 point in world frame
    vector <float> xcoo_panel_world_frame; //Contiene punti P1 P2 per ciascun pannello nell'ordine P1 P2 P2 P1 ecc
    vector <float> ycoo_panel_world_frame;
    
    //Coo P1 P2 in panel frame 
    vector <float> xcoo_panel_frame; //Contiene punti P1 P2 per ciascun pannello nell'ordine P1 P2 P2 P1 ecc
    vector <float> ycoo_panel_frame;
    
    //Center coordinate in world frame
    vector <float> xcenter_panel_world_frame;
    vector <float> ycenter_panel_world_frame;
    //Center coordinates in GPS frame
    vector <float> xcenter_panel_frame;
    vector <float> ycenter_panel_frame;
    
   
};



struct Waypoint_GPS {
    float delta = 0.0; //Fattore di traslazione rispetto panel frame (locato nel cnetro pannello 1) su asse x
    float eta = 0.0; ///Fattore di traslazione rispetto panel frame (locato nel cnetro pannello 1) su asse y
    float gamma = 0.0; //rotazione sistema di waypoint gps rispetto panel frame 
    //Coordinata punto 1 (riferimento) del GPS. Da inizializzare in main con coordinata P1_x P1_y del primo pannello
    float GPS1_x = 0.0;
    float GPS1_y = 0.0;
    
    vector <float> waypoints_x_coo_gps_frame;
    vector <float> waypoints_y_coo_gps_frame;
    vector <float> waypoints_x_coo_world_frame;
    vector <float> waypoints_y_coo_world_frame;
    
} waypoints;

// Function Declaration --> la struct viene passata a queste funzionu
//void create_GPS_coo_waypoint_map(Waypoint_GPS, int length , int n_panels, int distance_between_panels);
//void transformation_waypoint_from_gps_frame_to_world(Waypoint_GPS, float theta);

struct Drone {
    float drone_x;
    float drone_y;
    float drone_z;
    float drone_x_b;
    float drone_y_b;
    float desired_z;
    double drone_ang_vel_z;
    
    //Save Last Important Position 
    float last_x;
    float last_y;
    
    double drone_Yaw;
    double drone_roll;
    double drone_lin_vel_x;
    double drone_lin_vel_y;
    double drone_lin_vel_z;
    
    //Vel on body frame
    double drone_lin_vel_x_b;
    double drone_lin_vel_y_b;
    
    //Point
    
    //Next P1 Panel start point--> to execute passage from a panel to other
    vector <float> next_point_P1_x;
    vector <float> next_point_P1_y;
    
    //Control point D coordinates from GPS 
    float control_x_coo = 0.0;
    float control_y_coo = 0.0;
    
    //Final Control Point D from Thermal images 
    float image_control_x_coo = 0.0;
    float image_control_y_coo = 0.0;
    float image_control_x_coo_old = 0.0;
    float image_control_y_coo_old = 0.0;
    
    //Final Control Point D from RGB images 
    float image_RGB_control_x_coo = 0.0;
    float image_RGB_control_y_coo = 0.0;
    float image_RGB_control_x_coo_old = 0.0;
    float image_RGB_control_y_coo_old = 0.0;
    
    
    //COntrol point D1 coordinates from Thermal image line
    float control_Thermo_point1_x = 0.0;
    float control_Thermo_point1_y = 0.0;
    float control_Thermo_point2_x = 0.0;
    float control_Thermo_point2_y = 0.0;
    
    //Control point obtained frome the nearest second line 
    float control_Thermo_point11_x = 0.0;
    float control_Thermo_point11_y = 0.0;
    float control_Thermo_point22_x = 0.0;
    float control_Thermo_point22_y = 0.0;
    
    //COntrol point D1 coordinates from RGB image line
    float control_RGB_point1_x = 0.0;
    float control_RGB_point1_y = 0.0;
    float control_RGB_point2_x = 0.0;
    float control_RGB_point2_y = 0.0;
    
    
    //Counter to switch from the Vision control to the GPS Control
    int switch_counter = 0.0;
    int switch_RGB_counter = 0.0;
    int switch_control = 2; //Condition for switch
    
    
};

struct Kalman_Filter_RGB{
    //Observation Vector
    vector <float> a_obs_v;
    vector <float> c_obs_v;
    
    //State Matrix
    Eigen::Matrix2d A;
    
       
    //Observation matrix
    Eigen::Matrix2d C;
    
    // State Vector 
    Eigen::Vector2d X_old;
    Eigen::Vector2d X;
    
    //Observation vector 
    Eigen::Vector2d y;
   
 
   
    //Estimated Observation
    Eigen::Vector2d yh_;
   
    
    //A priori Estimated State
  
    Eigen::Vector2d xh_;
    
    Eigen::Vector2d xh_old;
    
    //A posteriori (updated) estimate of the current state 
    Eigen::Vector2d xh;
   
    
    //A priori estiumate of the state covariance matrix
    Eigen::Matrix2d Px_;
    
    //A posteriori (updated) state Covariance matrix
    Eigen::Matrix2d Px;
    
    
    //COvariace Noise measurement matrix
       Eigen::Matrix2d R;

    //Kalman Gain 
    Eigen::Matrix2d K;
   
    //Store Eigen Values of the Px matrix
    Eigen::Vector2d eigen_Px;
 
   //Obs std
    double stdev = 0.0;
    
    //a, c parameters estimated in body frame
    float a_b = 0.0;
    float c_b = 0.0;
    
    
    //Last Position when the KF was Active 
    float quad_y_old = 0;
    
    int switch_point = 1;
    //Kalman count
    int K_count = 0;
    
    //Serve per sapere se lo stato del filtro RGB viene aggiornato, quindi se sono diposnibili osservazioni veritiere.
    //Lo stato se aggiornao puo essere utilizzato dal filtro in cascata.
    bool RGB_update = false;
   
    bool initialization;
};

struct Kalman_Filter_thermo{
    //Observation Vector
    vector <float> a_obs_v;
    vector <float> c_obs_v;
    
    //State Matrix
    Eigen::Matrix2d A;
    
       
    //Observation matrix
    Eigen::Matrix2d C;
    
    // State Vector 
    Eigen::Vector2d X_old;
    Eigen::Vector2d X;
    
    //Observation vector 
    Eigen::Vector2d y;
   
 
   
    //Estimated Observation
    Eigen::Vector2d yh_;
   
    
    //A priori Estimated State
  
    Eigen::Vector2d xh_;
    
    Eigen::Vector2d xh_old;
    
    //A posteriori (updated) estimate of the current state 
    Eigen::Vector2d xh;
   
    
    //A priori estiumate of the state covariance matrix
    Eigen::Matrix2d Px_;
    
    //A posteriori (updated) state Covariance matrix
    Eigen::Matrix2d Px;
    
    
    //COvariace Noise measurement matrix
       Eigen::Matrix2d R;

    //Kalman Gain 
    Eigen::Matrix2d K;
   
   //Eigen values vector of Px matrix
    Eigen::Vector2d  eigen_Px;
   
   //Obs std
    double stdev = 0.0;
    
    //a, c parameters estimated in body frame
    float a_b = 0.0;
    float c_b = 0.0;
    
    
    //Last Position when the KF was Active 
    float quad_y_old = 0;
    
    int switch_point = 1;
    //Kalman count
    int K_count = 0;
    
    //Serve per sapere se lo stato del filtro termico viene aggiornato, quindi se sono diposnibili osservazioni veritiere.
    //Lo stato se aggiornato puo essere utilizzato dal filtro in cascata.
    bool thermal_update = false;
   
    bool initialization;
};

Drone drone;
Kalman_Filter_RGB KF_RGB;
Kalman_Filter_thermo KF;


bool flagDroneOdom = false;
bool flagDroneImu = false;
bool flagDroneFix_vel = false;
bool flagDroneThermoControlPoint1 = false;
bool flagDroneThermoControlPoint2 = false;
bool flagDroneThermoControlPoint21 = false; 
bool flagDroneThermoControlPoint22 = false;

bool flagDroneRGBControlPoint1 = false;
bool flagDroneRGBControlPoint2 = false;



//Global Flag 
bool init_panel = false;  //Quando il drone arriva al termine del pannello il flag è true e lo rimane finche non riconosce qualcosq dalle immagini
                          //Il flag true permette di continuare kla navigazione GPS finche non viene riconosciuto qualcosa, a quel punto switcha

//Desired Position in Drone body frame
float check_x_b = 0.0; //Checkpoint coordinate in drone body frame
float check_y_b = 0.0;

//Desired position from body frame to world frame
float check_x_w = 0.0;
float check_y_w = 0.0;


float error_from_GPS_line = 0.0;
float y_error_with_vision_line = 0.0;

//Da cancellare poi 
 std::ofstream outFile23("KF_std_dev.txt");



nav_msgs::Odometry drone_odom;
sensor_msgs::Imu drone_imu;
geometry_msgs::Vector3Stamped drone_fix_vel;
geometry_msgs::Point drone_Thermo_control_point1;
geometry_msgs::Point drone_Thermo_control_point2;
geometry_msgs::Point drone_Thermo_control_point11;
geometry_msgs::Point drone_Thermo_control_point22;
geometry_msgs::Point drone_RGB_control_point1;
geometry_msgs::Point drone_RGB_control_point2;

std_msgs::Empty myMsg;


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
    flagDroneOdom = true;
}

void drone_Imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    drone_imu = *msg;
    drone.drone_ang_vel_z = drone_imu.angular_velocity.z;
    flagDroneImu = true;
}

void drone_fix_Vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    drone_fix_vel = *msg;
    drone.drone_lin_vel_x = drone_fix_vel.vector.x;
    drone.drone_lin_vel_y = drone_fix_vel.vector.y;
    drone.drone_lin_vel_z = drone_fix_vel.vector.z;
    flagDroneFix_vel = true;
}




// OBTAIN CONTROL POINTS FROM THERMAL AND RGB IMAGES //
//#############  Desired Control POint 1 from Thermal images  ---> Usati per ottenere la retta esportata nel body vrame dall'image frame 
void drone_Thermo_control_point1_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    drone_Thermo_control_point1 = *msg;
   
    drone.control_Thermo_point1_x = drone_Thermo_control_point1.x;
    drone.control_Thermo_point1_y= drone_Thermo_control_point1.y;
    flagDroneThermoControlPoint1 = true;
     
}

//Desired COntrol Point 2 from thermal Images 
void drone_Thermo_control_point2_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    drone_Thermo_control_point2 = *msg;
    drone.control_Thermo_point2_x = drone_Thermo_control_point2.x;
    drone.control_Thermo_point2_y= drone_Thermo_control_point2.y;
    flagDroneThermoControlPoint2 = true;
     
}

//Second nearest point to the image center

void drone_Thermo_control_point1_callback2(const geometry_msgs::Point::ConstPtr& msg)
{
    drone_Thermo_control_point11 = *msg;
   
    drone.control_Thermo_point11_x = drone_Thermo_control_point1.x;
    drone.control_Thermo_point11_y= drone_Thermo_control_point1.y;
    flagDroneThermoControlPoint21 = true;
     
}

void drone_Thermo_control_point2_callback2(const geometry_msgs::Point::ConstPtr& msg)
{
    drone_Thermo_control_point22 = *msg;
   
    drone.control_Thermo_point22_x = drone_Thermo_control_point1.x;
    drone.control_Thermo_point22_y= drone_Thermo_control_point1.y;
    flagDroneThermoControlPoint22 = true;
     
}

// Desired Control POint 1 from RGB images ---> Usati per ottenere la retta esportata nel body vrame dall'image frame 
void drone_RGB_control_point1_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    drone_RGB_control_point1 = *msg;
    drone.control_RGB_point1_x = drone_RGB_control_point1.x;
    drone.control_RGB_point1_y= drone_RGB_control_point1.y;
    flagDroneRGBControlPoint1 = true;
     
}



//Desired COntrol Point 2 from thermal Images 
void drone_RGB_control_point2_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    drone_RGB_control_point2 = *msg;
    drone.control_RGB_point2_x = drone_RGB_control_point2.x;
    drone.control_RGB_point2_y= drone_RGB_control_point2.y;
    flagDroneRGBControlPoint2 = true;
     
}




double intlog(double base, double x)
{
    return (double)(log(x) / log(base));
}

void saturation(float velx, float vely, float th_x, float th_y)
{
    if (velx > th_x)
    {
        drone.drone_lin_vel_x_b = th_x;
    }
    else if (velx < -th_x)
    {
       drone.drone_lin_vel_x_b = -th_x; 
    }
    
    if (vely > th_y)
    {
        drone.drone_lin_vel_y_b = th_y; 
    }
    else if (vely < -th_y)
    {
        drone.drone_lin_vel_y_b = -th_y;
    }
    
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
     
    // drone.drone_lin_vel_x_b = drone.drone_lin_vel_x *cos(alfa) + drone.drone_lin_vel_y * sin(alfa);
     //drone.drone_lin_vel_y_b = -drone.drone_lin_vel_x*sin(alfa) + drone.drone_lin_vel_y* cos(alfa);
}

void Rotation_GF_to_BF_drone_vel(float velx, float vely, float alfa1)
{
    drone.drone_lin_vel_x_b = velx *cos(alfa1) + vely* sin(alfa1);
    drone.drone_lin_vel_y_b = -velx*sin(alfa1) + vely* cos(alfa1);
    saturation(drone.drone_lin_vel_x_b,drone.drone_lin_vel_y_b, 1.5, 1.5);
    
}

void Rotation_BF_to_GF_des_pos(float x_pos, float y_pos, float alfa)
{
    check_x_w = x_pos * cos(alfa) - y_pos * sin(alfa) + drone.drone_x;
    check_y_w = x_pos * sin(alfa) + y_pos * cos(alfa) + drone.drone_y;
}





//void transformation_waypoint_from_gps_frame_to_world(Waypoint_GPS waypoints,float theta)  //Per spiegazioni vedere note ipad
void transformation_waypoint_from_gps_frame_to_world(float theta) 
{ 
    int size_v = waypoints.waypoints_x_coo_gps_frame.size();
    int size_v1 = waypoints.waypoints_x_coo_world_frame.size();
   
    Eigen::Matrix3f T;
    Eigen::Vector3f coo_w(0.0,0.0,0.0);
    Eigen::Vector3f waypoints_vector_in_gps_frame(waypoints.waypoints_x_coo_gps_frame[size_v - 1],waypoints.waypoints_y_coo_gps_frame[size_v - 1], 1);
    
    cout<<"theta: "<< theta<< endl;
    
    //float waypoints_vector_in_gps_frame[3] = {waypoints.waypoints_x_coo_gps_frame[size_v - 1],waypoints.waypoints_y_coo_gps_frame[size_v - 1], 1}; //Definisco vettore waypoint in gps_frame da moltiplicare per matrice T
    /*
    for (int j = 0; j < 3; j++)
        {
            cout<<"waypoints_vector_in_gps_frame: "<< waypoints_vector_in_gps_frame[j] << endl;
            
        }
         */
    //float coo_w[3] = {0.0,0.0,0.0};
    
    
    //Transformation matrix from gps frame to world frame 
    //Initialize matrix  ---> considero la coordinata gps nel world frame del punto precedente (nella parte di traslazione) al fine di calcolare la sucessiva ---> guardare note ipad
    // gamma è un piccolo errore di rotazione, l'errore di tralsazione è aggiunto nel frame gps
   
    
    T << cos(theta + waypoints.gamma), -sin(theta + waypoints.gamma), waypoints.waypoints_x_coo_world_frame[0],
    sin(theta + waypoints.gamma), cos(theta + waypoints.gamma), waypoints.waypoints_y_coo_world_frame[0],
    0, 0,1;
    
    //float  T[3][3] = {{cos(theta + waypoints.gamma), -sin(theta + waypoints.gamma), waypoints.waypoints_x_coo_world_frame[0]}, {sin(theta + waypoints.gamma), cos(theta + waypoints.gamma), waypoints.waypoints_y_coo_world_frame[0]}, {0, 0,1}};
    /*
    int rows =  sizeof T / sizeof T[0]; // 3 rows  
    int cols = sizeof T[0] / sizeof(int); // 3 cols
    
    cout << "cols " << cols <<endl;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            cout<<"T: "<< T[i][j] << endl;
            
        }
    }
    */
   
    //Moltiplico la matrice di trasformazione con il vettore [Gps_x_coo, Gps_y_coo, 1]^(gps_frame). --> sono le coordinate che ottengo nel vettore waypoints.waypoints_x_coo_gps_frame
    //waypoints_x_coo_world_frame.
    /*
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            coo_w[i] += T[i][j] * waypoints_vector_in_gps_frame[j];
            
        }
        cout << "coo_w " << coo_w[i] <<endl;
    }
    
   */
    coo_w = T*waypoints_vector_in_gps_frame;    
 
    waypoints.waypoints_x_coo_world_frame.push_back(coo_w[0]);
    waypoints.waypoints_y_coo_world_frame.push_back(coo_w[1]);
    //cout << " waypoints.waypoints_x_coo_world_frame.size: " <<  waypoints.waypoints_x_coo_world_frame.size() <<endl;
    //cout << " waypoints.waypoints_y_coo_world_frame.size: " <<  waypoints.waypoints_y_coo_world_frame.size() <<endl;
}


//GPS waypoint functions 
//void create_GPS_coo_waypoint_map(Waypoint_GPS& waypoints, int length , int n_panels, int distance_between_panels, float theta)
void create_GPS_coo_waypoint_map( int length , int n_panels, int distance_between_panels, float theta)
{
    
    bool even = false;
    //Bisogna creare due punti per ciascun pannello, a distanza longitudinale length + delta (distaza pannello piu errore) 
    //e distanza tra i pannelli piu errore per passare da un punto all'altro.
    //Questi waypoint sono espressi nel GPS frame ---> verranno trasformati sucessivamente nel frame mondo in modo da ottenere coordinata x,y in world
    
                           
  /*           w4 <-------- -- w3 ^  
                                 |
                                 |
    //waipints: w1 ----------> w2| */
    
    bool left_to_right = true; //indica la direzione nel quale sto aggiungendo i punti nel frame:
                               //se left_to_rigth true mi muovo da sinistra verso destra lungo il pannelo.
                               //scorro il pannello fino al punto P
                               //Passo al punto P1 nel pannello sucessivo. 
                               //scorr il pannello da rigth a left quindi avro -(length + waypoints_eta)
                               
    for (int i = 1; i <  2*n_panels; i++)
    {
       if (even == false){ // even = 0 indica i punti nella parte destra  
          if (left_to_right == true)
          {
              //P1 ---> P2
              waypoints.waypoints_x_coo_gps_frame.push_back(waypoints.waypoints_x_coo_gps_frame[i - 1] + (length + waypoints.eta)); 
              waypoints.waypoints_y_coo_gps_frame.push_back(waypoints.waypoints_y_coo_gps_frame[i - 1]);
              left_to_right = false;
          }
          else
          {
              //P2 <---- P1
              waypoints.waypoints_x_coo_gps_frame.push_back(waypoints.waypoints_x_coo_gps_frame[i - 1] - (length + waypoints.eta)); 
              waypoints.waypoints_y_coo_gps_frame.push_back(waypoints.waypoints_y_coo_gps_frame[i - 1]);
              
          }
          //Trasformo il punto nel frame world, considerando l'inclinazione theta dei pannelli + un errore angolare gamma espresso in gradi.
          //In questo modo avro che il sistema di coordinate GPS risultera leggermente traslato rispetto i reali punti di start ed end dei pannelli
          
          //transformation_waypoint_from_gps_frame_to_world(waypoints, theta);
         transformation_waypoint_from_gps_frame_to_world(theta );  //ruoto i punti nel frame world 
          
          even = true;
       } 
       else 
       {
          
            //indica i punti nella parte sinistra della serpentina
          waypoints.waypoints_x_coo_gps_frame.push_back(waypoints.waypoints_x_coo_gps_frame[i - 1]); 
          waypoints.waypoints_y_coo_gps_frame.push_back(waypoints.waypoints_y_coo_gps_frame[i - 1] - (distance_between_panels + waypoints.delta ) ); //il meno dipende dal fatto che il primo apnnello ha una y maggiore del secondo e cosi via (nel world frame)
          //transformation_waypoint_from_gps_frame_to_world(waypoints, theta);
          transformation_waypoint_from_gps_frame_to_world(theta);
          even = false;
       }
       /*
       cout<< "waypoints.waypoints_x_coo_gps_frame[i]: " << waypoints.waypoints_x_coo_gps_frame[i] << endl;
       cout<< "waypoints.waypoints_y_coo_gps_frame[i]: " << waypoints.waypoints_y_coo_gps_frame[i] << endl;
       cout << " waypoints.waypoints_x_coo_world_frame.size: " <<  waypoints.waypoints_x_coo_world_frame.size() <<endl;
       cout << " waypoints.waypoints_y_coo_world_frame.size: " <<  waypoints.waypoints_y_coo_world_frame.size() <<endl;
       cout << " waypoints.x_coo_world: " <<  waypoints.x_coo_world <<endl;
        */ 
    }
    
}


//Utilizzo questa funzione per trovare amche coordinate GPS in world frame partendo da quelle reali dei punti P1
//P2
//Ovviamente le coordinate GPS faranno riferimento al frame collocato nel centro del pannello 1 
//Le coordinate GPS possono avere un errore ulteriore di rotazione e traslazione rispettivamente chiamati 
//
void transformation_panel_center_from_body_to_world(Structure *panel)
{
    //Transformation panel centers from body frame (panel body frame) to World frame
    //The angle theta of panel orientation is set in main 
    int size_v = panel -> xcenter_panel_frame.size();
    Eigen::Matrix3f T;
    Eigen::Vector3f coo_w(0.0,0.0,0.0);
    Eigen::Vector3f panel_center_vector_in_panel_frame(panel -> xcenter_panel_frame[size_v - 1],panel -> ycenter_panel_frame[size_v - 1], 1);
    
    
   // float panel_center_vector_in_panel_frame[3] = {panel -> xcenter_panel_frame[size_v - 1],panel -> ycenter_panel_frame[size_v - 1], 1}; //Definisco vettore waypoint in gps_frame da moltiplicare per matrice T
    T << cos(panel -> theta), -sin(panel -> theta), panel->xcenter_panel_world_frame[0],
    sin(panel -> theta), cos(panel -> theta), panel->ycenter_panel_world_frame[0],
    0, 0,1;
    
    //float  T[3][3] = {{cos(panel -> theta), -sin(panel -> theta), panel->xcenter_panel_world_frame[0]}, {sin(panel -> theta), cos(panel -> theta), panel->ycenter_panel_world_frame[0]}, {0, 0,1}};
    //int rows =  sizeof T / sizeof T[0]; // 3 rows  
    //int cols = sizeof T[0] / sizeof(int); // 3 cols
    
    //float coo_w[3] = {0.0,0.0,0.0};
    /*
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            coo_w[i] += T[i][j] * panel_center_vector_in_panel_frame[j];
            
        }
       
    }
    */
    coo_w = T*panel_center_vector_in_panel_frame;
    
    cout << "Panel center x in world frame:  " << coo_w[0] <<endl;
    cout << "Panel center y in world frame:  " << coo_w[1] <<endl;
    
    panel->xcenter_panel_world_frame.push_back(coo_w[0]);
    panel->ycenter_panel_world_frame.push_back(coo_w[1]);
    
}

//Place panel centers:
void  place_panel_centers(Structure *panel, int n_panels, float distance_between_panels)
{
    //Define centers in panel frame coordinates --> i want panel align algong the same x coo and different y coo
    for (float i = 0; i < n_panels; i++)
    {
        panel -> xcenter_panel_frame.push_back(0.0); 
        panel -> ycenter_panel_frame.push_back(- i * distance_between_panels); //Panels aligned along axe y in negative direction
       // cout<< "x center panel "<< i + 1 <<"frame : " << panel -> ycenter_panel_frame[i] << endl;
        //cout<< "y center panel "<< i + 1 <<"frame : " << panel -> ycenter_panel_frame[i] << endl;
        
        //Trasformation from panel frame to world frame.. The center of panel 1 in world frame is located in main 
        transformation_panel_center_from_body_to_world(panel);
    }
    
}


//Transformation from panel frame to world frame
void transformation_point_from_panel_to_world_frame(Structure *panel)
{
    //Transformation panel centers from body frame (panel body frame) to World frame
    //The angle theta of panel orientation is set in main 
    int size_v = panel -> xcoo_panel_frame.size();
    
    Eigen::Matrix3f T;
    Eigen::Vector3f coo_w(0.0,0.0,0.0);
    Eigen::Vector3f panel_P_xcoo_vector_in_panel_frame(panel -> xcoo_panel_frame[size_v - 1],panel -> ycoo_panel_frame[size_v - 1], 1);
    
    
    
    
    //cout<<"size Panel P x vec: "<<size_v<<endl; 
    //float panel_P_xcoo_vector_in_panel_frame[3] = {panel -> xcoo_panel_frame[size_v - 1],panel -> ycoo_panel_frame[size_v - 1], 1}; //Definisco vettore waypoint in gps_frame da moltiplicare per matrice T
    
    T << cos(panel -> theta), -sin(panel -> theta), panel->xcenter_panel_world_frame[0],
    sin(panel -> theta), cos(panel -> theta), panel->ycenter_panel_world_frame[0],
    0, 0,1;
    /*
    float  T[3][3] = {{cos(panel -> theta), -sin(panel -> theta), panel->xcenter_panel_world_frame[0]}, {sin(panel -> theta), cos(panel -> theta), panel->ycenter_panel_world_frame[0]}, {0, 0,1}};
    int rows =  sizeof T / sizeof T[0]; // 3 rows  
    int cols = sizeof T[0] / sizeof(int); // 3 cols
    
    float coo_w[3] = {0.0,0.0,0.0};
    
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            coo_w[i] += T[i][j] * panel_P_xcoo_vector_in_panel_frame[j];
            
        }
    }
    
   */
    coo_w = T *panel_P_xcoo_vector_in_panel_frame; 
 
    panel->xcoo_panel_world_frame.push_back(coo_w[0]);
    panel->ycoo_panel_world_frame.push_back(coo_w[1]);
    cout << "xcoo_panel_world_frame:  " << panel->xcoo_panel_world_frame[panel->xcoo_panel_world_frame.size() - 1] <<endl;
    cout << "ycoo_panel_world_frame:  " << panel->ycoo_panel_world_frame[panel->ycoo_panel_world_frame.size() - 1] <<endl;
}

//Trasformation GPS from GPS to world frame 
void transformation_GPS_point_from_GPS_to_world_frame(Structure *panel)
{
  //Transformation panel centers from body frame (panel body frame) to World frame
    //The angle theta of panel orientation is set in main 
    int size_v = waypoints.waypoints_x_coo_gps_frame.size();
    
    Eigen::Matrix3f T;
    Eigen::Matrix3f T1;
    Eigen::Vector3f coo_w(0.0,0.0,0.0);
    Eigen::Vector3f coo_w1(0.0,0.0,0.0);
    Eigen::Vector3f GPS_point_in_gps_frame(waypoints.waypoints_x_coo_gps_frame[size_v - 1],waypoints.waypoints_y_coo_gps_frame[size_v - 1], 1);
    
     T << cos(waypoints.gamma), -sin(waypoints.gamma), waypoints.delta,
    sin(waypoints.gamma), cos(waypoints.gamma), waypoints.eta,
    0, 0,1;
    
    /*
    cout<<"size Panel P x vec: "<<size_v<<endl;
    cout<<"waypoints.gamma: "<<waypoints.gamma<<endl;
    float GPS_point_in_gps_frame[3] = {waypoints.waypoints_x_coo_gps_frame[size_v - 1],waypoints.waypoints_y_coo_gps_frame[size_v - 1], 1}; //Definisco vettore waypoint in gps_frame da moltiplicare per matrice T

    //LA matrice di trasformazione fa riferimento a quella del panel frame. Vengono aggiunti delta, eta, gamma:
    //delta: errore di traslazione lungo asse x panel frame del gps frame 
    //eta: errore di traslazione lungo asse y panel frame del gps frame 
    //gamma: erorre di rotazione del gps frame rispetto al panel frame 
    //--> guardare ipad per maggiori inidicazioni
    
    //Prima trasformazione dal frame GPS al frame Panel (il frame GPS fa riferimento al frame dei pannelli in prima istanza)
    //La matrice di trasformazione viene definita come rotazione di angolo gamma e traslazione delta(asse x) e eta (asse y) rispetto frame pannelli
    float  T[3][3] = {{cos(waypoints.gamma), -sin(waypoints.gamma), waypoints.delta}, {sin(waypoints.gamma), cos(waypoints.gamma), waypoints.eta}, {0, 0,1}};
    int rows =  sizeof T / sizeof T[0]; // 3 rows  
    int cols = sizeof T[0] / sizeof(int); // 3 cols
    
    float coo_w[3] = {0.0,0.0,0.0};
    
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            coo_w[i] += T[i][j] * GPS_point_in_gps_frame[j];
            
        }
    }
    cout << "coo_w x from GPS to panel " << coo_w[0] <<endl;
    cout << "coo_w y from GPS to panel :  " << coo_w[1] <<endl;  
*/
   
    
    
     coo_w = T *  GPS_point_in_gps_frame;
    
    
    
    //    float  T[3][3] = {{cos(panel -> theta + waypoints.gamma), -sin(panel -> theta + waypoints.gamma), panel->xcenter_panel_world_frame[0] + waypoints.delta}, {sin(panel -> theta + waypoints.gamma), cos(panel -> theta + waypoints.gamma), panel->ycenter_panel_world_frame[0] + waypoints.eta}, {0, 0,1}};

    //Seconda trasformazione da frame Panel (dove ho riportato i punti del GPS) al frame world
     T1 << cos(panel -> theta), -sin(panel -> theta), panel->xcenter_panel_world_frame[0],
    sin(panel -> theta), cos(panel -> theta), panel->ycenter_panel_world_frame[0],
    0, 0,1;
    
    /*
    float  T1[3][3] = {{cos(panel -> theta), -sin(panel -> theta), panel->xcenter_panel_world_frame[0]}, {sin(panel -> theta), cos(panel -> theta), panel->ycenter_panel_world_frame[0]}, {0, 0,1}};

    float coo_w1[3] = {0.0,0.0,0.0};
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            coo_w1[i] += T1[i][j] * coo_w[j];
            
        }
    }
    */
    coo_w1 = T1* coo_w;
    //cout << coo_w1 << endl;

    waypoints.waypoints_x_coo_world_frame.push_back(coo_w1[0]);
    waypoints.waypoints_y_coo_world_frame.push_back(coo_w1[1]);
    cout << "waypoints_x_coo_world_frame:  " << waypoints.waypoints_x_coo_world_frame[waypoints.waypoints_x_coo_world_frame.size() - 1] <<endl;
    cout << "waypoints_y_coo_world_frame:  " << waypoints.waypoints_y_coo_world_frame[waypoints.waypoints_y_coo_world_frame.size() - 1] <<endl;  
}



//Place Point P1 and P2 
void place_panel_start_end_point(Structure *panel, int n_panels, float distance_between_panels, int configuration)
{
    bool even = false; //false indica i punti da sinistra veros destra , true da destra verso sinistra 
                       // i punti vengono aggiunti nel vettore come P1(destra) --> P2(sinistra)
                       // P2(sinistra) < --- P1(destra)
                       //ecc
    
    //Set flag for different  panel configuration 
    if (configuration == 1)
    {
    /*      -----------
     *      ------
     *      ------
     *      ----------
     */ 
        bool flag_panel_disposition_1 = true;  
    } 
    else
    {
        
    }               
        bool flag_panel_disposition_1 = false;   
   
                                                    
    
    //Place Panel point P1 and P2 before in Panel frame 
    for (int i = 0; i <  n_panels; i++)
    {
        if (even == false)
        {
            
            //Aggiungo punti in vettore da sinistra verso destra  P1 ---> P2
             panel->xcoo_panel_frame.push_back(panel->xcenter_panel_frame[i] - (panel->length/2));
             panel->ycoo_panel_frame.push_back(panel->ycenter_panel_frame[i]);
            
            //Aggiungo Punti GPS in corriposndenza dei punti P1-->P2 nel frame panel ---> IL frame GPS è collocato in corripondenza del frame panel 
            //quindi sul centro del pannello 1
             waypoints.waypoints_x_coo_gps_frame.push_back(panel->xcenter_panel_frame[i] - (panel->length/2));
             waypoints.waypoints_y_coo_gps_frame.push_back(panel->ycenter_panel_frame[i]);
             
             
            //Panel point trasformation to wolrd frame 
             transformation_point_from_panel_to_world_frame(panel);
             
            //GPS point trasformation to wolrd frame
             transformation_GPS_point_from_GPS_to_world_frame(panel); //--> trasfromarzione da GPS frame a panel frame poi da panel frame a world frame
             
             //P2
             panel->xcoo_panel_frame.push_back(panel->xcenter_panel_frame[i] + (panel->length/2));
             panel->ycoo_panel_frame.push_back(panel->ycenter_panel_frame[i]);
             
             //P2 in gps frame
             waypoints.waypoints_x_coo_gps_frame.push_back(panel->xcenter_panel_frame[i] + (panel->length/2));
             waypoints.waypoints_y_coo_gps_frame.push_back(panel->ycenter_panel_frame[i]);
             
             if (flag_panel_disposition_1 == true && i > 0)
             {
                 //Create New Configuration  
                 /* P1 --------------- P2
                  * P2 ------- P1
                  * P1 ------- P2 <--- siamo qui
                  * P2 ------------- P1
                  */
                  //Change position of P2: P1 --> P2
                 panel->xcoo_panel_frame[panel->xcoo_panel_frame.size() - 1] = panel->xcenter_panel_frame[i];
                 panel->ycoo_panel_frame[panel->ycoo_panel_frame.size() - 1] = panel->ycenter_panel_frame[i];
                 
                 waypoints.waypoints_x_coo_gps_frame[waypoints.waypoints_x_coo_gps_frame.size() -1] = panel->xcenter_panel_frame[i];
                 waypoints.waypoints_y_coo_gps_frame[waypoints.waypoints_y_coo_gps_frame.size() -1] = panel->ycenter_panel_frame[i];
                 flag_panel_disposition_1 = false;
             }
             
            even = true;
            
            cout <<"P1 x panel "<< i+1<<"in panel frame: "<< panel->xcoo_panel_frame[panel->xcoo_panel_frame.size() - 2]<<endl;
            cout <<"P1 y panel "<< i+1<<"in panel frame: "<< panel->ycoo_panel_frame[panel->xcoo_panel_frame.size() - 2]<<endl;
            cout <<"P2 x panel "<< i+1<<"in panel frame: "<< panel->xcoo_panel_frame[panel->xcoo_panel_frame.size() - 1]<<endl;
            cout <<"P2 y panel "<< i+1<<"in panel frame: "<< panel->ycoo_panel_frame[panel->xcoo_panel_frame.size() - 1]<<endl;
            
            //Transformation of P1 P2 from panel body frame to world frame
            transformation_point_from_panel_to_world_frame(panel);
            
            transformation_GPS_point_from_GPS_to_world_frame(panel);
        }
        else
        {
            //Aggiungo punti in vettore da destra  verso sinistra P2 <---- P1 
            //P1
            cout<<"panel->xcenter_panel_frame[i]: "<< panel->xcenter_panel_frame[i] << endl;
            cout<<"panel->ycenter_panel_frame[i]: "<< panel->ycenter_panel_frame[i] << endl;
            panel->xcoo_panel_frame.push_back(panel->xcenter_panel_frame[i] + (panel->length/2));
            panel->ycoo_panel_frame.push_back(panel->ycenter_panel_frame[i]);
            
             //P1 in gps frame
             waypoints.waypoints_x_coo_gps_frame.push_back(panel->xcenter_panel_frame[i] + (panel->length/2));
             waypoints.waypoints_y_coo_gps_frame.push_back(panel->ycenter_panel_frame[i]);
             
              if (flag_panel_disposition_1 == true)
             {
                 //Create New Configuration  
                 /* P1 --------------- P2
                  * P2 ------- P1 <--- siamo qui
                  * P1 ------- P2 
                  * P2 ------------- P1
                  */
                  //Change position of P1 --> coincide con la metà del pannello, dove è collocato il frame
                 panel->xcoo_panel_frame[panel->xcoo_panel_frame.size() - 1] = panel->xcenter_panel_frame[i];
                 panel->ycoo_panel_frame[panel->ycoo_panel_frame.size() - 1] = panel->ycenter_panel_frame[i];
                 
                 waypoints.waypoints_x_coo_gps_frame[waypoints.waypoints_x_coo_gps_frame.size() -1] = panel->xcenter_panel_frame[i];
                 waypoints.waypoints_y_coo_gps_frame[waypoints.waypoints_y_coo_gps_frame.size() -1] = panel->ycenter_panel_frame[i];
                 
             }
             
            transformation_point_from_panel_to_world_frame(panel);
            transformation_GPS_point_from_GPS_to_world_frame(panel);
            
            //P2
            panel->xcoo_panel_frame.push_back(panel->xcenter_panel_frame[i] - (panel->length/2));
            panel->ycoo_panel_frame.push_back(panel->ycenter_panel_frame[i]);
            //P2 in gps frame
            waypoints.waypoints_x_coo_gps_frame.push_back(panel->xcenter_panel_frame[i] - (panel->length/2));
            waypoints.waypoints_y_coo_gps_frame.push_back(panel->ycenter_panel_frame[i]);
            
            transformation_point_from_panel_to_world_frame(panel);
            transformation_GPS_point_from_GPS_to_world_frame(panel);
            
            even = false;
            
            
            cout <<"P1 x panel "<< i+1<<"in panel frame: "<< panel->xcoo_panel_frame[panel->xcoo_panel_frame.size() - 2]<<endl;
            cout <<"P1 y panel "<< i+1<<"in panel frame: "<< panel->ycoo_panel_frame[panel->xcoo_panel_frame.size() - 2]<<endl;
            cout <<"P2 x panel "<< i+1<<"in panel frame: "<< panel->xcoo_panel_frame[panel->xcoo_panel_frame.size() - 1]<<endl;
            cout <<"P2 y panel "<< i+1<<"in panel frame: "<< panel->ycoo_panel_frame[panel->xcoo_panel_frame.size() - 1]<<endl;
            
        }
        
        
    }
}


float evaluate_relative_angle(int count)
{
   float d1 = 0.0;
   float d2 = 0.0;
   float d3 = 0.0;
   float x0 = waypoints.waypoints_x_coo_world_frame[count - 1];
   float x1 = waypoints.waypoints_x_coo_world_frame[count];
   float x2 = waypoints.waypoints_x_coo_world_frame[count + 1];
   float y0 = waypoints.waypoints_y_coo_world_frame[count - 1];
   float y1 = waypoints.waypoints_y_coo_world_frame[count];
   float y2 = waypoints.waypoints_y_coo_world_frame[count + 1];
   
   //Evaluate distances between GPS point
   d1 = sqrt(pow(x1 - x0,2) + pow(y1 - y0,2)); //da P1 a P2 pannelo 1
   d2 = sqrt(pow(x2 - x1,2) + pow(y2 - y1,2)); //da P2 a P1 (pan1 --> pan2)
   d3 = sqrt(pow(x2 - x0,2) + pow(y2 - y0,2)); // da P1 a P1 (pan1 --> pan2)
   
   //Find relative angle with the rule of cosine 
   float cos_alfa = (((d1*d1) + (d2*d2) - (d3*d3))/(2*d2*d1));
   float alfa = acos(cos_alfa);
   
   return alfa;
}


//****************  ESTIMATE NEXT PANEL POSITION GIVEN THE GPS WAYPOINTS **********


void find_coo_start_point_next_panel_drone_body_frame(float yaw_des, int count, float alfa)
{
   //Evaluate point in drone body frame relative to the start position to reach on the other line of panels
   //GPS Points
  float GPS_P2_x = waypoints.waypoints_x_coo_world_frame[count]; //P2 panel 1
  float GPS_P2_y = waypoints.waypoints_y_coo_world_frame[count]; //P2 panel 1
  float GPS_P1_x = waypoints.waypoints_x_coo_world_frame[count + 1]; //P1 panel 2
  float GPS_P1_y = waypoints.waypoints_y_coo_world_frame[count + 1]; //P1 panel 2
  float d = 0.0; //distance between GPS_P2 and GPS_P2 between two panels
  
  float sigma = 0.0; //angolo desiderato nello yaw del drone
  float x_des_b = 0.0;
  float y_des_b = 0.0;
  
  bool even = false;
  if ( count % 2 == 0) 
  {
    even = true;  
  }
  else 
  {
    even = false;
  }
  
  d = sqrt(pow(GPS_P1_x - GPS_P2_x, 2) + pow(GPS_P1_y - GPS_P2_y, 2)); //Evaluate distance between GPS point at the end of the previous panel and 
                                                                        //at the start of the next panel
  if (even == false) //even --> left , odd --> rigth
  {                                                                    
    //Yaw_des indica l'orientazione quando il drone è allineato con la prima vela.
     //Quando il drone arriva alla fine del pannello, esegue una rotazione di 180 gradi, quindi 
     //lo yaw nella vela sucessiva sara yaw_des + 180.
     
    /*NB: QUESTI CALCOLI VENGONO FATTI CONSIDERANDO CHE IL DRONE DEVE ANCORA RUOTARE DI 180 AL TERMINE
     * DELLA VELA. iL PUNTO DI START DEL OPANNELLO SUCESSIVO VIENE GIA CALCOLATO NEL FRAME DEL DRONE 
     * CONSIDERANDOLO GIA RUOTATO DI 180 GRADI, NEL CASO DESTRO */
     
    //Ruoto il punto di start del pannello sucessivo nel body frame del drone considerando lo yaw_des + 180 nel caso sinistro
       Rotation_GF_to_BF_des_pos(GPS_P1_x , GPS_P1_y,  yaw_des + M_PI);
                 
       Rotation_GF_to_BF_drone_pos(yaw_des + M_PI); 
  }
  else
  {
       Rotation_GF_to_BF_des_pos(GPS_P1_x , GPS_P1_y,  yaw_des);
                 
       Rotation_GF_to_BF_drone_pos(yaw_des); 
  }
    
    if (check_y_b > 0)
    {
        //Allora mi muovo verso il basso, cioe sulle y positive del drone per raggiungere il pannello sucessivo
        //L'angolo theta considerato rispetto al frame del drone gia ruotato di 180 gradi è: 
         sigma = yaw_des + M_PI + alfa;
         
         x_des_b = d*cos(alfa); //--Perche è come dire 180 + yaw_des (che diventa nuovo riferimento per frame ruotato) + alfa
                                // equivalete ad avere solo alfa col frame ruotato (vedi ipad per dettagli)
         y_des_b = d*sin(alfa);
         
         //Rotate desired position in world frame 
         Rotation_BF_to_GF_des_pos(x_des_b, y_des_b, yaw_des + M_PI);
         
         //Calcolo dx e dy nel frame del drone realtivo al punto indicato dalla distanza d e angolo sigma
         drone.next_point_P1_x.push_back(check_x_w);
         drone.next_point_P1_y.push_back(check_y_w);
         
    }
    else 
    {
        //Filare sucessivo è sopra
        sigma = yaw_des + M_PI + alfa;
        x_des_b = d*cos(-alfa); 
        y_des_b = d*sin(-alfa);
        //Rotate desired position in world frame 
         Rotation_BF_to_GF_des_pos(x_des_b, y_des_b, yaw_des + M_PI);
         
         //Calcolo dx e dy nel frame del drone realtivo al punto indicato dalla distanza d e angolo sigma
         drone.next_point_P1_x.push_back(check_x_w);
         drone.next_point_P1_y.push_back(check_y_w);
        
   
        
    }
    
    //Save last Important Drone Position
    drone.last_x = drone.drone_x;
    drone.last_y = drone.drone_y;
                                                
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//Evaluate control point on line r passing throug P1 and P2--- Taking Information from GPS POINTS 
void evaluate_control_point(float x_target_P1, float y_target_P1, float x_target_P2, float y_target_P2, int target_point)
{
    float x_target_P1_body = 0.0;
    float y_target_P1_body = 0.0;
    float x_target_P2_body = 0.0;
    float y_target_P2_body = 0.0;
    
    //Equation line parameters Passing trhiug P1 and P2
    float a = 0.0; //equivale a m coefficente angolare
    float c = 0.0;
    float b = -1;
   //Rotate Point P1 in drone body frame 
    Rotation_GF_to_BF_des_pos(x_target_P1 , y_target_P1,  drone.drone_Yaw);
    x_target_P1_body =  check_x_b;
    y_target_P1_body = check_y_b;
    
    //Rotate Point P2 in drone body frame 
    Rotation_GF_to_BF_des_pos(x_target_P2 , y_target_P2,  drone.drone_Yaw);
    x_target_P2_body =  check_x_b;
    y_target_P2_body = check_y_b; 
    
    /*
    cout<< "[GPS CONTROL FUNCTION]: x_target_P1_body: "<< x_target_P1_body <<endl;
     cout<< "[GPS CONTROL FUNCTION]: y_target_P1_body: "<< y_target_P1_body <<endl;
     cout<< "[GPS CONTROL FUNCTION]: x_target_P2_body: "<< x_target_P2_body <<endl;
     cout<< "[GPS CONTROL FUNCTION]: y_target_P2_body: "<< y_target_P2_body <<endl;
     */
   //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0
     a =  ((y_target_P2_body  -  y_target_P1_body)/(x_target_P2_body - x_target_P1_body));
     c = ((-(a) * x_target_P1_body) + y_target_P1_body);
     /*
     a = y_target_P2_body - y_target_P1_body;
     b = -1*(x_target_P2_body  - x_target_P1_body);
     c = ((-(y_target_P2_body - y_target_P1_body)* x_target_P1_body) + y_target_P1_body*( x_target_P2_body - x_target_P1_body));
     */
     /*
     cout<< "a: "<< a <<endl;
     cout<< "b: "<< b <<endl;
     cout<< "c: "<< c <<endl;
     */
     
     
     //Find distance point line: point is the body Origin and line is the line r
     double e = (abs(c)/(sqrt(pow(a,2) + pow(b,2))));
     error_from_GPS_line = e;
     //cout<< "[GPS CONTROL FUNCTION]: distace e: "<< e<<endl;
     
     //Define vector Vx starting from body frame origin and parallel to the line r
     double Vx[2] = {1/a, -1/b};
     
     //cout<< "Vx[0]: "<< Vx[0]<<endl;
     //cout<< "Vx[1]: "<< Vx[1]<<endl;
     
     
     double Kx = 0.3; //Coefficiente moltiplicativo del vettore parallelo alla retta r
     double Vx_norm[2] = {Kx * ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[0]), Kx* ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[1])};
     //Il vettore Vx matematicamente viene sempre calcolato con stessa direzione della retta parallela a r ma con verso fisso.
     //in questo caso la direzione permette di muoversi correttamante ma ilv erso fa si che il drone possa muoversi con velocita opposte
     // a quelle necessarie per raggiungere il target 
     //in questa maniera è fondamentale direzionare il verso in modo da avere velocita che permettano di raggiungere sempre il target indipendentemente dallo yaw
     
     //verifico segno del target finale nel drone body frame: Se positivo il target risulta con x positiva nel drone bnody frame allora i segni di
     // Vxx  e Vxy devono essere cambiati:
     //cout<< "------ Vx_norm x : "<< Vx_norm[0] <<endl;
     //cout<< "------ Vx_norm y: "<< Vx_norm[1] <<endl;
     
     //cout<<"target_point: "<< target_point<<endl;
     float rot_Vx = 0.0;
     float rot_Vy = 0.0;
     if (target_point == 1)
     {
         //punto di target è P1:
         if (x_target_P1_body >= 0 and  Vx_norm[0] < 0)
         {
     
            rot_Vx =  Vx_norm[0] * cos(M_PI) - Vx_norm[1] * sin(M_PI);
            rot_Vy = Vx_norm[0] * sin(M_PI) + Vx_norm[1] * cos(M_PI);
            Vx_norm[0] = rot_Vx;
            Vx_norm[1] = rot_Vy;
            //cout<< "sono qui"<< endl;
         }
         
         else{
             Vx_norm[0] = Vx_norm[0];
             Vx_norm[1] = Vx_norm[1];
            
         }
   
      
     }    
     else
     {
         if (x_target_P2_body >= 0 and  Vx_norm[0] < 0)
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
    // cout<< "Vy_norm x: "<< Vy_norm[0] <<endl;
     //cout<< "Vy_norm y: "<< Vy_norm[1]  <<endl;
     /*
     if (c > 0)
     {
         double Vy[2] = {1/b, 1/a};
         Vy_norm[0] = Ky * e *((1/(sqrt(pow(Vy[0],2) + pow(Vy[1],2)))) * Vy[0]); 
         Vy_norm[1] = Ky * e *((1/(sqrt(pow(Vy[0],2) + pow(Vy[1],2)))) * Vy[1]);
         cout<< "Vy_norm x: "<< Vy_norm[0] <<endl;
         cout<< "Vy_norm y: "<< Vy_norm[1]  <<endl;
     }
     else 
     {
         double Vy[2] = {-1/b, -1/a};  //Per puntare con verso verso la retta 
         Vy_norm[0] = Ky * e *((1/(sqrt(pow(Vy[0],2) + pow(Vy[1],2)))) * Vy[0]); 
         Vy_norm[1] = Ky * e *((1/(sqrt(pow(Vy[0],2) + pow(Vy[1],2)))) * Vy[1]);
         cout<< "Vy_norm x: "<< Vy_norm[0] <<endl;
         cout<< "Vy_norm y: "<< Vy_norm[1]  <<endl;
     }
     */
     //cout<< "Vx_norm x: "<< Vx_norm[0] <<endl;
     //cout<< "Vx_norm y: "<< Vx_norm[1] <<endl;
     
     
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
     //float rot_Dx = 0;
     //float rot_Dy = 0;
     
     /*
     if  (D[0] < 0)
     {
         rot_Dx = D[0]* cos(M_PI/2) - D[1]* sin(M_PI/2);
         rot_Dy = D[0]* sin(M_PI/2) + D[1]* cos(M_PI/2);
         drone.control_x_coo = rot_Dx;
         drone.control_y_coo = rot_Dy;
         cout<< "D x: "<< rot_Dx <<endl;
         cout<< "D y: "<< rot_Dy <<endl;
     }
     else
     {
        drone.control_x_coo = D[0];
        drone.control_y_coo = D[1]; 
        cout<< "D x: "<< D[0] <<endl;
        cout<< "D y: "<< D[1] <<endl;
     }
     */
    
     drone.control_x_coo = D[0];
     drone.control_y_coo = D[1]; 
     //cout<< "[GPS CONTROL FUNCTION]: D x: "<< D[0] <<endl;
     //cout<< "[GPS CONTROL FUNCTION]: D y: "<< D[1] <<endl;
     
     
}


/******* Control on the line obtained by the Thermal image frame and exported in body frame ***********/


void evaluate_control_point_from_Thermal_image_points(int target_point)
{
    float x_target_P1_body = 0.0;
    float y_target_P1_body = 0.0;
    float x_target_P2_body = 0.0;
    float y_target_P2_body = 0.0;
    if (KF.switch_point == 1)
    {
        //Sceglie la seconda linea detectata se l'errore supera la deviazione standard
     x_target_P1_body = drone.control_Thermo_point1_x;
     y_target_P1_body = drone.control_Thermo_point1_y;
     x_target_P2_body = drone.control_Thermo_point2_x;
     y_target_P2_body = drone.control_Thermo_point2_y;
        
    }
    else
    {
     x_target_P1_body = drone.control_Thermo_point11_x;
     y_target_P1_body = drone.control_Thermo_point11_y;
     x_target_P2_body = drone.control_Thermo_point22_x;
     y_target_P2_body = drone.control_Thermo_point22_y;
        
        
    }
   
    
    //Equation line parameters Passing trhiug P1 and P2
    float a = 0.0; //equivale a m coefficente angolare
    float c = 0.0;
    float b = -1;
   
  
     //cout<<"a image: "<<a<<endl;
     //cout<<"c image: "<<c<<endl;
     if (KF.K_count > 100 or KF_RGB.K_count > 100) //and  KF.c_b  != 0)
     {
        cout<<"[THERMO CONTROL FUNCTION]: --FOllow KF estimation: "<< KF.a_b << ", "<< KF.c_b << "\n";
         a = KF.a_b;
         c = KF.c_b;
         
         
         
     }
     else 
     {
         //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0
          a =  ((y_target_P2_body  -  y_target_P1_body)/(x_target_P2_body - x_target_P1_body));
          c = ((-(a) * x_target_P1_body) + y_target_P1_body); 
     }
     
     
     
     if (abs(a) > 1)
     {
         return;
     }
 
     
     /*
     a = y_target_P2_body - y_target_P1_body;
     b = -1*(x_target_P2_body  - x_target_P1_body);
     c = ((-(y_target_P2_body - y_target_P1_body)* x_target_P1_body) + y_target_P1_body*( x_target_P2_body - x_target_P1_body));
     */
     /*
     cout<< "[THERMO CONTROL FUNCTION]: x_target_P1_body: "<< x_target_P1_body <<endl;
     cout<< "[THERMO CONTROL FUNCTION]: y_target_P1_body: "<< y_target_P1_body <<endl;
      */
     //cout<< "x_target_P2_body: "<< x_target_P2_body <<endl;
    // cout<< "y_target_P2_body: "<< y_target_P2_body <<endl;
     
     /*
     cout<< "a: "<< a <<endl;
     cout<< "b: "<< b <<endl;
     cout<< "c: "<< c <<endl;
     */
     
     
     //Find distance point line: point is the body Origin and line is the line r
     double e = (abs(c)/(sqrt(pow(a,2) + pow(b,2))));
     //Mettere una variabile qui da salvare 
     y_error_with_vision_line = e;
     
     //cout<< "distace e: "<< e<<endl;
     ROS_INFO("[THERMO CONTROL FUNCTION]: --ERROR DISTANCE: %f--", e);
     
     //Define vector Vx starting from body frame origin and parallel to the line r
     double Vx[2] = {1/a, -1/b};
     
     //cout<< "Vx[0]: "<< Vx[0]<<endl;
     //cout<< "Vx[1]: "<< Vx[1]<<endl;
     
     
     double Kx = 0.3; //Coefficiente moltiplicativo del vettore parallelo alla retta r
     double Vx_norm[2] = {Kx * ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[0]), Kx* ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[1])};
     //Il vettore Vx matematicamente viene sempre calcolato con stessa direzione della retta parallela a r ma con verso fisso.
     //in questo caso la direzione permette di muoversi correttamante ma ilv erso fa si che il drone possa muoversi con velocita opposte
     // a quelle necessarie per raggiungere il target 
     //in questa maniera è fondamentale direzionare il verso in modo da avere velocita che permettano di raggiungere sempre il target indipendentemente dallo yaw
     
     //verifico segno del target finale nel drone body frame: Se positivo il target risulta con x positiva nel drone bnody frame allora i segni di
     // Vxx  e Vxy devono essere cambiati:
     //cout<< "------ Vx_norm x : "<< Vx_norm[0] <<endl;
    // cout<< "------ Vx_norm y: "<< Vx_norm[1] <<endl;
    

    // cout<<"target_point: "<< target_point<<endl;
     float rot_Vx = 0.0;
     float rot_Vy = 0.0;
     if (target_point == 1)
     {
         //punto di target è P1:
         if (x_target_P1_body >= 0 and  Vx_norm[0] < 0)
         {
     
            rot_Vx =  Vx_norm[0] * cos(M_PI) - Vx_norm[1] * sin(M_PI);
            rot_Vy = Vx_norm[0] * sin(M_PI) + Vx_norm[1] * cos(M_PI);
            Vx_norm[0] = rot_Vx;
            Vx_norm[1] = rot_Vy;
            //cout<< "sono qui"<< endl;
         }
         
         else{
             Vx_norm[0] = Vx_norm[0];
             Vx_norm[1] = Vx_norm[1];
            
         }
   
      
     }    
     else
     {
         if (x_target_P2_body >= 0 and  Vx_norm[0] < 0)
         {
            rot_Vx =  Vx_norm[0] * cos(M_PI) - Vx_norm[1] * sin(M_PI);
            rot_Vy = Vx_norm[0] * sin(M_PI) + Vx_norm[1] * cos(M_PI);
            Vx_norm[0] = rot_Vx;
            Vx_norm[1] = rot_Vy;
              //cout<< "sono quiiiiii"<< endl;
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
     Vy_norm[0] =Ky * e *((1/(sqrt(pow(Vy[0],2) + pow(Vy[1],2)))) * Vy[0]); 
     Vy_norm[1] = Ky * e *((1/(sqrt(pow(Vy[0],2) + pow(Vy[1],2)))) * Vy[1]);
     if (c > 0 && Vy_norm[1] < 0)
     {
         Vy_norm[1] = -1 * Vy_norm[1];
     }
     else if (c < 0 && Vy_norm[1] > 0)
     {
         Vy_norm[1] = -1 * Vy_norm[1];
     }
    
     //cout<< "Vx_image_norm x: "<< Vx_norm[0] <<endl;
     //cout<< "Vx_image_norm y: "<< Vx_norm[1] <<endl;
     
     
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
     
    
     drone.image_control_x_coo = abs(D[0]);
     drone.image_control_y_coo = D[1];
     
     //Se le coordinate sono NAN alla prima iterazione le metto 0.0;
     if (drone.image_control_x_coo != drone.image_control_x_coo and drone.image_control_y_coo != drone.image_control_y_coo)
     {
         drone.image_control_x_coo = 0.0;
         drone.image_control_y_coo = 0.0;
     }
     
     //Se le coordinate sono le stesse da un po di tempo, quindi il drone continua ad avanzare seguendo la retta precedenete 
     //Ma non avendo piu nuovi riferimenti, ridefinisco le ccordinate in 0.0, in questo 
     //modo lo switch sottotante ritorna a fare riferimento al waypoint fissato col GPS
     if (drone.image_control_x_coo == drone.image_control_x_coo_old and drone.image_control_y_coo == drone.image_control_y_coo_old )
     {
         drone.switch_counter = drone.switch_counter + 1;
         if (drone.switch_counter > 250)
         {
            drone.image_control_x_coo = 0.0;
            drone.image_control_y_coo = 0.0;
            drone.switch_counter  = 0;
         }
         ROS_INFO("Panel Not Recognized from %d iterations: ", drone.switch_counter);
     }
     else
     {
         init_panel = false;
     }
     
     
     cout<< "[THERMO CONTROL FUNCTION]: D x image: "<< D[0] <<endl;
     cout<< "[THERMO CONTROL FUNCTION]: D y image: "<< D[1] <<endl;
     
     
     //Save Old Coordinates 
     drone.image_control_x_coo_old = drone.image_control_x_coo;
     drone.image_control_y_coo_old = drone.image_control_y_coo;
     
}



/******* Control on the line obtained by the RGB image frame and exported in body frame ***********/


void evaluate_control_point_from_RGB_image_points(int target_point)
{
    float x_target_P1_body = drone.control_RGB_point1_x;
    float y_target_P1_body = drone.control_RGB_point1_y;
    float x_target_P2_body = drone.control_RGB_point2_x;
    float y_target_P2_body = drone.control_RGB_point2_y;
    
    //Equation line parameters Passing trhiug P1 and P2
    float a = 0.0; //equivale a m coefficente angolare
    float c = 0.0;
    float b = -1;
   
   //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0
     a =  ((y_target_P2_body  -  y_target_P1_body)/(x_target_P2_body - x_target_P1_body));
     c = ((-(a) * x_target_P1_body) + y_target_P1_body);
     
     
     /*
     a = y_target_P2_body - y_target_P1_body;
     b = -1*(x_target_P2_body  - x_target_P1_body);
     c = ((-(y_target_P2_body - y_target_P1_body)* x_target_P1_body) + y_target_P1_body*( x_target_P2_body - x_target_P1_body));
     */
     
     /*
     cout<< "x_target_P1_body: "<< x_target_P1_body <<endl;
     cout<< "y_target_P1_body: "<< y_target_P1_body <<endl;
     cout<< "x_target_P2_body: "<< x_target_P2_body <<endl;
     cout<< "y_target_P2_body: "<< y_target_P2_body <<endl;
     */
     
     /*
     cout<< "a: "<< a <<endl;
     cout<< "b: "<< b <<endl;
     cout<< "c: "<< c <<endl;
     */
     
     
     //Find distance point line: point is the body Origin and line is the line r
     double e = (abs(c)/(sqrt(pow(a,2) + pow(b,2))));
     //cout<< "distace e: "<< e<<endl;
     ROS_INFO("[RGB FUNCTION]--ERROR DISTANCE: %f--", e);
     
     //Define vector Vx starting from body frame origin and parallel to the line r
     double Vx[2] = {1/a, -1/b};
     
     //cout<< "Vx[0]: "<< Vx[0]<<endl;
     //cout<< "Vx[1]: "<< Vx[1]<<endl;
     
     
     double Kx = 0.3; //Coefficiente moltiplicativo del vettore parallelo alla retta r
     double Vx_norm[2] = {Kx * ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[0]), Kx* ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[1])};
     //Il vettore Vx matematicamente viene sempre calcolato con stessa direzione della retta parallela a r ma con verso fisso.
     //in questo caso la direzione permette di muoversi correttamante ma ilv erso fa si che il drone possa muoversi con velocita opposte
     // a quelle necessarie per raggiungere il target 
     //in questa maniera è fondamentale direzionare il verso in modo da avere velocita che permettano di raggiungere sempre il target indipendentemente dallo yaw
     
     //verifico segno del target finale nel drone body frame: Se positivo il target risulta con x positiva nel drone bnody frame allora i segni di
     // Vxx  e Vxy devono essere cambiati:
     //cout<< "------ Vx_norm x : "<< Vx_norm[0] <<endl;
    // cout<< "------ Vx_norm y: "<< Vx_norm[1] <<endl;
     
     
     float rot_Vx = 0.0;
     float rot_Vy = 0.0;
     if (target_point == 1)
     {
         //punto di target è P1:
         if (x_target_P1_body >= 0 and  Vx_norm[0] < 0)
         {
     
            rot_Vx =  Vx_norm[0] * cos(M_PI) - Vx_norm[1] * sin(M_PI);
            rot_Vy = Vx_norm[0] * sin(M_PI) + Vx_norm[1] * cos(M_PI);
            Vx_norm[0] = rot_Vx;
            Vx_norm[1] = rot_Vy;
            //cout<< "sono qui"<< endl;
         }
         
         else{
             Vx_norm[0] = Vx_norm[0];
             Vx_norm[1] = Vx_norm[1];
            
         }
   
      
     }    
     else
     {
         if (x_target_P2_body >= 0 and  Vx_norm[0] < 0)
         {
            rot_Vx =  Vx_norm[0] * cos(M_PI) - Vx_norm[1] * sin(M_PI);
            rot_Vy = Vx_norm[0] * sin(M_PI) + Vx_norm[1] * cos(M_PI);
            Vx_norm[0] = rot_Vx;
            Vx_norm[1] = rot_Vy;
              //cout<< "sono quiiiiii"<< endl;
         }
         
         else
         {
            Vx_norm[0] = Vx_norm[0];
            Vx_norm[1] = Vx_norm[1];
           
         }
          
     }
      
  //Define vector Vy starting from body frame origin and perpendicular to line r
    
 
     //Il vettore Vy è moltiplicato pe run guadagno Ky e anche per l'errore dato dalla distanza punto retta e, la quale deve tendere a zero
     float Ky = 0.1; //Coefficiente moltiplicativo del vettore perp alla retta r
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
    
     //cout<< "Vx_image_norm x: "<< Vx_norm[0] <<endl;
     //cout<< "Vx_image_norm y: "<< Vx_norm[1] <<endl;
     
     
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
     
    
     drone.image_RGB_control_x_coo = D[0];
     drone.image_RGB_control_y_coo = D[1];
     
     //Se le coordinate sono NAN alla prima iterazione le metto 0.0;
     if (drone.image_RGB_control_x_coo != drone.image_RGB_control_x_coo and drone.image_RGB_control_y_coo != drone.image_RGB_control_y_coo)
     {
         drone.image_RGB_control_x_coo = 0.0;
         drone.image_RGB_control_y_coo = 0.0;
     }
     
     //Se le coordinate sono le stesse da un po di tempo, quindi il drone continua ad avanzare seguendo la retta precedenete 
     //Ma non avendo piu nuovi riferimenti, ridefinisco le ccordinate in 0.0, in questo 
     //modo lo switch sottotante ritorna a fare riferimento al waypoint fissato col GPS
     if (drone.image_RGB_control_x_coo == drone.image_RGB_control_x_coo_old and drone.image_RGB_control_y_coo == drone.image_RGB_control_y_coo_old )
     {
         drone.switch_RGB_counter = drone.switch_RGB_counter + 1;
         if (drone.switch_RGB_counter > 300)
         {
            drone.image_RGB_control_x_coo = 0.0;
            drone.image_RGB_control_y_coo = 0.0;
            drone.switch_RGB_counter  = 0;
         }
         ROS_INFO("Panel Not Recognized from %d iterations: ", drone.switch_counter);
     }
     else
     {
         init_panel = false;
     }
     
     
     cout<< "[RGB FUNCTION] D x image: "<< drone.image_RGB_control_x_coo <<endl;
     cout<< "[RGB FUNCTION] D y image: "<<  drone.image_RGB_control_y_coo<<endl;
     
     
     //Save Old Coordinates 
     drone.image_RGB_control_x_coo_old = drone.image_RGB_control_x_coo;
     drone.image_RGB_control_y_coo_old = drone.image_RGB_control_y_coo;
     
}

// **************************** IMplementata la sequenza di KF1 e KF2 che permettono di unire entrambe le osservazioni 
//" kalnman filter sono impolmentati in maniera sequenziale, cioe a cascata. Il primo prende osservazioni dalla telecamera RGB e stima uno stato.
//ILo stato stimato e la matrice P stimata sono inviati in cascata al secondo KF il qulae riceve osservazioni dalla telecamera termica. Lo stato stimato dal 
//filtro relaticvo la camera RGB diventa lo stato di riferimento del secondo filtro.
// Nel caso all'iterazione considerata  non ci fossero osservazioni si passsa sucessivamente al filtro sucessivo altimrngti si esce senza stimare nulla.
//********************************* In futuro sarà necessario creare uno script a parte...


//Definizione variabili globali per evitare di cambiarli


//verificare perche non viene aggiornato lo stato del filtro RGB quando viene cambiato pannello
void Kalman_Filter_RGB(float x_target_P1, float y_target_P1, float x_target_P2, float y_target_P2, int target_point)
{
    //Primo Kalman FIlter nella sequenza 
     float x1_obs = 0.0;
     float y1_obs = 0.0;
     float x2_obs = 0.0;
     float y2_obs = 0.0;
     
     /*
     //Se non sono presenti osservazioni il filtro RGB non viene aggiornato
     if (flagDroneRGBControlPoint1 == false or flagDroneRGBControlPoint2 == false)
     {
         return;
     }
     */
     
   //Ruotare Punti RGB nel GF 

      Rotation_BF_to_GF_des_pos(drone.control_RGB_point1_x, drone.control_RGB_point1_y, drone.drone_Yaw); //---RIsultato finisce in check_x_w e check_y_w variabili globali
      x1_obs = check_x_w;
      y1_obs = check_y_w;
      //cout<< "##########  point1_x_body : "<< drone.control_RGB_point1_x << " x1_obs_w: "<< x1_obs << endl;
      //cout<< "##########  point1_y_body : "<< drone.control_RGB_point1_y << " y1_obs_w: "<< y1_obs << endl;
      
      Rotation_BF_to_GF_des_pos(drone.control_RGB_point2_x, drone.control_RGB_point2_y, drone.drone_Yaw); 
      x2_obs = check_x_w;
      y2_obs = check_y_w;
      //cout<< "##########  point2_x_body : "<< drone.control_RGB_point2_x << " x2_obs_w: "<< x2_obs << endl;
      //cout<< "##########  point2_y_body : "<< drone.control_RGB_point2_y << " y2_obs_w: "<< y2_obs << endl;
      
      
      //Evaluate a,b,c equation parameters given the the target points in GFs
      float a_target = (y_target_P2 - y_target_P1)/(x_target_P2 - x_target_P1);
      float b_target = 1;
      float c_target = ((-1*a_target * x_target_P1) + y_target_P1);
      
      //Evaluate a,b,c equation parameters given the the obs points in GF
      float a_obs = (y2_obs - y1_obs)/(x2_obs - x1_obs);
      float b_obs = 1;
      float c_obs = ((-1*a_obs * x1_obs) + y1_obs);
      
      cout<< "[KALMAN FILTER RGB] a_target : "<< a_target << " c_target: "<< c_target << endl;
      
      cout<< "[KALMAN FILTER RGB] a_obs : "<< a_obs <<"    c_obs: "<< c_obs << endl;
      
      
      //Define sitution when the KF is not updated
      if (isnan(a_obs) == 1 or isnan(c_obs) == 1)
      {
          return;
      }
      
       //Check per riportare il xh al valore reale una volta che diverge 
      
      
     if (KF_RGB.K_count < 10)
     {
      
           if (c_obs < KF_RGB.xh_[1] - 3.5 or c_obs > KF_RGB.xh_[1] + 3.5)
           {
               return;
           }
      }
      else
      {
          if (a_obs < KF_RGB.xh_[0] - 0.2 or a_obs >  KF_RGB.xh_[0] + 0.2)
          {
             //Eliminate very wrong predictions
             return;
           }
           
            if (c_obs < KF_RGB.xh_[1] - 2 or c_obs > KF_RGB.xh_[1] + 2)
           {
               cout<< "SONO QUIIIIIIIIIIIIIIIIIIIIIIIIIII" <<endl;
               cout<< "KF_RGB.xh_[1]: " <<KF_RGB.xh_[1]<<endl;
               return;
           }
      }
      
       //Evaluate mean and covariance of c_obs
      KF_RGB.c_obs_v.push_back(c_obs);
      float sum = accumulate(KF_RGB.c_obs_v.begin(), KF_RGB.c_obs_v.end(), 0.0); //SOmma i valori nel vettore partendo da zero
      float mean = sum / KF_RGB.c_obs_v.size();
      vector<double> diff(KF_RGB.c_obs_v.size());
      transform(KF_RGB.c_obs_v.begin(), KF_RGB.c_obs_v.end(), diff.begin(),
               bind2nd(std::minus<double>(), mean));
      double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
      KF_RGB.stdev = sqrt(sq_sum / KF_RGB.c_obs_v.size());
       
      cout<< "[KALMAN FILTER RGB]  mean: "<< mean << endl;
      cout<< " [KALMAN FILTER RGB]  stdev: "<< KF_RGB.stdev << endl;
      
   
      if (abs(c_obs) - abs(KF_RGB.xh_[1]) >  2.5* KF_RGB.stdev and KF_RGB.K_count > 40)
      {
          cout<<"SONO IN RETURN PER STD"<<endl;
          cout<<"KF_RGB.xh_[1]"<<endl;
            return;
      }
      
      //Define Matrices for aritmetic evaluations 
      Eigen::Matrix2d res;
      Eigen::Matrix2d res1;
      Eigen::Matrix2d I;
      Eigen::Vector2d inov(0.0,0.0);
      
      I << 1.0 ,0.0,
      0.0, 1.0;
      //float res[N][N];
      //float inv[N][N]; // To store inverse of A[][]
      //float adj[N][N];   //To store the adj matrix to make the inverse
 
      if (KF_RGB.K_count == 0 and KF_RGB.initialization == true)
      {
          //Initialize Kalman Filter 
          int State_dim = 2;
          int Obs_dim = 2;
          
          //Initial Real STate 
            
          
          KF_RGB.X << a_target, c_target; 
           KF.X  =  KF_RGB.X ;
          
          KF_RGB.X_old << a_target, c_target;
           KF.X_old =  KF_RGB.X_old;
          
          KF_RGB.y << a_obs, c_obs;
           KF.y  =  KF_RGB.y;
          
          KF_RGB.xh << 0.0349, 2.24; // Inizializzo con a e b misurati dalla retta che passa tra i due GPS Waypoints
           KF.xh  =  KF_RGB.xh ;
          
          //KF.xh[1] = 2;
          
          KF_RGB.Px << 4.0, 0.0,
          0.0, 4.0;
          KF.Px  = KF_RGB.Px;
           
         KF_RGB.R   << 0.2, 0.0,  
          0.0, 0.25;
         
         KF.R << 0.2, 0.0, //é composta dalla variannza dell'errore tra le osservazioni e lo stato reale osservato. é l'errroe di misurazione 
          0.0, 0.054;
         
          KF_RGB.A << 1.0, 0.0,
          0.0,1.0;
          KF.A = KF_RGB.A;
          
          KF_RGB.C << 1.0, 0.0,
          0.0,1.0;
          KF.C = KF_RGB.C;
          
          KF_RGB.K << 0.0, 0.0,
          0.0, 0.0;
           KF.K =  KF_RGB.K ;
           
          KF_RGB.Px_ << 0.0, 0.0,
          0.0, 0.0;
          KF.Px_ = KF_RGB.Px_;
          
          KF_RGB.xh_old << 0.0, 0.0;
          KF.xh_old =  KF_RGB.xh_old;
          
          KF.initialization = false;
          KF_RGB.initialization = false;
   
        }
         else
        {
           if (target_point == 2 and KF_RGB.K_count > 0) //Forse meglio con la distanza
           { 
               //Update state iteration
            KF_RGB.X << KF_RGB.X_old[0], KF_RGB.X_old[1]; 
            
            KF_RGB.y << a_obs, c_obs;
            cout<<"[KALMAN FILTER RGB] Update state Iteration ---- "<<endl;
           
            
           }
           else
           {
               /*
                KF_RGB.c_obs_v = vector<float>();
                diff = vector<double>();
                */
                
                 KF_RGB.X << a_target, c_target;
          
                
                 KF_RGB.X_old << a_target, c_target;
          
                
                 KF_RGB.y << a_obs, c_obs;
                 
                 //KF.xh <<1.5* KF.xh_old[0] , KF.xh_old[1] -1.5* (KF.quad_y_old - drone.drone_y ); //lo stato iniziale va inizializzato come quello precedente piu la distanza massima tra i pannelli espressa da drone.drone_y - KF.quad_y_old              
                 KF_RGB.xh << 1* KF_RGB.xh_old[0] , KF_RGB.xh_old[1] -1* (KF_RGB.quad_y_old - drone.drone_y);  
                 
                                
                 
                 KF_RGB.Px << 0.3, 0.0,
                  0.0, 0.3;
                 
           
                 KF_RGB.R << 0.2, 0.0,  
                    0.0, 0.25;
                
                 
                 KF_RGB.A << 1.0, 0.0,
                 0.0,1.0;
                 
                 
                 
                 KF_RGB.C << 1.0, 0.0,
                 0.0,1.0;
                 
                 
                 
                 KF_RGB.K << 0.0, 0.0,
                 0.0, 0.0; 
                 
          
                 KF_RGB.Px_ << 0.0, 0.0,
                 0.0, 0.0;
                
                 
            
                
                
                
                
                
               //Al momento del cambio di pannello il KF relativo alle immagini termiche si occupa di aggiornare lo stato col valroe corretto
               //return;
           }
        }
   
    KF_RGB.RGB_update = true;
    
      //*************** MULTI DIM KF ************//
//-------------------  A Priori estimate of the current state x(t|t-1) = A*x(t-1|t-1))
    
      if  ( KF.thermal_update == true)
      {
         KF_RGB.xh_ <<  KF.xh[0],  KF.xh[1];
         cout << "[KALMAN FILTER RGB] --> thermo_update --> KF_RGB.xh:" << KF_RGB.xh << endl; 
         
        //--------------------- A priori estimate of the state covariance matrix P(t|t-1) = A *P(t-1|t-1) * A' + Q)
        //Q = 4*4 zero matrix
     
         KF_RGB.Px_ = KF_RGB.A * KF.Px * KF_RGB.A;
         KF.thermal_update = false;    
      }
      else
      {
          KF_RGB.xh_ <<  KF_RGB.xh[0],  KF_RGB.xh[1];
         cout << "[KALMAN FILTER RGB]  KF_RGB.xh:" << KF_RGB.xh << endl; 
         KF_RGB.Px_ = KF_RGB.A * KF_RGB.Px * KF_RGB.A; 
      }
          
      
      

//------------------- MEASUREMENT UPDATE: 
//---------Kalamn Filter update K(t) = P(t|t-1) * C' * inv(C*P(t|t-1) * C' + R)
       res = KF_RGB.Px_ * KF_RGB.C;
      
       res1 = KF_RGB.C * KF_RGB.Px_* KF_RGB.C + KF_RGB.R;
      
       res1 = res1.inverse().eval();
      
       KF_RGB.K = res * res1;
     
       
      // cout << "[KALMAN FILTER]  KF.K:" << KF.K << endl; 
   
    
    
 //------------  Estimated  observation y(t|t-1) = C*X(t|t-1)
       KF_RGB.yh_ = KF_RGB.C * KF_RGB.xh_;
      // cout << "[KALMAN FILTER]  KF.xh:" << KF.xh << endl; 
       //cout << "[KALMAN FILTER]  KF.xh_:" << KF.xh_ << endl; 
       //cout << "[KALMAN FILTER]  KF.yh_:" << KF.yh_ << endl; 
       //cout << "[KALMAN FILTER]  KF.y:" << KF.y << endl;
      
// ----------- Measuremet residual innovation error y(t) - y(t|t-1)

       inov = KF_RGB.y.transpose() - KF_RGB.yh_.transpose();
       //cout << "[KALMAN FILTER] inov:" << inov << endl; 

//------------ A posteriori updated estimate of the current state x(t|t) = x(t|t-1) + K(t)*((y(t) - y(t|t-1))
     KF_RGB.xh = KF_RGB.xh_ + KF_RGB.K * inov;

//------------ A posteriori updated state covariance matrix P(t|t) = (I - K(t)*C) * P(t|t-1))
     
     KF_RGB.Px = (I - KF_RGB.K* KF_RGB.C) * KF_RGB.Px_;
     
    
     cout << "[KALMAN FILTER RGB] KF_RGB.xh_:" << KF_RGB.xh_ << endl; 
 
     
    
     Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(KF_RGB.Px);
     if (eigensolver.info() != Eigen::Success) abort();
     Eigen::Vector2d eigenvalues1 =  eigensolver.eigenvalues();
    
     KF_RGB.eigen_Px = eigenvalues1; 
     cout<<" Eigen values of KF RGB Px: "<< eigenvalues1<< endl;
     
    
     
     
     KF_RGB.K_count = KF_RGB.K_count + 1;
       KF_RGB.quad_y_old = drone.drone_y;
       KF_RGB.xh_old =  KF_RGB.xh;      
       
           
           
}

//***********************  KALMAN FILTER thermal LINE PARAMETER ESTIMATION *******************//
void Kalman_Filter(float x_target_P1, float y_target_P1, float x_target_P2, float y_target_P2, int target_point)
{
     float x1_obs = 0.0;
     float y1_obs = 0.0;
     float x2_obs = 0.0;
     float y2_obs = 0.0;
   //Ruotare Punti termo nel GF 

      Rotation_BF_to_GF_des_pos(drone.control_Thermo_point1_x,  drone.control_Thermo_point1_y, drone.drone_Yaw); //---RIsultato finisce in check_x_w e check_y_w variabili globali
      x1_obs = check_x_w;
      y1_obs = check_y_w;
      
      Rotation_BF_to_GF_des_pos(drone.control_Thermo_point2_x, drone.control_Thermo_point2_y, drone.drone_Yaw); 
      x2_obs = check_x_w;
      y2_obs = check_y_w;
      
      //Evaluate a,b,c equation parameters given the the target points in GFs
      float a_target = (y_target_P2 - y_target_P1)/(x_target_P2 - x_target_P1);
      float b_target = 1;
      float c_target = ((-1*a_target * x_target_P1) + y_target_P1);
      
      //Evaluate a,b,c equation parameters given the the obs points in GF
      float a_obs = (y2_obs - y1_obs)/(x2_obs - x1_obs);
      float b_obs = 1;
      float c_obs = ((-1*a_obs * x1_obs) + y1_obs);
      
      cout<< "[KALMAN FILTER] a_target : "<< a_target << "     c_target: "<< c_target << endl;
      
      cout<< "[KALMAN FILTER] a_obs : "<< a_obs <<"    c_obs: "<< c_obs << endl;
      
      
      //Define sitution when the KF is not updated
      if (isnan(a_obs) == 1 or isnan(c_obs) == 1)
      {
          return;
      }
      
      //Check per riportare il xh al valore reale una volta che diverge 
      
      
     if (KF.K_count < 10)
     {
      
           if (c_obs < KF.xh_[1] - 3.5 or c_obs > KF.xh_[1] + 3.5)
           {
               return;
           }
      }
      else
      {
          if (a_obs < KF.xh_[0] - 0.2 or a_obs >  KF.xh_[0] + 0.2)
          {
             //Eliminate very wrong predictions
             return;
           }
           
            if (c_obs < KF.xh_[1] - 2.0 or c_obs > KF.xh_[1] + 2.0)
           {
             
               return;
           }
      }
      
       //Evaluate mean and covariance of c_obs
      KF.c_obs_v.push_back(c_obs);
      float sum = accumulate(KF.c_obs_v.begin(), KF.c_obs_v.end(), 0.0); //SOmma i valori nel vettore partendo da zero
      float mean = sum / KF.c_obs_v.size();
      vector<double> diff(KF.c_obs_v.size());
      transform(KF.c_obs_v.begin(), KF.c_obs_v.end(), diff.begin(),
               bind2nd(std::minus<double>(), mean));
      double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
      KF.stdev = sqrt(sq_sum / KF.c_obs_v.size());
       
      cout<< "########### [KALMAN FILTER]  mean: "<< mean << endl;
      cout<< "########### [KALMAN FILTER]  stdev: "<< KF.stdev << endl;
      
     
      
      cout<< "[KALMAN FILTER]  abs(c_obs) - abs(KF.xh_[1]) = "<< abs(c_obs) - abs(KF.xh_[1]) << endl;
      if (abs(c_obs) - abs(KF.xh_[1]) >  2.5* KF.stdev and KF.K_count > 40)
      {
           cout<< "########### [KALMAN FILTER]  CAMBIARE OSSERVAZIONI #######################: "<< endl;
           /*
           if (KF.switch_point == 1)
           {
               KF.switch_point = 2;
           }
           else
           {
               KF.switch_point = 1;
           }
          */ 
            return;
      }
      
       
      //Define Matrices for aritmetic evaluations 
      Eigen::Matrix2d res;
      Eigen::Matrix2d res1;
      Eigen::Matrix2d I;
      Eigen::Vector2d inov(0.0,0.0);
      
      I << 1.0 ,0.0,
      0.0, 1.0;
      //float res[N][N];
      //float inv[N][N]; // To store inverse of A[][]
      //float adj[N][N];   //To store the adj matrix to make the inverse
 
      if (KF.K_count == 0 and KF.initialization == true)
      {
          //Initialize Kalman Filter 
          int State_dim = 2;
          int Obs_dim = 2;
          
          //Initial Real STate 
            
          
          KF.X << a_target, c_target; 
          KF_RGB.X =  KF.X ;
          
          KF.X_old << a_target, c_target;
          KF_RGB.X_old = KF.X_old;
          
          KF.y << a_obs, c_obs;
          KF_RGB.y =  KF.y; 
          
          //Lo stato è inizializzato con i valori di a e c della retta che passa tra i due GPS waypoints
          KF.xh << 0.0349, 2.24;
          KF_RGB.xh = KF.xh;
          
          //KF.xh[1] = 2;
          
          KF.Px << 4.0, 0.0, //Va inizializzata con l'errore che si suppone di avere tra l'inizializazzione dello stato e il valore vero dello stato iniziale 
          0.0, 4.0;
          KF_RGB.Px = KF.Px;
           
        //R è calcolata per via sperimentale, in quanto ho calcolato la varianza dell'errore tra le osservazioni e los statto vero
          KF.R << 0.2, 0.0, //é composta dalla variannza dell'errore tra le osservazioni e lo stato reale osservato. é l'errroe di misurazione 
          0.0, 0.053;
          KF_RGB.R << 0.2, 0.0,  
          0.0, 0.25;
         
          KF.A << 1.0, 0.0,
          0.0,1.0;
          KF_RGB.A  =  KF.A ;
           
          KF.C << 1.0, 0.0,
          0.0,1.0;
          KF_RGB.C =  KF.C;
          
          KF.K << 0.0, 0.0,
          0.0, 0.0;
          KF_RGB.K = KF.K;
          
          KF.Px_ << 0.0, 0.0,
          0.0, 0.0;
          KF_RGB.Px_ =  KF.Px_ ; 
           
          KF.xh_old << 0.0, 0.0;
          KF_RGB.xh_old = KF.xh_old;
          
          KF.initialization = false;
          KF_RGB.initialization = false;
   
        }
        else
        {
           if (target_point == 2 and KF.K_count > 0) //Forse meglio con la distanza
           { 
               //Update state iteration
            KF.X << KF.X_old[0], KF.X_old[1]; 
            
            KF.y << a_obs, c_obs;
            cout<<"[KALMAN FILTER] Update state Iteration ---- "<<endl;
           
            
           }
           else
           {
             if (drone.drone_y  >  KF.quad_y_old) 
             {
                 //Lo stato iniziale sulla c va umentato nella nuova inizializazzione in quanto il drone si è spostato lungo le y in maniera crescente 
                 //durante il cambio di pannello
                 
                 
                 //riniializzo P a dei valori simili a quelli dello stato ma piu grandi.. tuttavia non troppo
                 // x 
                 // ecc come in K = 0
                 
                 KF.X << a_target, c_target; 
                 KF_RGB.X << a_target, c_target;
          
                 KF.X_old << a_target, c_target;
                 KF_RGB.X_old << a_target, c_target;
          
                 KF.y << a_obs, c_obs;
                 KF_RGB.y << a_obs, c_obs;
                 
                 KF.xh <<1* KF.xh_old[0] , KF.xh_old[1] +1* (drone.drone_y  -  KF.quad_y_old); //lo stato iniziale va inizializzato come quello precedente piu la distanza massima tra i pannelli espressa da drone.drone_y - KF.quad_y_old               
                 KF_RGB.xh << 1* KF.xh_old[0] , KF.xh_old[1] +1* (drone.drone_y  -  KF.quad_y_old);  
                 
                 
                 //Px è l'errore che si suppone di avere tra lo stato iniziale supposto e il vero stato iniziale.
                 //P evolve durante il processo di predizione e tende a diminuire perche si suppone che lea predizione dello stato sia 
                 //piu simile a quello reale  iterazione dopo iterazione
                 //Tuttava Ph_ rappresenta la covarianza dell'errore stimato tra stato stimato e stato reale.
                 KF.Px << 0.3, 0.0,
                  0.0, 0.3;
                 KF_RGB.Px =  KF.Px;
           
                 KF.R << 0.2, 0.0, //é composta dalla variannza dell'errore tra le osservazioni e lo stato reale osservato. é l'errroe di misurazione 
                 0.0, 0.053;
                 KF_RGB.R << 0.2, 0.0,  
                 0.0, 0.25;
                 
                 KF.A << 1.0, 0.0,
                 0.0,1.0;
                 KF_RGB.A = KF.A;
                 
                 
                 KF.C << 1.0, 0.0,
                 0.0,1.0;
                 KF_RGB.C = KF.C;
                 
                 
                 KF.K << 0.0, 0.0,
                 0.0, 0.0; 
                 KF_RGB.K = KF.K;
          
                 KF.Px_ << 0.0, 0.0,
                 0.0, 0.0;
                 KF_RGB.Px_ = KF.Px_;
                 
                  //Free the std vector  
                 KF.c_obs_v = vector<float>();
                 diff = vector<double>();
                 
                
                 
                 cout<<"################################ "<< endl; 
                 cout<<"KALMAN FILTER] ---- SPostameneto lungo Y positive "<<endl;
                cout<<"################################ "<< endl;    
                 
                 
             }
             else
             {
               //il drone si è spostato negativammte lungo le y quindi lo stato iniziale per la nuova vela va inizializzato al valore precendente meno un offsett maggiore della distanza dei due pannelli
             
             
             
                 KF.X << a_target, c_target; 
                 KF_RGB.X << a_target, c_target;
          
                 KF.X_old << a_target, c_target;
                 KF_RGB.X_old << a_target, c_target;
          
                 KF.y << a_obs, c_obs;
                 KF_RGB.y << a_obs, c_obs;
                 
                 KF.xh <<1.5* KF.xh_old[0] , KF.xh_old[1] -1.5* (KF.quad_y_old - drone.drone_y ); //lo stato iniziale va inizializzato come quello precedente piu la distanza massima tra i pannelli espressa da drone.drone_y - KF.quad_y_old              
                 KF_RGB.xh << 1.5* KF.xh_old[0] , KF.xh_old[1] -1.5* (KF.quad_y_old - drone.drone_y );  
                 
                                
                 
                 KF.Px << 0.3, 0.0,
                  0.0, 0.3;
                 KF_RGB.Px =  KF.Px;
           
                 KF.R << 0.2, 0.0, //é composta dalla variannza dell'errore tra le osservazioni e lo stato reale osservato. é l'errroe di misurazione 
                 0.0, 0.053;
                 KF_RGB.R << 0.2, 0.0,  
                 0.0, 0.25;
                 
                 KF.A << 1.0, 0.0,
                 0.0,1.0;
                 KF_RGB.A = KF.A;
                 
                 
                 KF.C << 1.0, 0.0,
                 0.0,1.0;
                 KF_RGB.C = KF.C;
                 
                 
                 KF.K << 0.0, 0.0,
                 0.0, 0.0; 
                 KF_RGB.K = KF.K;
          
                 KF.Px_ << 0.0, 0.0,
                 0.0, 0.0;
                 KF_RGB.Px_ = KF.Px_;
                 
                  //Free the std vector  
                 KF.c_obs_v = vector<float>();
                 diff = vector<double>();
                 
                 cout<<"################################ "<< endl; 
                 cout<<"KALMAN FILTER] ---- SPostameneto lungo Y negative "<<endl;
                  cout<<"################################ "<< endl; 
                 
              }
             
             
           }
        }
    
   
      KF.thermal_update = true;
      //*************** MULTI DIM KF ************//
//-------------------  A Priori estimate of the current state x(t|t-1) = A*x(t-1|t-1))
    
      if  (KF_RGB.RGB_update == true)
      {
          KF.xh_ <<  KF_RGB.xh[0],  KF_RGB.xh[1];
         cout << "[KALMAN FILTER] --> RGB_update --> KF.xh:" << KF.xh_ << endl; 
         
        //--------------------- A priori estimate of the state covariance matrix P(t|t-1) = A *P(t-1|t-1) * A' + Q)
        //Q = 4*4 zero matrix
     
          KF.Px_ = KF.A * KF_RGB.Px * KF.A;
      }
      else
      {
          KF.xh_ <<  KF.xh[0],  KF.xh[1];
         cout << "[KALMAN FILTER]  KF.xh:" << KF.xh << endl; 
         KF.Px_ = KF.A * KF.Px * KF.A; 
      }
          
      
      

//------------------- MEASUREMENT UPDATE: 
//---------Kalamn Filter update K(t) = P(t|t-1) * C' * inv(C*P(t|t-1) * C' + R)
       res = KF.Px_ * KF.C;
      
       res1 = KF.C * KF.Px_* KF.C + KF.R;
      
       res1 = res1.inverse().eval();
      
       KF.K = res * res1;
     
       
      // cout << "[KALMAN FILTER]  KF.K:" << KF.K << endl; 
   
    
    
 //------------  Estimated  observation y(t|t-1) = C*X(t|t-1)
       KF.yh_ = KF.C * KF.xh_;
      // cout << "[KALMAN FILTER]  KF.xh:" << KF.xh << endl; 
       //cout << "[KALMAN FILTER]  KF.xh_:" << KF.xh_ << endl; 
       //cout << "[KALMAN FILTER]  KF.yh_:" << KF.yh_ << endl; 
       //cout << "[KALMAN FILTER]  KF.y:" << KF.y << endl;
      
// ----------- Measuremet residual innovation error y(t) - y(t|t-1)

       inov = KF.y.transpose() - KF.yh_.transpose();
       //cout << "[KALMAN FILTER] inov:" << inov << endl; 

//------------ A posteriori updated estimate of the current state x(t|t) = x(t|t-1) + K(t)*((y(t) - y(t|t-1))
     KF.xh = KF.xh_ + KF.K * inov;

//------------ A posteriori updated state covariance matrix P(t|t) = (I - K(t)*C) * P(t|t-1))
     
     KF.Px = (I - KF.K* KF.C) * KF.Px_;
     
    
     cout << "[KALMAN FILTER] KF.xh_:" << KF.xh_ << endl; 
 
   
    
     Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(KF.Px);
     if (eigensolver.info() != Eigen::Success) abort();
     Eigen::Vector2d eigenvalues1 =  eigensolver.eigenvalues();
    
     KF.eigen_Px = eigenvalues1; 
     cout<<" Eigen values of Px via eigensolver: "<< eigenvalues1<< endl;
    
    
       KF.K_count = KF.K_count + 1;
       KF.quad_y_old = drone.drone_y;
       KF.xh_old =  KF.xh;
}




void  KF_estimation_exportation_in_BF_for_control_point_generation()
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
      
      //Take the most recently estimation available for the evaluation of the control point
      if  (KF_RGB.RGB_update == true and  KF.thermal_update == false) //Commentare quando si fanno test solo con RGB
      //if  (KF.thermal_update == false)
      {
          //Use the RGB KF estimation
          y1_w =  KF_RGB.xh[0]*x1_w + KF_RGB.xh[1];
          y2_w =  KF_RGB.xh[0]*x2_w + KF_RGB.xh[1];
          cout<<"[KF EXP FUNC] KF RGB estimation exported for control Point Evaluation" << endl;
          cout<<"[KF EXP FUNC] y1_w: " <<y1_w << ", "<< "y2_w: " << y2_w << endl;
      }
      else 
      {
           //Use the thermal KF estimation in the case that RGB are not available or if both estimation are avalilable 
           //since the thermal KF is the last in the sequential KF structure 
          y1_w =  KF.xh[0]*x1_w + KF.xh[1];
          y2_w =  KF.xh[0]*x2_w + KF.xh[1];
          if (KF_RGB.RGB_update == false and  KF.thermal_update == true)
          {
              cout<<"[KF EXP FUNC] ONLY KF Thermal estimation exported for control Point Evaluation" << endl;
          }
          else 
          {
             cout<<"[KF EXP FUNC] RGB and THERMAL KF estimation exported for control Point Evaluation" << endl;
          }
           cout<<"[KF EXP FUNC] y1_w: " <<y1_w << ", "<< "y2_w: " << y2_w << endl;
           cout<<"[KF EXP FUNC] y_drone: " <<drone.drone_y << endl;
      }
    
     //ROtate both points in body frame 
      Rotation_GF_to_BF_des_pos(x1_w, y1_w,drone.drone_Yaw);
      x1_b = check_x_b;
      y1_b = check_y_b;
      
      Rotation_GF_to_BF_des_pos(x2_w, y2_w,drone.drone_Yaw);
      x2_b = check_x_b;
      y2_b = check_y_b;
    
    
     //Evaluate a,c parameters of the line in body frame
       KF.a_b =  ((y2_b  - y1_b)/(x2_b - x1_b));
       KF.c_b = ((-(KF.a_b) * x1_b) + y1_b);
      
       cout << "[KF EXP FUNC] KF.a_b: " << KF.a_b << ", " <<"KF.c_b: "<< KF.c_b << endl; 
       
       
       //I paramteri KF.a_b e KF.c_b entreranno nella funzione 
      // evaluate_control_point_from_Thermal_image_points dove verra stimato il punto di controllo predittivo da seguire collocato sulla retta
      // stimata dal filtro. 
      //Non è piu necessaria la funzione evaluate_control_point_from_RGB_image_points in quanto titto viene elaborata dalla seconda siccome 
      //il punto di controllo è valutato intermamente sulla stima in output dai filtri. 
      //Se non fossero presenti osservazioni RGB o termiche  la struttura software relativa al filtraggio gestisce in autonomo la prosuzione della stima, 
      //la quale viene sempre aggiornata indipendentemente dal tipo di informazioni in inputb ai filtri
      
      
}








int main(int argc, char** argv)
{
    ros::init(argc, argv, "Altitude_controller");

    ros::NodeHandle nh;
    
    bool Take_off = false;
    bool flag_even = false;
    bool flag_end_point = false;

   //PARAMETRS 
    int n_panels = 4;
    float distance_between_panels = 2.2; //1.84;
    int configuration = 0;  //1 se si vuole la configurazione con panneli a ferro di cavallo
     
    
    
    //Create a vectore of struceture for each panel
    //vector<Panel> panel_vec;
    Structure panel;
    
    
    //Per le misure di ciasucna configurazione fare riferimento al file ardrone_testworld.world
    panel.x_center = 6.37;
    panel.y_center = 2; //1.60;//1.60;
    panel.theta = 0; //45.0 * M_PI/180;
    panel.size = 1.0;
    panel.length = 12;
    
    //GPS error
    waypoints.gamma = 2*M_PI/180; //Rotational error in rad
    waypoints.delta = 1.0; //Traslational error along x respect panel frame located in panel 1
    waypoints.eta = 0.5; //Traslational error along y
    
    //Inserisco i centri nel vettore 
    panel.xcenter_panel_world_frame.push_back( panel.x_center);
    panel.ycenter_panel_world_frame.push_back( panel.y_center);
    cout <<  "panel.xcenter_panel_world_frame[0]: "<< panel.xcenter_panel_world_frame[0]<<endl;
    cout <<  "panel.ycenter_panel_world_frame[0]: "<< panel.ycenter_panel_world_frame[0]<<endl;
    
    
    
    float a = 0.0;
    float b = 0.0;
    
     //Place panel centers 
     place_panel_centers(&panel, n_panels, distance_between_panels);
     
     //Place Point Start P1 and end P2 in map for each panel
     place_panel_start_end_point(&panel, n_panels, distance_between_panels, configuration);
     

    //Write Panel real point in a file
    std::ofstream outFile("sim_data_complete/panel_x_point_in_world_frame.txt");
    std::ofstream outFile1("sim_data_complete/panel_y_point_in_world_frame.txt");
    for (int i = 0; i < panel.xcoo_panel_world_frame.size(); i++)
    {
        outFile << panel.xcoo_panel_world_frame[i] << "\n";
        outFile1 << panel.ycoo_panel_world_frame[i] << "\n";
    }
    
    //Write GPS real point in a file
    std::ofstream outFile2("sim_data_complete/GPS_x_point_in_world_frame.txt");
    std::ofstream outFile3("sim_data_complete/GPS_y_point_in_world_frame.txt");
    for (int i = 0; i <   waypoints.waypoints_x_coo_world_frame.size(); i++)
    {
        outFile2 <<   waypoints.waypoints_x_coo_world_frame[i] << "\n";
        outFile3 <<   waypoints.waypoints_y_coo_world_frame[i] << "\n";
    }
    
    
    for (int i = 0; i < waypoints.waypoints_x_coo_gps_frame.size(); i++)
    {
       cout<< waypoints.waypoints_x_coo_gps_frame[i] << endl;
       cout<< waypoints.waypoints_x_coo_gps_frame[i] << endl; 
    }
    
 
      
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
     
    PID pid_z = PID(dt, 3, 3, Kp_z, Kd_z, 0.01);
    PID pid_yaw = PID(dt, 1, -1, Kp_yaw, Kd_yaw, 0.01);
    
    
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
    
    
    
    // Publish and Subscribers Topic
    ros::Publisher takeOff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
    ros::Publisher Land = nh.advertise<std_msgs::Empty>("/ardrone/land",1);
    ros::Publisher vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //Publish drone position to camera in order to create idealistic gimbal
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient client_RGB = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    
    
    ros::Subscriber odom_drone_sub = nh.subscribe("/ground_truth/state", 50, drone_odom_callback);
    ros::Subscriber imu_drone_sub = nh.subscribe("/ardrone/imu", 5, drone_Imu_callback);
    ros::Subscriber vel_drone_sub = nh.subscribe("/fix_velocity", 1, drone_fix_Vel_callback);
    //Subscribe to the control points extracted from Thermal images 
    ros::Subscriber control_point_1 = nh.subscribe("/Thermo_control_point_1", 1, drone_Thermo_control_point1_callback);
    ros::Subscriber control_point_2 = nh.subscribe("/Thermo_control_point_2", 1, drone_Thermo_control_point2_callback);
    
    //Subscribe to the second control points extracted from Thermal images 
    ros::Subscriber control_point_11 = nh.subscribe("/Thermo_control_point2_1", 1, drone_Thermo_control_point1_callback2);
    ros::Subscriber control_point_22 = nh.subscribe("/Thermo_control_point2_2", 1, drone_Thermo_control_point2_callback2);
    
    //Subscribe to the control points extracted from RGB images 
    ros::Subscriber control_RGB_point_1 = nh.subscribe("/RGB_control_point_1", 1, drone_RGB_control_point1_callback);
    ros::Subscriber control_RGB_point_2 = nh.subscribe("/RGB_control_point_2", 1, drone_RGB_control_point2_callback);
    
    
    ros::Rate r(20); // 20
    double secs =ros::Time::now().toSec();
    
    //Counter 
    int count = 0;
    int count_setpoint = 0;
    bool gps_waypoint = false;
    bool flag_create_setpoint = true;
    bool coming_back_to_GPS_path = false;
    
    //Target : GPS o P ogni inizio pannello
    float x_target_P1 = 0.0;
    float y_target_P1 = 0.0; 
    float x_target_P2 = 0.0;
    float y_target_P2 = 0.0; 
    
    float x_target = 0.0;
    float y_target = 0.0;
    
    
    
    //Only for Debug Purpose
    float y_target_vision = 0.0;
    float y_target_GPS = 0.0;
    //int target_point = panel.target_point; //Suggerisce il segno del target al quale il drone deve puntare in evaluate control point. --> permette di definire la direzione del vettore Vx
    // Write Desired and real velocity
   
        std::ofstream outFile4("sim_data_complete/des_x_vel.txt");
        std::ofstream outFile5("sim_data_complete/des_y_vel.txt");
        std::ofstream outFile6("sim_data_complete/des_z_vel.txt");
        std::ofstream outFile7("sim_data_complete/x_vel.txt");
        std::ofstream outFile8("sim_data_complete/y_vel.txt");
        std::ofstream outFile9("sim_data_complete/z_vel.txt");
        std::ofstream outFile10("sim_data_complete/drone_x_pos_saved.txt");
        std::ofstream outFile11("sim_data_complete/drone_y_pos_saved.txt");
        std::ofstream outFile13("sim_data_complete/y_error_VISION_LINE.txt");
        std::ofstream outFile14("sim_data_complete/roll.txt");
    
        std::ofstream outFile15("sim_data_complete/a_GPS.txt");
        std::ofstream outFile16("sim_data_complete/c_GPS.txt");
        std::ofstream outFile17("sim_data_complete/a_est.txt");
        std::ofstream outFile18("sim_data_complete/c_est.txt");
        std::ofstream outFile19("sim_data_complete/x_target.txt");
        std::ofstream outFile20("sim_data_complete/y_target.txt");
        std::ofstream outFile21("sim_data_complete/a_obs.txt");
        std::ofstream outFile22("sim_data_complete/c_obs.txt");
        
        //Da cancellare poi 
        std::ofstream outFile23("sim_data_complete/KF_std_dev.txt");
        
        std::ofstream outFile24("sim_data_complete/a_RGB_est.txt");
        std::ofstream outFile25("sim_data_complete/c_RGB_est.txt");
        std::ofstream outFile26("sim_data_complete/KF_RGB_std_dev.txt");
        std::ofstream outFile27("sim_data_complete/a_RGB_obs.txt");
        std::ofstream outFile28("sim_data_complete/c_RGB_obs.txt");
        std::ofstream outFile29("sim_data_complete/Eigen_Px_KF_RGB.txt");
        std::ofstream outFile30("sim_data_complete/Eigen_Px_KF_thermal.txt");
        std::ofstream outFile31("sim_data_complete/drone_x_pos.txt");
        std::ofstream outFile32("sim_data_complete/drone_y_pos.txt");
        std::ofstream outFile33("sim_data_complete/drone_z_pos.txt");
        std::ofstream outFile34("sim_data_complete/error_from_GPS_line.txt");
        std::ofstream outFile35("sim_data_complete/error_from_vision_line.txt");
        
        
        
        
        
        
        
    int count_hovering = 0;
    KF.initialization = true;
    KF_RGB.initialization = true;
    while(nh.ok()) {
        geometry_msgs::Twist drone_vel_msg;
       
        
        if(Take_off == false) {
            takeOff.publish(myMsg);
            drone.desired_z = 5;
            //Increasing altitude 
            drone_vel_msg.linear.z = pid_z.position_control_knowing_velocity(drone.desired_z, drone.drone_z, 0, drone.drone_lin_vel_z); // des_pos, actual_pose, des_vel, actual_vel

            //Align Yaw drone with solar Panel
            
            //yaw_des = atan2(panel_vec[count].P1_y,panel_vec[count].P1_x);
            yaw_des = atan2(panel.ycoo_panel_world_frame[count + 1],panel.xcoo_panel_world_frame[count + 1]);
            
            drone_vel_msg.angular.z  = pid_yaw.position_control_knowing_velocity(yaw_des, drone.drone_Yaw, 0, drone.drone_ang_vel_z);
            
            cout << " drone_vel_msg.angular.z:  " <<  drone_vel_msg.angular.z << endl;
            cout << " drone_yaw:  " <<  drone.drone_Yaw<< endl;
            
           
            // publish the message
            vel.publish(drone_vel_msg);

            flagDroneOdom = false;
            flagDroneImu = false;
            flagDroneFix_vel = false;

            
            if (drone.drone_z < drone.desired_z - 0.5)
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
        drone_vel_msg.linear.z = Kp_z * (drone.desired_z - drone.drone_z) + Kd_z * (0 - drone.drone_lin_vel_z);
        
        drone_yaw_degree = drone.drone_Yaw * (180/M_PI);
        cout<<"[MAIN]: drone_yaw_degree: "<< drone_yaw_degree<<endl;
        
        float vel_x = 0.0;
        float vel_y = 0.0;
        float y = 0.0;
        
        
        disturb = sin(0.1 * time);
        
        if (flag_end_point == false) //sono in Start point panel count
        {
            
            if (count == 0)
            {
                
                //Le coordinate di target sono le coordinate P1 del nuovo pannello, conosciute alla prima iterzione
                x_target_P1 =  panel.xcoo_panel_world_frame[count];
                y_target_P1 =  panel.ycoo_panel_world_frame[count];
                
                x_target_P2 =  panel.xcoo_panel_world_frame[count + 1];
                y_target_P2 =  panel.ycoo_panel_world_frame[count + 1];
             //####################################  DECOMMEMTARE #################################   
                ////TEST ---> EVALUATE CONTROL POINT FROM IMAGES 
                //evaluate_control_point_from_image_points(target_point);
                
               // Kalman_Filter(x_target_P1,  y_target_P1,  x_target_P2,  y_target_P2, panel.target_point);
                
                //Alla prima iterazione conosco le coordinate di P1 e P2 nel frame world 
                evaluate_control_point(x_target_P1, y_target_P1, x_target_P2, y_target_P2,panel.target_point );
                
              //####################################  DECOMMEMTARE #################################      
                
                x_target = panel.xcoo_panel_world_frame[count];
                y_target = panel.ycoo_panel_world_frame[count];

                cout << "[START MISSION]--> Reaching P1 panel "<< count +1 << " --> distance --> " <<  cartesian_distance_err<< endl;
            }
            else 
            {
                //Le coordinate di target sono le coordinate P1 del nuovo pannello, simulo l'aggancio al pannello stesso
                
                x_target =  drone.next_point_P1_x[drone.next_point_P1_x.size() -1];
                y_target = drone.next_point_P1_y[drone.next_point_P1_y.size() -1];
                cout<<"x_target: " << x_target <<endl;
                cout<<"y_target: " << y_target <<endl;
                cout << "[NEXT PANEL] --> Reaching next point P1 evaluated from GPS "<< count +1 << "distance --> " <<  cartesian_distance_err<< endl;
            }
                 
            

             cartesian_distance_err = sqrt(pow(x_target- drone.drone_x,2) + pow(y_target- drone.drone_y,2));
             //Attitude
             
             drone_vel_msg.angular.z  = pid_yaw.position_control_knowing_velocity(yaw_des , abs(drone.drone_Yaw), 0, drone.drone_ang_vel_z);
             if (drone.drone_Yaw < 0)
             {
                 drone_vel_msg.angular.z = -1* drone_vel_msg.angular.z;
             }
            // cout << " drone_vel_msg.angular.z : "<<  drone_vel_msg.angular.z  << endl;
             yaw_err = yaw_des - abs(drone.drone_Yaw) ;
             
             
             //cout << "yaw_des: "<< yaw_des *180/M_PI << endl;
             //cout<<"drone.drone_Yaw: "<<drone.drone_Yaw * 180/M_PI<<endl;
             //cout << "abs(yaw_err): "<< abs(yaw_err) << endl;
             if (abs(yaw_err) < 0.5 || count == 0)
             {
                 //Reaching Point Start P1
                
                 //Rotate positions desired position in body frame
                 
                 Rotation_GF_to_BF_des_pos(x_target , y_target,  drone.drone_Yaw);
                 
                 Rotation_GF_to_BF_drone_pos(drone.drone_Yaw);
                 //cout<<"drone.drone_Yaw: "<<drone.drone_Yaw<<endl;
                 
                 //PID control on velocity defined on body frame
                  //cout<<"[MAIN]:  check_x_b: "<<check_x_b<<endl;
                  //cout<<"[MAIN]:  check_y_b: "<<check_y_b<<endl;

                  drone_vel_msg.linear.x  = pid_x.calculate(check_x_b, drone.drone_x_b);
                  drone_vel_msg.linear.y = pid_y.calculate(check_y_b, drone.drone_y_b);
                  
                 
                  
                  //cout << "Reaching Start point Panel "<< count +1 << "distance --> " <<  cartesian_distance_err<< endl;
             }
             else 
             {
                
                 

                 //Rotation_GF_to_BF_des_pos(panel_vec[count - 1].P2_x, panel_vec[count - 1].P2_y, drone.drone_Yaw);
                 Rotation_GF_to_BF_des_pos(panel.xcoo_panel_world_frame[count -1] , panel.ycoo_panel_world_frame[count - 1],  drone.drone_Yaw);
                 
                 Rotation_GF_to_BF_drone_pos(drone.drone_Yaw);

                 // PID control on velocity defined on body frame
                
                 drone_vel_msg.linear.x = pid_x.calculate(check_x_b, drone.drone_x_b);
                 drone_vel_msg.linear.y = pid_y.calculate(check_y_b, drone.drone_y_b);

                 
             }
              //Raggiunto Point P1 mi dirigo verso point P2
              if (cartesian_distance_err < 0.2)
              {
                
                flag_end_point = true; //Mi dirigo lungo il pannello verso punto P2 di end --> passo all'else sottostante 
                flag_create_setpoint = true;
                if (count == 0)
                {
                    yaw_des = panel.theta;
                }
                count = count + 1; //--> Raggiungo punto P2 stesso pannello
                panel.target_point = 2;
              }
                  
                 //Se gps_waypoint = true
        }
        else 
        {
                
                x_target_P1 =  panel.xcoo_panel_world_frame[count- 1];
                y_target_P1 =  panel.ycoo_panel_world_frame[count - 1];
                
                x_target_P2 =  panel.xcoo_panel_world_frame[count];
                y_target_P2 =  panel.ycoo_panel_world_frame[count];
                
                
                
                
                //******************************** SEQUENTIAL  KALMAN FILTERING FOR RGB AND THERMAL IMAGES **********************************
                Kalman_Filter_RGB(x_target_P1, y_target_P1, x_target_P2,  y_target_P2,  panel.target_point);
                
                Kalman_Filter(x_target_P1,  y_target_P1,  x_target_P2,  y_target_P2, panel.target_point);
                
                //Exportation of the estimation genertated in drone body frame -> evaluation of two points in WF lies on the estimated line,
                //Exportation of this two point in BF for definition of the control law to folllow the line 
                KF_estimation_exportation_in_BF_for_control_point_generation();
                //******************************************************************************************************************+
                
                
                
                
                //Check if the drone is on the correct line defined by GPS
                evaluate_control_point(x_target_P1, y_target_P1, x_target_P2, y_target_P2, panel.target_point);
                
                
                ////TEST ---> EVALUATE CONTROL POINT FROM IMAGES 
                
                 //####################################  DECOMMEMTARE #################################   
                //I punti di controllo che vengono passati all'interno di questa funzione sono storati in KF,a_b e KF.c_b
                //Entrambi i paramteri vengono clacolati sia nel caso solo le osservazioni RGB siano presenti, sia nel caso solo le osservazioni termiche siano
                //presenti, sia entrambe le osservazioni
                evaluate_control_point_from_Thermal_image_points(panel.target_point); //---> In realtà ora questa funzione prende informazioni da entrambe le camere in quanto considera la stima finale del filtro
                 //evaluate_control_point_from_RGB_image_points(panel.target_point);
                 
                  //####################################  DECOMMEMTARE #################################   
                 
                 
                 
                 //Alla prima iterazione conosco le coordinate di P1 e P2 nel frame world --
                 //Se le coordinate ottenute dall'immagine sono 0.0 mi baso su quelle GPS--- mi baso sul GPS anche se sono in prossimità della fine del pannelo, verificnado la vicinanza con il waypoint 
               
                 /* Può succedere che il controllo del drone è riacquistato dalla linea tracciata tra i punti del GPS quando:
                 * ---> Il drone non ha ancora riconosciuto alcun pannello
                 * ---> Il drone è tot tempo che non riconosce piu alcun pannello
                 * ---> Il drone è prossimo al punto di arrivo di fine della vela
                 * ---> Il drone si è discostato per piu di un threshold th dalla retta traciata tra i punti GPS
                 * 
                 * 
                */
                
                //Check di controllo relativo alla fine della vela 
                if ( cartesian_distance_err < 2.0)
                      {
                         //Inserisco flag nel quale utilizza GPS route finche 
                         //non riconosce qualcosa dalle immagini 
                         init_panel = true;
                }
                      
               // cout<<"drone.switch_control: "<<drone.switch_control << endl;      
                switch (drone.switch_control)
                {
                    case 1:
                    //Thermal Images Control
                      
                      //if ((drone.image_control_x_coo == 0.0 and drone.image_control_y_coo == 0.0) or (drone.image_RGB_control_x_coo == 0.0 and drone.image_RGB_control_y_coo == 0.0 ) or abs(error_from_GPS_line) > 0.6 or cartesian_distance_err < 1.5 or init_panel == true or coming_back_to_GPS_path == true)
                     if ((drone.image_control_x_coo == 0.0 and drone.image_control_y_coo == 0.0)  or abs(error_from_GPS_line) > 1.5 or cartesian_distance_err < 1.5 or init_panel == true or coming_back_to_GPS_path == true)
                    //if ((drone.image_RGB_control_x_coo == 0.0 and drone.image_RGB_control_y_coo == 0.0)  or abs(error_from_GPS_line) > 1.6 or cartesian_distance_err < 1.5 or init_panel == true or coming_back_to_GPS_path == true)

                    
                     {
                          drone.switch_control = 2; //SWITCH TO GPS CONTROL
                      }
                      /*
                      else if (abs(error_from_GPS_line) > 0.6)
                      {
                          //SWITCH TO RGB CONTROL, IF SOMETHING IS RECOGNIZED
                           drone.switch_control = 2;
                          
                      }
                       */
                      else
                      {
                          // Acquisisco le informazioni anche da telecamera RGB (se presenti).
                          //Le osservazioni entrano nel KF e vengono fuse con le osservazioni termiche.
                          //Solo in evaluate control from thermal images vengono utilizzate le stime di Kalman per definire i target
                          
 //####################################  DECOMMEMTARE #################################   
                           //evaluate_control_point_from_Thermal_image_points(panel.target_point);
                           //evaluate_control_point_from_RGB_image_points(panel.target_point);
                           //x_target = abs(drone.image_RGB_control_x_coo); 
                           //y_target = drone.image_RGB_control_y_coo; 
                           //ROS_INFO("VISION RGB GUIDANCE"); 
                           
                           x_target = abs(drone.image_control_x_coo);  //Ottiene direttamente punti di controllo nel body frame
                           y_target = drone.image_control_y_coo; 
                           y_target_vision = y_target;
                           //cout<<"VISION THERMAL GUIDANCE"<<endl;
                           ROS_INFO("VISION THERMAL GUIDANCE"); 
                           drone.switch_control = 1; //MANTAIN THERMAL IMAGES CONTROL
//####################################  DECOMMEMTARE #################################   
                      }
                      
                      break;
                      
                      /*
                      case 2:
                      //RGB IMAGES CONTROL
                       //Verifico che il thermal control in background
                       evaluate_control_point_from_Thermal_image_points(target_point);
                       if ((drone.image_RGB_control_x_coo == 0.0 and drone.image_RGB_control_y_coo == 0.0 ) or cartesian_distance_err < 1.5 or abs(error_from_GPS_line) > 1 or init_panel == true or coming_back_to_GPS_path == true)
                       {
                           //Coming Back to GPS CONTROL
                           drone.switch_control = 3;
                           
                       }
                       if (drone.image_control_x_coo != 0.0 and drone.image_control_y_coo != 0.0)
                       {
                           //SE le coordinate del thermo control si sono aggiornate allora switch al case 1
                           drone.switch_control = 1;
                       }
                       else
                       {
                           evaluate_control_point_from_RGB_image_points(target_point);
                           x_target = abs(drone.image_control_x_coo); 
                           y_target = drone.image_control_y_coo; 
                           ROS_INFO("VISION RGB GUIDANCE"); 
                           drone.switch_control = 2; //MANTAIN RGB IMAGES CONTROL 
                       }
                       break;
                       */
                       case 2:
                       
                       //GPS CONTROL 
                       evaluate_control_point(x_target_P1, y_target_P1, x_target_P2, y_target_P2, panel.target_point); //EValuate GPS control Point
                       if (abs(error_from_GPS_line) > 1.5)
                       {
                          coming_back_to_GPS_path = true;
                          //GPS CONTROL
                         x_target = drone.control_x_coo; 
                         y_target = drone.control_y_coo; ;
                         y_target_GPS = y_target;
                         ROS_INFO("GPS CONTROL");
                          drone.switch_control = 2;
                       }
                      if (abs(error_from_GPS_line) < 0.15)
                      {
                          coming_back_to_GPS_path = false;
                          drone.switch_control = 1;
                      }
                       
                      if (init_panel == true or cartesian_distance_err < 1)
                      {
                         x_target = drone.control_x_coo; 
                         y_target = drone.control_y_coo; ;
                         y_target_GPS = y_target;
                         ROS_INFO("GPS CONTROL"); 
                         drone.switch_control = 2;
                      }
                       
                       
                       /*
                      if (coming_back_to_GPS_path == false or init_panel == false)
                      {
                      if (coming_back_to_GPS_path == false and drone.image_control_x_coo != 0.0 and drone.image_control_y_coo != 0.0 and cartesian_distance_err > 1.5)
                          {
                            if (drone.image_control_x_coo != 0.0 and drone.image_control_y_coo != 0.0 )
                            {
                              //THERMAL CONTROL
                              drone.switch_control = 1;
                            }
                            else if (drone.image_control_x_coo == 0.0 and drone.image_control_y_coo == 0.0 and drone.image_RGB_control_x_coo != 0.0 and drone.image_RGB_control_y_coo != 0.0)
                            {
                              //RGB CONTROL
                              drone.switch_control = 2;
                            }
                            
                          }
                        }
                        else
                        {
                        //GPS CONTROL
                         x_target = drone.control_x_coo; 
                         y_target = drone.control_y_coo; ;
                         y_target_vision = y_target;
                         ROS_INFO("GPS CONTROL");
                        }
                         */
                       break;
                
                }
                
                
                /*
                if (drone.image_control_x_coo == 0.0 and drone.image_control_y_coo == 0.0 or cartesian_distance_err < 1.5 or abs(error_from_GPS_line) > 0.6 or init_panel == true or coming_back_to_GPS_path == true)
                {
                     evaluate_control_point(x_target_P1, y_target_P1, x_target_P2, y_target_P2, target_point);
                       //Output funzione di controllo, corrisponde alle coordinate del punto di controllo D a cui tende il vettore V, dato dalla somma di Vx e Vy
                      x_target = drone.control_x_coo; 
                      y_target = drone.control_y_coo; ;
                      y_target_vision = y_target;
                      ROS_INFO("GPS GUIDANCE");
                      
                      //Se sono nel caso relativo alla cartesian distance vuol dire che il drone ha appena raggiunto il termine della vela, 
                      //in questo caso deve autopilotarsi con GPS fino a che non riconosce qualcosa dalla visione 
                      if ( cartesian_distance_err < 1.5)
                      {
                         //Inserisco flag nel quale utilizza GPS route finche 
                         //non riconosce qualcosa dalle immagini 
                         init_panel = true;
                      }
                      
                      
                      /* VERIFICO DISTANZA DA GPS PATH
                       * SE >1.5 VIENE CORRETTA LA TRAIETTORIA FINCHE NON TORNA AD UN VALORE MINORE DI 0.4
                       */
                       /*
                      if (abs(error_from_GPS_line) > 0.7)
                      {
                          coming_back_to_GPS_path = true;
                      }
                      if (abs(error_from_GPS_line) < 0.15)
                      {
                          coming_back_to_GPS_path = false;
                      }
                }
                else
                {
                    //Altrimenti mi baso su coordinate nel body frame ricavate dall'immagine 
                     x_target = abs(drone.image_control_x_coo); 
                     y_target = drone.image_control_y_coo; 
                     ROS_INFO("VISION GUIDANCE"); 
                    
                }
               */
           /*     
            cout<<"x_target_P1: "<<x_target_P1<<endl;
            cout<<"y_target_P1: "<<y_target_P1<<endl;
            cout<<"x_target_P2: "<<x_target_P2<<endl;
            cout<<"y_target_P2: "<<y_target_P2<<endl;
            */
            
            
            //Evaluate cartesian distance between final and setpoints
            cartesian_distance_err = sqrt(pow(panel.xcoo_panel_world_frame[count] - drone.drone_x,2) + pow(panel.ycoo_panel_world_frame[count]- drone.drone_y,2));
             
              //yaw_des = atan2(panel.ycoo_panel_world_frame[count],panel.xcoo_panel_world_frame[count]);
             //Attitude
             drone_vel_msg.angular.z  = pid_yaw.position_control_knowing_velocity(yaw_des, abs(drone.drone_Yaw), 0, drone.drone_ang_vel_z); //ci andrebbe yaw des
             if (drone.drone_Yaw < 0)
             {
                 drone_vel_msg.angular.z = -1* drone_vel_msg.angular.z;
             }
             yaw_err = yaw_des- drone.drone_Yaw;
            
              cout<<"drone.drone_Yaw: "<< drone.drone_Yaw <<"\n";
             //Rotation_GF_to_BF_des_pos(panel_vec[count].P2_x, panel_vec[count].P2_y , drone.drone_Yaw);
            //Rotation_GF_to_BF_des_pos(panel.xcoo_panel_world_frame[count] , panel.ycoo_panel_world_frame[count],  drone.drone_Yaw);
            
            //Rotation_GF_to_BF_des_pos(panel.xcoo_panel_world_frame[count], panel.ycoo_panel_world_frame[count],  drone.drone_Yaw);
            Rotation_GF_to_BF_drone_pos(drone.drone_Yaw);

             // PID control on velocity defined on body frame
            
             drone_vel_msg.linear.x = pid_x.calculate(x_target, drone.drone_x_b);
             drone_vel_msg.linear.y = pid_y.calculate(y_target, drone.drone_y_b);
             
             //cout<< "count_hovering: "<< count_hovering << endl;
             if (count_hovering > 10000)
             {
                 drone_vel_msg.linear.x = 0;
                 drone_vel_msg.linear.y = 0;
             }
              
              //cout << "[MAIN]:  Reaching end point Panel "<< count +1 << "distance --> " <<  cartesian_distance_err<< endl;
              
              //If Setpoint is reached, passing to the next setpoint
              
              /// RAGGIUNTO POINT P2 AL TERMINE DEL PANNELLO
              if (cartesian_distance_err < 0.3)
              {
                  //******When arrives at end Point 2, evaluate position of the next Point P1 on the other Panel************//
                  //Find relative angle of the GPS
                  float alfa = 0.0;
                  alfa = evaluate_relative_angle(count); //rad  --> ok
                  cout<<"alfa: "<< alfa * 180/M_PI<<endl;
                  
                  
                  //Evaluate next start point in other panel knowing the GPS waypoints and drone last position 
                  find_coo_start_point_next_panel_drone_body_frame(yaw_des, count, alfa); //Angle yaw when aligned with panels
                  
                  /*******************************************************/
                  
                 
                  if (count == (2*n_panels -1))
                  {
                      count = (2*n_panels -1);
                      flag_end_point = true;
                  }
                  else 
                  {
                      count = count + 1;
                      flag_end_point = false;
                      panel.target_point = 1;
                    //Reaching Point 2
                    //yaw_des = atan2(panel.ycoo_panel_world_frame[count],panel.xcoo_panel_world_frame[count]);
                     yaw_des = yaw_des - M_PI;
                    //check angle
                    if (yaw_des < -M_PI)
                    {
                        //yaw_des = atan2(panel.ycoo_panel_world_frame[count],panel.xcoo_panel_world_frame[count]);
                        yaw_des = yaw_des + 2*M_PI;
                    }
                    //Point to GPS P1 point --> then attrack the real P1 point at the start of the panel
                    gps_waypoint = true;
                    
                    //Reset Variables
                  x_target = 0;
                  y_target = 0; 
                  
                  //Necessario per aggiornare il filtro ad ogni cambio di vela con il nuovo valore dello stato 
                  //stimato piu simile a quello vero --> nella relata verrà inizializzato con i parametri della 
                  //nuova retta che passa tra i waypoints GPS della vela sucessiva
                  
                  KF.K_count = 0;
                  KF_RGB.K_count = 0;
                  }
              }
        }

        
        
        
        
        outFile4 << drone_vel_msg.linear.x << "\n";
        outFile5 << drone_vel_msg.linear.y << "\n";
        outFile6 << drone_vel_msg.linear.z << "\n";
        outFile7 << drone.drone_lin_vel_x << "\n";
        outFile8 << drone.drone_lin_vel_y << "\n";
        outFile9 << drone.drone_lin_vel_z << "\n";
        outFile10 <<drone.last_x << "\n";
        outFile11 << drone.last_y << "\n";
        outFile13<<y_target_vision<<"\n";
        outFile14<<drone.drone_roll<<"\n";
       
        outFile15<<KF.X[0]<<"\n";
        outFile16<<KF.X[1]<<"\n";
        outFile17<<KF.xh_[0]<<"\n";
        outFile18<<KF.xh_[1]<<"\n";
        outFile19<< x_target <<"\n";
        outFile20<< y_target <<"\n";
        outFile21<< KF.y[0] <<"\n";
        outFile22<< KF.y[1] <<"\n";
        outFile23 << KF.stdev << "\n";
        
        //Outfile for KF RGB filter
        outFile24<<KF_RGB.xh_[0]<<"\n";
        outFile25<<KF_RGB.xh_[1]<<"\n";
        outFile26<< KF_RGB.stdev << "\n";
        outFile27<< KF_RGB.y[0] <<"\n";
        outFile28<< KF_RGB.y[1] <<"\n";
        
        outFile29<< KF_RGB.eigen_Px[0] << ","<<" " << KF_RGB.eigen_Px[1] << "\n";
        outFile30<< KF.eigen_Px[0] << ","<<" " <<   KF.eigen_Px[1] << "\n";
        
        //Save Drone Positions 
        outFile31<< drone.drone_x <<"\n";
        outFile32<< drone.drone_y << "\n";
        outFile33<< drone.drone_z  << "\n";
        //Save Error with GPS (real) line of the apnels
        outFile34<< error_from_GPS_line << "\n";
        outFile35<< y_error_with_vision_line <<"\n";
        
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
        bottom_camera_pose_RGB.pose.position.z = drone.drone_z+ 0.25;
        
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
        
        // publish velocity message
        
        vel.publish(drone_vel_msg);

        flagDroneOdom = false;
        flagDroneImu = false;
        flagDroneFix_vel = false;
        flagDroneThermoControlPoint1 = false;
        flagDroneThermoControlPoint2  = false;
        flagDroneThermoControlPoint21  = false;
        flagDroneThermoControlPoint22  = false;
        flagDroneRGBControlPoint1  = false; 
        flagDroneRGBControlPoint2  = false;
      
        // x_ground_old = x_ground;
        // y_ground_old = y_ground;
        // yaw_ground_old = Mobile_orientation_theta;
        
        //Re inizializzo flag relativi l'update del filtro di Kalman 
       
        KF_RGB.RGB_update = false;
       
        time = time + dt;
        count_hovering = count_hovering + 1;
        ros::spinOnce();
        r.sleep();
    }
    
}




 
