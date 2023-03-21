#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <fstream>
#include "/home/lucamora/catkin_ws/src/solar_panel_project/include/solar_panel_project/pid.h"



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
// Solar Panel Position and Size

struct Panel {
    float x_center;
    float y_center;
    float P1_x;
    float P1_y;
    float P2_x;
    float P2_y;
    float length;
    float size;
    float theta;
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
    
    //Control point D coordinates
    float control_x_coo = 0.0;
    float control_y_coo = 0.0;
};



Drone drone;


bool flagDroneOdom = false;
bool flagDroneImu = false;
bool flagDroneFix_vel = false;

//Desired Position in Drone body frame
float check_x_b = 0.0; //Checkpoint coordinate in drone body frame
float check_y_b = 0.0;

//Desired position from body frame to world frame
float check_x_w = 0.0;
float check_y_w = 0.0;

nav_msgs::Odometry drone_odom;
sensor_msgs::Imu drone_imu;
geometry_msgs::Vector3Stamped drone_fix_vel;
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
     cout<<"theta: "<< theta<< endl;
    
    float waypoints_vector_in_gps_frame[3] = {waypoints.waypoints_x_coo_gps_frame[size_v - 1],waypoints.waypoints_y_coo_gps_frame[size_v - 1], 1}; //Definisco vettore waypoint in gps_frame da moltiplicare per matrice T
    for (int j = 0; j < 3; j++)
        {
            cout<<"waypoints_vector_in_gps_frame: "<< waypoints_vector_in_gps_frame[j] << endl;
            
        }
    float coo_w[3] = {0.0,0.0,0.0};
    
    
    //Transformation matrix from gps frame to world frame 
    //Initialize matrix  ---> considero la coordinata gps nel world frame del punto precedente (nella parte di traslazione) al fine di calcolare la sucessiva ---> guardare note ipad
    // gamma è un piccolo errore di rotazione, l'errore di tralsazione è aggiunto nel frame gps
    float  T[3][3] = {{cos(theta + waypoints.gamma), -sin(theta + waypoints.gamma), waypoints.waypoints_x_coo_world_frame[0]}, {sin(theta + waypoints.gamma), cos(theta + waypoints.gamma), waypoints.waypoints_y_coo_world_frame[0]}, {0, 0,1}};
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
    
    //Moltiplico la matrice di trasformazione con il vettore [Gps_x_coo, Gps_y_coo, 1]^(gps_frame). --> sono le coordinate che ottengo nel vettore waypoints.waypoints_x_coo_gps_frame
    //waypoints_x_coo_world_frame.
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            coo_w[i] += T[i][j] * waypoints_vector_in_gps_frame[j];
            
        }
        cout << "coo_w " << coo_w[i] <<endl;
    }
    
    
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
void transformation_panel_center_from_body_to_world(Panel *panel)
{
    //Transformation panel centers from body frame (panel body frame) to World frame
    //The angle theta of panel orientation is set in main 
    int size_v = panel -> xcenter_panel_frame.size();
    
    float panel_center_vector_in_panel_frame[3] = {panel -> xcenter_panel_frame[size_v - 1],panel -> ycenter_panel_frame[size_v - 1], 1}; //Definisco vettore waypoint in gps_frame da moltiplicare per matrice T

    
    float  T[3][3] = {{cos(panel -> theta), -sin(panel -> theta), panel->xcenter_panel_world_frame[0]}, {sin(panel -> theta), cos(panel -> theta), panel->ycenter_panel_world_frame[0]}, {0, 0,1}};
    int rows =  sizeof T / sizeof T[0]; // 3 rows  
    int cols = sizeof T[0] / sizeof(int); // 3 cols
    
    float coo_w[3] = {0.0,0.0,0.0};
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            coo_w[i] += T[i][j] * panel_center_vector_in_panel_frame[j];
            
        }
       
    }
    
    cout << "Panel center x in world frame:  " << coo_w[0] <<endl;
    cout << "Panel center y in world frame:  " << coo_w[1] <<endl;
    
    panel->xcenter_panel_world_frame.push_back(coo_w[0]);
    panel->ycenter_panel_world_frame.push_back(coo_w[1]);
    
}

//Place panel centers:
void  place_panel_centers(Panel *panel, int n_panels, int distance_between_panels)
{
    //Define centers in panel frame coordinates --> i want panel align algong the same x coo and different y coo
    for (int i = 0; i < n_panels; i++)
    {
        panel -> xcenter_panel_frame.push_back(0.0); 
        panel -> ycenter_panel_frame.push_back(- i * distance_between_panels); //Panels aligned along axe y in negative direction
        //cout<< "x center panel "<< i + 1 <<"frame : " << panel -> ycenter_panel_frame[i] << endl;
        //cout<< "y center panel "<< i + 1 <<"frame : " << panel -> ycenter_panel_frame[i] << endl;
        
        //Trasformation from panel frame to world frame.. The center of panel 1 in world frame is located in main 
        transformation_panel_center_from_body_to_world(panel);
    }
    
}


//Transformation from panel frame to world frame
void transformation_point_from_panel_to_world_frame(Panel *panel)
{
    //Transformation panel centers from body frame (panel body frame) to World frame
    //The angle theta of panel orientation is set in main 
    int size_v = panel -> xcoo_panel_frame.size();
    //cout<<"size Panel P x vec: "<<size_v<<endl; 
    float panel_P_xcoo_vector_in_panel_frame[3] = {panel -> xcoo_panel_frame[size_v - 1],panel -> ycoo_panel_frame[size_v - 1], 1}; //Definisco vettore waypoint in gps_frame da moltiplicare per matrice T

    
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
    
    
    panel->xcoo_panel_world_frame.push_back(coo_w[0]);
    panel->ycoo_panel_world_frame.push_back(coo_w[1]);
    //cout << "xcoo_panel_world_frame:  " << panel->xcoo_panel_world_frame[panel->xcoo_panel_world_frame.size() - 1] <<endl;
    //cout << "ycoo_panel_world_frame:  " << panel->ycoo_panel_world_frame[panel->ycoo_panel_world_frame.size() - 1] <<endl;
}

//Trasformation GPS from GPS to world frame 
void transformation_GPS_point_from_GPS_to_world_frame(Panel *panel)
{
  //Transformation panel centers from body frame (panel body frame) to World frame
    //The angle theta of panel orientation is set in main 
    int size_v = waypoints.waypoints_x_coo_gps_frame.size();
    
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


    
    //    float  T[3][3] = {{cos(panel -> theta + waypoints.gamma), -sin(panel -> theta + waypoints.gamma), panel->xcenter_panel_world_frame[0] + waypoints.delta}, {sin(panel -> theta + waypoints.gamma), cos(panel -> theta + waypoints.gamma), panel->ycenter_panel_world_frame[0] + waypoints.eta}, {0, 0,1}};

    //Seconda trasformazione da frame Panel (dove ho riportato i punti del GPS) al frame world
    float  T1[3][3] = {{cos(panel -> theta), -sin(panel -> theta), panel->xcenter_panel_world_frame[0]}, {sin(panel -> theta), cos(panel -> theta), panel->ycenter_panel_world_frame[0]}, {0, 0,1}};

    float coo_w1[3] = {0.0,0.0,0.0};
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            coo_w1[i] += T1[i][j] * coo_w[j];
            
        }
    }
    
    waypoints.waypoints_x_coo_world_frame.push_back(coo_w1[0]);
    waypoints.waypoints_y_coo_world_frame.push_back(coo_w1[1]);
    cout << "waypoints_x_coo_world_frame:  " << waypoints.waypoints_x_coo_world_frame[waypoints.waypoints_x_coo_world_frame.size() - 1] <<endl;
    cout << "waypoints_y_coo_world_frame:  " << waypoints.waypoints_y_coo_world_frame[waypoints.waypoints_y_coo_world_frame.size() - 1] <<endl;  
}



//Place Point P1 and P2 
void place_panel_start_end_point(Panel *panel, int n_panels, int distance_between_panels, int configuration)
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
        bool flag_panel_disposition_1 = true;   
   
                                                    
    
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
//Evaluate control point on line r passing throug P1 and P2
void evaluate_control_point(float x_target_P1, float y_target_P1, float x_target_P2, float y_target_P2)
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
    
    cout<< "x_target_P1_body: "<< x_target_P1_body <<endl;
     cout<< "y_target_P1_body: "<< y_target_P1_body <<endl;
     cout<< "x_target_P2_body: "<< x_target_P2_body <<endl;
     cout<< "y_target_P2_body: "<< y_target_P2_body <<endl;
     
   //Evaluate parameters a, b c of the line equation ax^b + by^ + c = 0
     a =  ((y_target_P2_body  -  y_target_P1_body)/(x_target_P2_body - x_target_P1_body));
     c = ((-(a) * x_target_P1_body) + y_target_P1_body);
     cout<< "a: "<< a <<endl;
     cout<< "c: "<< c <<endl;
     
     
     
     //Find distance point line: point is the body Origin and line is the line r
     double e = (abs(c)/(sqrt(pow(a,2) + pow(b,2))));
     cout<< "distace e: "<< e<<endl;
     
     //Define vector Vx starting from body frame origin and parallel to the line r
     double Vx[2] = {1/a, -1/b};
     double Kx = 1.0; //Coefficiente moltiplicativo del vettore parallelo alla retta r
     double Vx_norm[2] = {Kx * ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[0]), Kx* ((1/(sqrt(pow(Vx[0],2) + pow(Vx[1],2)))) * Vx[1])};
     
     //Define vector Vy starting from body frame origin and perpendicular to line r
     //Il vettore Vy è moltiplicato pe run guadagno Ky e anche per l'errore dato dalla distanza punto retta e, la quale deve tendere a zero
     float Ky = 1.0; //Coefficiente moltiplicativo del vettore perp alla retta r
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
     cout<< "Vy_norm x: "<< Vy_norm[0] <<endl;
     cout<< "Vy_norm y: "<< Vy_norm[1]  <<endl;
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
     cout<< "Vx_norm x: "<< Vx_norm[0] <<endl;
     cout<< "Vx_norm y: "<< Vx_norm[1] <<endl;
     
     
     //Find equation parameters of the line r1 passing trhoug vector Vx. Line r1 is parallel to line r but passing throung origin --> ha los tessto coefficiente angolare a della retta r
     float a1 = a;
     float b1 = -1;
     float c1 = 0;
     
     //Find equation parameters of the line r2 passing trhoug vector Vy. Line r2 is perpendicular  to line r but passing throung origin --> coeff angolare è 
     float a2 = Vy_norm[1]/Vy_norm[0];
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
     if  (D[0] < 0)
     {
         D[0] = -1* D[0];
     }
     
     cout<< "D x: "<< D[0] <<endl;
     cout<< "D y: "<< D[1] <<endl;
     
     drone.control_x_coo = D[0];
     drone.control_y_coo = D[1];
}






int main(int argc, char** argv)
{
    ros::init(argc, argv, "Altitude_controller");

    ros::NodeHandle nh;
    
    bool Take_off = false;
    bool flag_even = false;
    bool flag_end_point = false;

    int n_panels = 4;
    float distance_between_panels = 4;
    int configuration = 1;  
     
    
    
    //Create a vectore of struceture for each panel
    //vector<Panel> panel_vec;
    Panel panel;
    
    //panel_vec.push_back(Panel());
    
    /*
    panel_vec[0].x_center = 4.5;
    panel_vec[0].y_center = 7;
    panel_vec[0].theta = 0; //degree
    panel_vec[0].size = 2;
    panel_vec[0].length = 10;
   */
    panel.x_center = 5;
    panel.y_center = 0;
    panel.theta = 45.0 * M_PI/180;
    panel.size = 2;
    panel.length = 10;
    
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
    std::ofstream outFile("panel_x_point_in_world_frame.txt");
    std::ofstream outFile1("panel_y_point_in_world_frame.txt");
    for (int i = 0; i < panel.xcoo_panel_world_frame.size(); i++)
    {
        outFile << panel.xcoo_panel_world_frame[i] << "\n";
        outFile1 << panel.ycoo_panel_world_frame[i] << "\n";
    }
    
    //Write GPS real point in a file
    std::ofstream outFile2("GPS_x_point_in_world_frame.txt");
    std::ofstream outFile3("GPS_y_point_in_world_frame.txt");
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
    float Kp_x = 0.40;
    float Kp_y = 0.45;
    float Kd_x = 0.01;
    float Kd_y = 0.02;
    float Ki_x = 0.15;//0.9; //0.1
    float Ki_y = 0.1;//0.55; //0.1
    
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
    ros::Subscriber odom_drone_sub = nh.subscribe("/ground_truth/state", 5, drone_odom_callback);
    ros::Subscriber imu_drone_sub = nh.subscribe("/ardrone/imu", 5, drone_Imu_callback);
    ros::Subscriber vel_drone_sub = nh.subscribe("/fix_velocity", 1, drone_fix_Vel_callback);

    

    ros::Rate r(5);
    double secs =ros::Time::now().toSec();
    
    //Counter 
    int count = 0;
    int count_setpoint = 0;
    bool gps_waypoint = false;
    bool flag_create_setpoint = true;
    
    //Target : GPS o P ogni inizio pannello
    float x_target_P1 = 0.0;
    float y_target_P1 = 0.0; 
    float x_target_P2 = 0.0;
    float y_target_P2 = 0.0; 
    
    float x_target = 0.0;
    float y_target = 0.0;
    
    // Write Desired and real velocity
    
        std::ofstream outFile4("des_x_vel.txt");
        std::ofstream outFile5("des_y_vel.txt");
        std::ofstream outFile6("des_z_vel.txt");
        std::ofstream outFile7("x_vel.txt");
        std::ofstream outFile8("y_vel.txt");
        std::ofstream outFile9("z_vel.txt");
        std::ofstream outFile10("drone_x_pos_saved.txt");
        std::ofstream outFile11("drone_y_pos_saved.txt");
    
    while(nh.ok()) {
        geometry_msgs::Twist drone_vel_msg;
        
        
        if(Take_off == false) {
            takeOff.publish(myMsg);
            drone.desired_z = 2;
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
                //Alla prima iterazione conosco le coordinate di P1 e P2 nel frame world 
                evaluate_control_point(x_target_P1, y_target_P1, x_target_P2, y_target_P2);
                
                x_target = panel.xcoo_panel_world_frame[count];
                y_target = panel.ycoo_panel_world_frame[count];

                cout << "[START MISSION]--> Reaching P1 panel "<< count +1 << " --> distance --> " <<  cartesian_distance_err<< endl;
            }
            else 
            {
                //Le coordinate di target sono le coordinate P1 del nuovo pannello, simulo l'aggancio al pannello stesso
                
                x_target =  drone.next_point_P1_x[drone.next_point_P1_x.size() -1];
                y_target = drone.next_point_P1_y[drone.next_point_P1_y.size() -1];
                cout << "[NEXT PANEL] --> Reaching next point P1 evaluated from GPS "<< count +1 << "distance --> " <<  cartesian_distance_err<< endl;
            }
                 
            

             cartesian_distance_err = sqrt(pow(x_target- drone.drone_x,2) + pow(y_target- drone.drone_y,2));
             //Attitude
             
             drone_vel_msg.angular.z  = pid_yaw.position_control_knowing_velocity(yaw_des, drone.drone_Yaw, 0, drone.drone_ang_vel_z);
             yaw_err = yaw_des - drone.drone_Yaw;
             
             
             cout << "yaw_des: "<< yaw_des << endl;
             if (abs(yaw_err) < 0.2 || count == 0)
             {
                 //Reaching Point Start P1
                
                 //Rotate positions desired position in body frame
                 
                 Rotation_GF_to_BF_des_pos(x_target , y_target,  drone.drone_Yaw);
                 
                 Rotation_GF_to_BF_drone_pos(drone.drone_Yaw);
                 cout<<"drone.drone_Yaw: "<<drone.drone_Yaw<<endl;
                 
                 //PID control on velocity defined on body frame
                
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
        
              if (cartesian_distance_err < 0.2)
              {
                
                flag_end_point = true; //Mi dirigo lungo il pannello verso punto P2 di end --> passo all'else sottostante 
                flag_create_setpoint = true;
                if (count == 0)
                {
                    yaw_des = panel.theta;
                }
                count = count + 1; //--> Raggiungo punto P2 stesso pannello
              }
                  
                 //Se gps_waypoint = true
        }
        else 
        {
            
                x_target_P1 =  panel.xcoo_panel_world_frame[count- 1];
                y_target_P1 =  panel.ycoo_panel_world_frame[count - 1];
                
                x_target_P2 =  panel.xcoo_panel_world_frame[count];
                y_target_P2 =  panel.ycoo_panel_world_frame[count];
                
                //Alla prima iterazione conosco le coordinate di P1 e P2 nel frame world 
                evaluate_control_point(x_target_P1, y_target_P1, x_target_P2, y_target_P2);
                
                //Output funzione di controllo, corrisponde alle coordinate del punto di controllo D a cui tende il vettore V, dato dalla somma di Vx e Vy
                x_target = drone.control_x_coo; 
                y_target = drone.control_y_coo; 
                
                
            
            //Evaluate cartesian distance between final and setpoints
            cartesian_distance_err = sqrt(pow(panel.xcoo_panel_world_frame[count] - drone.drone_x,2) + pow(panel.ycoo_panel_world_frame[count]- drone.drone_y,2));
             
              //yaw_des = atan2(panel.ycoo_panel_world_frame[count],panel.xcoo_panel_world_frame[count]);
             //Attitude
             drone_vel_msg.angular.z  = pid_yaw.position_control_knowing_velocity(yaw_des, drone.drone_Yaw, 0, drone.drone_ang_vel_z);
             yaw_err = yaw_des - drone.drone_Yaw;
            

             //Rotation_GF_to_BF_des_pos(panel_vec[count].P2_x, panel_vec[count].P2_y , drone.drone_Yaw);
            //Rotation_GF_to_BF_des_pos(panel.xcoo_panel_world_frame[count] , panel.ycoo_panel_world_frame[count],  drone.drone_Yaw);
            
            //Rotation_GF_to_BF_des_pos(panel.xcoo_panel_world_frame[count], panel.ycoo_panel_world_frame[count],  drone.drone_Yaw);
            Rotation_GF_to_BF_drone_pos(drone.drone_Yaw);

             // PID control on velocity defined on body frame
            
             drone_vel_msg.linear.x = pid_x.calculate(x_target, drone.drone_x_b);
             drone_vel_msg.linear.y = pid_y.calculate(y_target, drone.drone_y_b);

              cout << "Reaching end point Panel "<< count +1 << "distance --> " <<  cartesian_distance_err<< endl;
              
              //If Setpoint is reached, passing to the next setpoint
              
              
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
        

        

        // publish the message
        vel.publish(drone_vel_msg);

        flagDroneOdom = false;
        flagDroneImu = false;
        flagDroneFix_vel = false;

        // x_ground_old = x_ground;
        // y_ground_old = y_ground;
        // yaw_ground_old = Mobile_orientation_theta;
       
        time = time + dt;
        ros::spinOnce();
        r.sleep();
    }
     
}




 
