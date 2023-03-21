#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
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
#include "gazebo_msgs/SetModelState.h"
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
//#include "image_converter.h"

#include "/home/lucamora/catkin_ws/src/solar_panel_project/include/solar_panel_project/waypoints.h"
#include "/home/lucamora/catkin_ws/src/solar_panel_project/include/solar_panel_project/Panel.h"




using namespace std;
#define N 2 

// Solar Panel Position and Size
unsigned int sleep(unsigned int seconds);

std::ofstream outFile("/home/lucamora/catkin_ws/simulation_data/sim_data_complete/y_offset.txt");
std::ofstream outFile1("simulation_data/sim_data_complete/Y_camera.txt");
std::ofstream outFile2("/home/lucamora/catkin_ws/simulation_data/sim_data_complete/Y_matrice_DJI_WORLD.txt");
std::ofstream outFile3("/home/lucamora/catkin_ws/simulation_data/sim_data_complete/Y_UAV_GAZEBO.txt");

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
  
  float start_position_x;
  float start_position_y;

  float drone_x;
  float drone_y;
  float drone_z;
  float drone_Yaw;
  float yaw_in_Rad_DJI;
 
  float MATRICE_local_position_x = 0.0;
  float MATRICE_local_position_y = 0.0;
  float MATRICE_local_position_z = 0.0;

  bool flagDroneMatricePointD = false;
  bool flagMatriceLocalPos = false;
  bool flagDroneAttitude = false;

} drone;



struct Mission
{
  int state = 0;
  
 
 
  vector<float> P1_target;
  vector<float> P2_target;
  //Eigen::Vector2f Target;

  //Target Vision Point in BF from MAtrice
  geometry_msgs::Point target_from_MATRICE;
  geometry_msgs::Point target_from_MATRICE_old;

  float x_target = 0.0;
  float y_target = 0.0;
  int target_point = 0.0;
  float cartesian_distance_err = 0.0;
  float offset_y_error = 0.0;


  int count = 0;
  int navigation_iteration_count = 0;
  
  bool start_task = false;
  bool GAZEBO_LINK = false;
 
} mission;


int stop_navigation_counter = 0;
float check_x_w = 0.0;
float check_y_w = 0.0;
float check_x_b = 0.0;
float check_y_b = 0.0;

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 ans;
    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    return ans;
}


void Matrice_point_D_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point take_MATRICE_points;
    take_MATRICE_points = *msg;
    mission.target_from_MATRICE.x = take_MATRICE_points.x;
    mission.target_from_MATRICE.y = take_MATRICE_points.y;
    mission.target_from_MATRICE.z = take_MATRICE_points.z;   //offset .. only valid when dropme take images from the ground
    drone.flagDroneMatricePointD = true;
}

void DJI_local_pos_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::Point local_position;
    local_position = *msg;
    drone.MATRICE_local_position_x = local_position.x;;
    drone.MATRICE_local_position_y = local_position.y;
    drone.MATRICE_local_position_z = local_position.z;
    drone.flagMatriceLocalPos = true;
}


void drone_attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    geometry_msgs::Quaternion current_atti;
    current_atti = msg->quaternion;
    drone.yaw_in_Rad_DJI = toEulerAngle(current_atti).z;
    drone.flagDroneAttitude = true;
}



void absolute_pos()
{
    drone.drone_x = 0.5*drone.MATRICE_local_position_x;
    drone.drone_y = drone.MATRICE_local_position_y;
    drone.drone_z = drone.MATRICE_local_position_z;
    drone.drone_Yaw = 0.0;


}


void Rotation_BF_to_GF_des_pos(float x_pos, float y_pos, float alfa)
{
    check_x_w = x_pos * cos(alfa) - y_pos * sin(alfa) + drone.drone_x;
    check_y_w = x_pos * sin(alfa) + y_pos * cos(alfa) + drone.drone_y;
}

void Rotation_GF_to_BF_des_pos(float x_pos, float y_pos, float alfa)
{
     check_x_b = x_pos*cos(alfa) + y_pos * sin(alfa) -cos(alfa)* drone.start_position_x - sin(alfa)* drone.start_position_y; //Checkpoint coordinate in drone body frame
     check_y_b = -x_pos*sin(alfa) + y_pos * cos(alfa) +sin(alfa)* drone.start_position_x- cos(alfa)*drone.start_position_y;
}

void evaluate_error_offset_from_true_line()
{
    float y_UAV = 0.0;
    if (drone.drone_y <= 0)
    {
            y_UAV = -1*drone.drone_y;
    }
    else
    {
      y_UAV = drone.drone_y;
    }
    mission.offset_y_error =  drone.start_position_y - y_UAV;
    cout<<"Offset Error: " << mission.offset_y_error << endl;

    //Write in TXT FILE
    outFile << mission.offset_y_error << "\n";
    outFile3 << y_UAV << endl;
}


float x_ref = 0.0;
float y_ref = 0.0;
float x_off = 0.0;
float y_off = 0.0;
void relative_pos()
{
     
    if (mission.target_from_MATRICE.x != 0 || mission.target_from_MATRICE.y != 0)
    {
        if (stop_navigation_counter == 0)
        {
           x_ref = drone.MATRICE_local_position_x;
           y_ref = drone.MATRICE_local_position_y;
        }
        x_off = drone.MATRICE_local_position_x - x_ref;
        y_off = drone.MATRICE_local_position_y + y_ref;

        drone.drone_x = abs(x_off);
        drone.drone_y = y_off + drone.start_position_y;
        stop_navigation_counter = stop_navigation_counter + 1;
    }
    else
    {
        drone.drone_x = drone.start_position_x;
        drone.drone_y =  drone.start_position_y;
        drone.drone_z = 5.0;
        drone.drone_Yaw = 0.0;

    }
    
    drone.drone_x = 0.5*drone.MATRICE_local_position_x + 1.0;
    drone.drone_y = 0.5*drone.MATRICE_local_position_y;
    drone.drone_z = drone.MATRICE_local_position_z;
    drone.drone_Yaw = 0.0;
    cout << "MATRICE_local_position.x: " << drone.MATRICE_local_position_x << endl;



}


void relative_pos_same_array()
{
    if (mission.target_from_MATRICE.x !=  mission.target_from_MATRICE_old.x  && mission.target_from_MATRICE.y !=  mission.target_from_MATRICE_old.y || mission.GAZEBO_LINK == true )
    {
        
        //Rotazione da frame locale i DJI a frame in Gazebo
        Rotation_GF_to_BF_des_pos(drone.MATRICE_local_position_x, drone.MATRICE_local_position_y, drone.yaw_in_Rad_DJI);
        drone.drone_x  = drone.start_position_x + 0.5*check_x_b;
       
        drone.drone_y = check_y_b ;
        drone.drone_z = 5.0 +  0.1 * mission.target_from_MATRICE.z;
        cout<< "drone.drone_z : " << drone.drone_z  << endl;
        cout << "drone.MATRICE_local_position_x: " << drone.MATRICE_local_position_x << " Y: " <<  drone.MATRICE_local_position_y << endl;
        cout << "X: " << drone.drone_x << " Y: " <<  drone.drone_y << endl;
        evaluate_error_offset_from_true_line();

        mission.GAZEBO_LINK = true;
        if (drone.drone_x > 30)
        {
            mission.GAZEBO_LINK = false;
        }
        outFile1 << check_y_b << "\n";
        outFile2 <<  drone.MATRICE_local_position_y << "\n";
       
    }
    else
    {
        // Rotation_GF_to_BF_des_pos(drone.MATRICE_local_position_x, drone.MATRICE_local_position_y, drone.yaw_in_Rad_DJI);
        // drone.drone_x  = 0.5*check_x_b + 1.0;
        // drone.drone_y = check_y_b;
        // drone.drone_z = 5.0 +  0.1 * mission.target_from_MATRICE.z;

        // if(drone.drone_x < drone.start_position_x)
        // {
        //     mission.start_task = false;
        // }
        // else
        // {
        //     mission.start_task = true;
        // }

        cout << "Waiting In Start Position Data Fron Matrice: X " << drone.drone_x << " Y: " <<  drone.drone_y << endl;
        drone.drone_x = drone.start_position_x;
        drone.drone_y =  drone.start_position_y;
        drone.drone_z = 5.0;
        drone.drone_Yaw = 0.0;
        
        cout << "Waiting In Start Position : X " << drone.start_position_x << " Y: " <<   drone.start_position_y << endl;
    }



}

//ENTRA QUA 

void relative_pos_parallel_arrays(bool direction)
{
    if (direction == 1)
    {
       if (mission.target_from_MATRICE.x !=  mission.target_from_MATRICE_old.x  && mission.target_from_MATRICE.y !=  mission.target_from_MATRICE_old.y || mission.GAZEBO_LINK == true)
       {
        
        //Rotazione da frame locale i DJI a frame in Gazebo
        Rotation_GF_to_BF_des_pos(drone.MATRICE_local_position_x, drone.MATRICE_local_position_y, drone.yaw_in_Rad_DJI);
        drone.drone_x  = drone.start_position_x + 0.35*check_x_b;
        drone.drone_y = check_y_b;
        drone.drone_z = 5.0 +  1.4* mission.target_from_MATRICE.z;
        cout<< "drone.drone_z : " << drone.drone_z  << endl;
        cout << "drone.MATRICE_local_position_x: " << drone.MATRICE_local_position_x << " Y: " <<  drone.MATRICE_local_position_y << endl;
        cout << "X: " << drone.drone_x << " Y: " <<  drone.drone_y << endl ;
        evaluate_error_offset_from_true_line();

        mission.GAZEBO_LINK = true;
        if (drone.drone_x > 30) //DIventa 1000 quando raggiunto il valore di fine del pannello
        {
            mission.GAZEBO_LINK = false;
        }

        if (drone.MATRICE_local_position_x > 100 || drone.MATRICE_local_position_y > 100 || drone.MATRICE_local_position_x < -100   || drone.MATRICE_local_position_y < -100 )
        {
            direction = 2;
            drone.start_position_x = 2.7;
            drone.start_position_y = -0.3;
            drone.drone_Yaw = 0.0;
            mission.GAZEBO_LINK = false;
        }
        }
        else
        {
            cout << "Waiting In Start Position Data Fron Matrice: X " << drone.drone_x << " Y: " <<  drone.drone_y << endl;
            drone.drone_x = drone.start_position_x;
            drone.drone_y =  drone.start_position_y;
            drone.drone_z = 5.0;
            drone.drone_Yaw = 0.0;
        }
    }
    else
    {
        if (mission.target_from_MATRICE.x !=  mission.target_from_MATRICE_old.x  && mission.target_from_MATRICE.y !=  mission.target_from_MATRICE_old.y || mission.GAZEBO_LINK == true)
        {
        
       
        //Rotazione da frame locale i DJI a frame in Gazebo
        Rotation_GF_to_BF_des_pos(drone.MATRICE_local_position_x, drone.MATRICE_local_position_y, drone.yaw_in_Rad_DJI);
        drone.drone_x  = 0.5*check_x_b + 1.0;
        drone.drone_y = check_y_b;
        drone.drone_z = 5.0 + 1.4*  mission.target_from_MATRICE.z;
        cout<< "drone.drone_z : " << drone.drone_z  << endl;
        cout << "drone.MATRICE_local_position_x: " << drone.MATRICE_local_position_x << " Y: " <<  drone.MATRICE_local_position_y << endl;
        cout << "X: " << drone.drone_x << " Y: " <<  drone.drone_y << endl;
        evaluate_error_offset_from_true_line();

        mission.GAZEBO_LINK = true;
        if (drone.drone_x > 30) //DIventa 1000 quando raggiunto il valore di fine del pannello
        {
            mission.GAZEBO_LINK = false;
        }

        if (drone.MATRICE_local_position_x > 500 || drone.MATRICE_local_position_y > 500 )
        {
            direction = 1;
            drone.start_position_x = 2.7;
            drone.start_position_y = 2.0;
            drone.drone_Yaw = 0.0;
            mission.GAZEBO_LINK = false;
        }
        }
        else
        {
            cout << "Waiting In Start Position Data Fron Matrice: X " << drone.drone_x << " Y: " <<  drone.drone_y << endl;
            drone.drone_x = drone.start_position_x;
            drone.drone_y =  drone.start_position_y;
            drone.drone_z = 5.0;
            drone.drone_Yaw = 0.0;
        }

    }
     
    
   
}


//Use this option with ardrone_matrice_testworld 
void camera_follows_Matrice_without_rotation()
{
    if (mission.target_from_MATRICE.x !=  mission.target_from_MATRICE_old.x  && mission.target_from_MATRICE.y !=  mission.target_from_MATRICE_old.y || mission.GAZEBO_LINK == true)
    {
        
        //Rotazione da frame locale i DJI a frame in Gazebo
        drone.drone_x  = 0.5*drone.MATRICE_local_position_x + 1.0;
        drone.drone_y = drone.MATRICE_local_position_y;
        drone.drone_z = 5.0;
        cout << "drone.MATRICE_local_position_x: " << drone.MATRICE_local_position_x << " Y: " <<  drone.MATRICE_local_position_y << endl;
        cout << "X: " << drone.drone_x << " Y: " <<  drone.drone_y << endl;
        evaluate_error_offset_from_true_line();

        mission.GAZEBO_LINK = true;
        if (drone.drone_x > 30)
        {
            mission.GAZEBO_LINK = false;
        }
       
    }
    else
    {
        cout << "Waiting In Start Position Data Fron Matrice: X " << drone.drone_x << " Y: " <<  drone.drone_y << endl;
        drone.drone_x = drone.MATRICE_local_position_x;
        drone.drone_y =  drone.MATRICE_local_position_y;
        drone.drone_z = 5.0;
        drone.drone_Yaw = 0.0;
       

    }

}

void read_record()
{
  
    // File pointer
    fstream fin;
  
    // Open an existing file
    fin.open("/home/lucamora/Desktop/result.csv", ios::in);
  
    // Get the roll number
    // of which the data is required
    int rollnum, roll2, count = 0;
    cout << "Enter the roll number "
         << "of the student to display details: ";
    cin >> rollnum;
  
    // Read the Data from the file
    // as String Vector
    vector<string> row;
    string line, word, temp;
  
    while (fin >> temp) {
  
        row.clear();
  
        // read an entire row and
        // store it in a string variable 'line'
        getline(fin, line);
  
        // used for breaking words
        stringstream s(line);
  
        // read every column data of a row and
        // store it in a string variable, 'word'
        while (getline(s, word, ', ')) {
  
            // add all the column data
            // of a row to a vector
            row.push_back(word);
        }
  
        // convert string to integer for comparision
        roll2 = stoi(row[0]);
  
        // Compare the roll number
        if (roll2 == rollnum) {
  
            // Print the found data
            count = 1;
            cout << "Details of Roll " << row[0] << " : \n";
            cout << "Name: " << row[1] << "\n";
            cout << "Maths: " << row[2] << "\n";
            cout << "Physics: " << row[3] << "\n";
            cout << "Chemistry: " << row[4] << "\n";
            cout << "Biology: " << row[5] << "\n";
            break;
        }
    }
    if (count == 0)
        cout << "Record not found\n";
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "Solar_fligth_control_sim");

    ros::NodeHandle nh;
     
    read_record();
      
    
  
  
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
