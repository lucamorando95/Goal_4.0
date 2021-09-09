/** @file demo_flight_control.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef MISSION_H
#define MISSION_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/KF.h"
#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/pid.h"
#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/Drone.h"


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

using namespace std;
/*!
 * @brief a bare bone state machine to track the stage of the mission
 */
class Mission
{
public:
  // The basic state transition flow is:
  // 0---> 1 ---> 2 ---> ... ---> N ---> 0
  // where state 0 means the mission is note started
  // and each state i is for the process of moving to a target point.
 //Define variables present inside teh function that behave to class Mission
  int state;
  int enter_state; //used to use case ALIGN_YAW for every GPS waypoint
  
  int target_point = 0.0;
  int inbound_counter;
  int outbound_counter;
  int break_counter;
  
  //Distance done by the drone between actual and starting position
  float target_offset_x;
  float target_offset_y;
  float target_offset_z;
  float target_yaw;
   
  //GPS TARGET
  float target_GPS_offset_x;
  float target_GPS_offset_y;
  float target_GPS_offset_z;
  float target_GPS_yaw;
  

  //Kalamn Filter Initialization covariance 
  Eigen::Matrix2f R;
  Eigen::Matrix2f Px;

  
  //Remaining distance from the target.
  float x_offset_remaining;
  float y_offset_remaining;
  float z_offset_remaining;
  
  //Remaining distance from GS target 
  float x_GPS_offset_remaining;
  float y_GPS_offset_remaining;
  float z_GPS_offset_remaining;

 // Define x, y target for setpoint position during evaluate control--> following line 
  float line_setpoint_distance = 2.0;
  float x_target = 0.0;
  float y_target = 0.0;

  float  speedFactor = 2.0; //maximum velocity only in Position Control
  float  yawThresholdInDeg   = 2.0; //Puvbblica su Joy, non puo superare due il comando --> massima estensione joystick


  //Variable relative to Jump Panel case
  float x_target_P1 = 0.0;
  float y_target_P1 = 0.0; 

  float  x_target_P2 = 0.0;
  float y_target_P2 = 0.0;

  float x_target_P2_B = 0.0;
  float y_target_P2_B = 0.0;
  
  float x_target_P2_BF = 0.0;
  float y_target_P2_BF = 0.0;

  float previous_x_start = 0.0;
  float previous_y_start = 0.0;
  
  //Velocity saturation
  float sat_x_off = 0.0;
  float sat_y_off = 0.0;


  //Latitude of the drone in P1 
  float P1_drone_lat = 0.0;
  float P1_drone_lon = 0.0;
  float P2_lat = 0.0;
  float P2_lon = 0.0;
  
  //Fixed Waypoints distance between two waypoints 
  float fixed_waypoints_distance = 0.0;

  float error_from_vision_line = 0.0;
  float cartesian_distance_2D;
  float cartesian_distance_3D;
  float cartesian_distance_2D_old;
  
  float distance_to_target = 2.0;
  int distance_vector_index = 0;
  sensor_msgs::NavSatFix start_gps_location;
  geometry_msgs::Point start_local_position;
  // TAKE OFF HOME FRAME 
  sensor_msgs::NavSatFix HOME_gps_location;
  geometry_msgs::Point HOME_local_position;

  float referred_local_yaw;
  
  //desired reference altitude during navigation with laser sensor 
  float desired_laser_navigation_heigth = 15;
  
  int  counter_altitude_sensor = 0;
  //Accele ssaturation counter 
  int counter_sat_x_ddot_pos = 0;
  int counter_sat_x_ddot_neg = 0;
  int counter_sat_y_ddot_pos = 0;
  int counter_sat_y_ddot_neg = 0;

  int  breaking_counter  = 0.0;
  
  //Counter case 8
  int counter_case_8 = 0;
  int counter_case_8_exit_value = 0;
  
  int counter_RGB_detection = 0;

  bool finished;
  bool take_off_result;
  bool target_reached = false;
  bool panel_array_initialization = false; //--> permette al drone di muoversi un pochino lungo il pannello seguendo la gps line finche non viene detecteato qualcosa
  bool flag_navigate_to_waypoints = false;
  
  bool desired_yaw_flag = false;
  //flag relative to the use of the altitude respect the ground 
  bool flag_altitude_init = false;
  bool height_above_ground = false;
  //Flag if Observation from images are received 
  bool from_image = false;
  
  //Flag relative to enabling parameters optimization 
  bool Optimization_enabled = false;
  bool start_optimization_task = false;
  bool Optimization_completed = false;
  
//flag to control the altitude in navigatuon using the onboard laser sensor 
  bool enable_heigth_control_via_sensor = false;
  bool altitude_fixed_flag = false; //Permette di decidere se avere sempre un altitudeine costante oppure se seguire quela scelta dall'utente a inzio vela
//Enable Velocity Control 
  bool enable_navigation_velocity_control_flag = false;
//Flag to enable navigation only along a single array with the use of two fixed waypoints 
  bool  use_two_single_fixed_waypoints_flag = false;
  bool GAZEBO_LINK = true;
  bool end_panel_reached = false;

  bool inNavigation = false;


  
  Mission() : state(0), inbound_counter(0), outbound_counter(0), break_counter(0),
              target_offset_x(0.0), target_offset_y(0.0), target_offset_z(0.0),
              finished(false)
  {
  }

  void step();

  void setTarget(float x, float y, float z, float yaw)
  {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw      = yaw;
  }

  void setGPSTarget(float x, float y, float z, float yaw)
  {
    /*
    Set GPS WAYPOINT target place at the end of the panel array.
    Use it to have a security control if the drone fly away from the GPS line
    Use it to jump to the parallel array
    */

     target_GPS_offset_x  = x;
     target_GPS_offset_y = y;
     target_GPS_offset_z = z;
     target_GPS_yaw = yaw;
  }


  void reset()
  {
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    finished = false;
  }


  float compute_distance_derivative()
  {
    /*
    Compute Derivative 
    */
    float num = cartesian_distance_2D - cartesian_distance_2D_old;
    float delta_T = 0.05;
    
    float derivative = num/delta_T;
    return derivative;
  }

  void compute_path_difference(float x_travelled, float y_travelled,  geometry_msgs::Vector3& target_offset_respect_local_pos)
  {
   
    float distance_from_start_pos =  sqrt(pow( x_travelled,2) + pow(y_travelled,2));
    //float distance_from_start_to_GPS_waypoint =  sqrt(pow(target_GPS_offset_y- start_local_position.x,2) + pow( target_GPS_offset_x - start_local_position.y,2));
    float distance_from_start_to_GPS_waypoint =  sqrt(pow(target_offset_respect_local_pos.x- start_local_position.x,2) + pow( target_offset_respect_local_pos.y - start_local_position.y,2));
    float diff = distance_from_start_to_GPS_waypoint - distance_from_start_pos ;
    cout<<"distance_from_start_pos: " << distance_from_start_pos << endl;
    cout<<"distance_from_start_to_GPS_waypoint: " << distance_from_start_to_GPS_waypoint << endl;
    cout << "target_GPS_offset_x: " << target_offset_respect_local_pos.x << " target_GPS_offset_y: " << target_offset_respect_local_pos.y << endl;
    cout << "start_local_position.x: " << start_local_position.x << " start_local_position.y: " << start_local_position.y << endl;
    cout << "diff: " << diff << endl;
    
    if (diff < 4)
    {
      flag_navigate_to_waypoints = true;
    }
  }



 // Function to print the
// index of an element
void getIndex(vector<double> v, double K)
{
    auto it = find(v.begin(), v.end(), K);
 
    // If element was found
    if (it != v.end())
    {
     
        // calculating the index
        // of K
        distance_vector_index = it - v.begin();
        
    }
    else {
        // If the element is not
        // present in the vector
        cout << "-1" << endl;
    }
}


//Velocity Saturation given acceleration 
void velocity_saturation_given_acceleration(float x_ddot, float y_ddot)
{
  
    if (x_ddot > 2.5 || counter_sat_x_ddot_pos > 0)
    {
     
      sat_x_off = 0.2;
      sat_y_off = 0.2;
      if (counter_sat_x_ddot_pos > 100)
      {
       counter_sat_x_ddot_pos = 0;
      }
      else
      {
       counter_sat_x_ddot_pos =counter_sat_x_ddot_pos + 1;
      }
      cout << "[!!!! X VEL SATURATED]" << endl;
    }
    else
    {
      counter_sat_x_ddot_pos = 0;
    }
    
    if (x_ddot < -2.5 || counter_sat_x_ddot_neg > 0)
    {
      sat_x_off = 0.2;
      sat_y_off = 0.2;
      if (counter_sat_x_ddot_neg > 100)
      {
        counter_sat_x_ddot_neg = 0;
      }
      else
      {
        counter_sat_x_ddot_neg = counter_sat_x_ddot_neg + 1;
      }
       cout << "[!!!! X VEL SATURATED]" << endl;
    }
    else
    {
      counter_sat_x_ddot_neg = 0;
    }

    if (y_ddot > 2.5 || counter_sat_y_ddot_pos > 0)
    {
       sat_x_off = 0.2;
      sat_y_off = 0.2;

      if (counter_sat_y_ddot_pos > 100)
      {
        counter_sat_y_ddot_pos = 0;
      }
      else
      {
        counter_sat_y_ddot_pos = counter_sat_y_ddot_pos + 1;
      }
       cout << "[!!!! Y VEL SATURATED]" << endl;
    }
    else
    {
      counter_sat_y_ddot_pos = 0;
    }

     if (y_ddot < -2.5 || counter_sat_y_ddot_neg > 0)
    {
      sat_x_off = 0.2;
      sat_y_off = 0.2;
      if (counter_sat_y_ddot_neg > 100)
      {
        counter_sat_y_ddot_neg = 0;
      }
      else
      {
        counter_sat_y_ddot_neg = counter_sat_y_ddot_neg + 1;
      }
       cout << "[!!!! Y VEL SATURATED]" << endl;
    }
    else
    {
      counter_sat_y_ddot_neg = 0;
    }

}


};

//Declaration of the function which body is defined in file.cpp
void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin);

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

// void take_off(Drone *drone, PID *pid_x, PID *pid_y, PID *pid_z, PID *pid_yaw;

// void  align_Yaw_with_starting_GPS_waypoint(Drone *drone,PID *pid_z, PID *pid_yaw);


#endif // DEMO_FLIGHT_CONTROL_H
