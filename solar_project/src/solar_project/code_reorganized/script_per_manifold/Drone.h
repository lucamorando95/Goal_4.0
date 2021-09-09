

#ifndef DRONE_H
#define DRONE_H



// ROS includes
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
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

class Drone
{
  ros::NodeHandle nh_;
  ros::Subscriber control_RGB_point_1;
  ros::Subscriber control_RGB_point_2;
  ros::Subscriber control_thermo_point_1;
  ros::Subscriber control_thermo_point_2;

  //For real flight
  ros::Subscriber attitudeSub;
  ros::Subscriber gpsSub;
  ros::Subscriber localPosition;

  ros::Publisher pub_P1_estimated;
  ros::Publisher pub_P2_estimated;

  // Publish the control signal
  ros::Publisher ctrlPosYawPub; 
 
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
  
  
  float yaw_in_Rad;
 
  sensor_msgs::NavSatFix current_gps;
  geometry_msgs::Point current_local_pos; //Posizione local ripsetto home frame 
  geometry_msgs::Point local_pos_I; //posizione rispetto frame fissato tra un target e il sucessivo
  //point lies on the estimated line expresse in body frame 
  Eigen::Vector2f P1_B;
  Eigen::Vector2f P2_B;

  bool flagDroneRGBControlPoint1 = false; 
  bool flagDroneRGBControlPoint2 = false;
  bool flagDroneThermoControlPoint1 = false;
  bool flagDroneThermoControlPoint2 = false;
  bool flagDroneAttitude = false;
  bool flagDroneGPSpos = false;

  Drone()
  {
 
    control_RGB_point_1 = nh_.subscribe("/RGB_control_point_1", 1, &Drone::drone_RGB_control_point1_callback, this);
    control_RGB_point_2 = nh_.subscribe("/RGB_control_point_2", 1, &Drone::drone_RGB_control_point2_callback, this);
    control_thermo_point_1 = nh_.subscribe("/Thermo_control_point_1", 1, &Drone::drone_thermo_control_point1_callback, this);
    control_thermo_point_2 = nh_.subscribe("/Thermo_control_point_2", 1, &Drone::drone_thermo_control_point2_callback, this);
    attitudeSub = nh_.subscribe("/dji_osdk_ros/attitude", 10, &Drone::drone_attitude_callback, this);
    gpsSub      = nh_.subscribe("dji_osdk_ros/gps_position", 10, &Drone::drone_gps_callback, this);
    localPosition = nh_.subscribe("dji_osdk_ros/local_position", 10, &Drone::drone_local_position_callback, this);

    pub_P1_estimated  = nh_.advertise<geometry_msgs::Point>("/P1_estimated_control_point",1);
    pub_P2_estimated =  nh_.advertise<geometry_msgs::Point>("/P2_estimated_control_point",1);

    ctrlPosYawPub = nh_.advertise<sensor_msgs::Joy>("dji_osdk_ros/flight_control_setpoint_ENUposition_yaw", 10);
  }

    
//Define Drone Class Desctructor
  ~Drone()
  {
  
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
  geometry_msgs::Quaternion current_atti;
  current_atti = msg->quaternion; //msg->quaternion;
  yaw_in_Rad = toEulerAngle(current_atti).z;
  flagDroneAttitude = true;
}

void drone_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  sensor_msgs::NavSatFix current_gps;
  current_gps = *msg;
  // gps_latitude = current_gps.latitude;
  // gps_longitude = current_gps.longitude;

  //Condivido variabile
  flagDroneGPSpos = true;
}

void drone_local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  geometry_msgs::Point current_local_pos;
  current_local_pos = msg->point;
  // drone_local_pos_x = current_local_pos.x;
  // drone_local_pos_y = current_local_pos.y;

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

};


#endif