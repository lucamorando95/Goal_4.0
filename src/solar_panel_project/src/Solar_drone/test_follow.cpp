#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <fstream>



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
    
};

struct Drone {
    float drone_x;
    float drone_y;
    float drone_z;
    float drone_x_b;
    float drone_y_b;
    float desired_z;
    double drone_ang_vel_z;

    double drone_Yaw;
    double drone_lin_vel_x;
    double drone_lin_vel_y;
    double drone_lin_vel_z;
    
    //Vel on body frame
    double drone_lin_vel_x_b;
    double drone_lin_vel_y_b;
};

Drone drone;


bool flagDroneOdom = false;
bool flagDroneImu = false;
bool flagDroneFix_vel = false;

//Desired Position in Drone body frame
float check_x_b = 0.0; //Checkpoint coordinate in drone body frame
float check_y_b = 0.0;
  

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

void Rotation_GF_to_BF_des_pos(float x_pos, float y_pos, float alfa)
{
     check_x_b = x_pos*cos(alfa) + y_pos * sin(alfa); //Checkpoint coordinate in drone body frame
     check_y_b = -x_pos*sin(alfa) + y_pos * cos(alfa);
}

void Rotation_GF_to_BF_drone_pos(float alfa)
{
    
     drone.drone_x_b = drone.drone_x*cos(alfa) + drone.drone_y * sin(alfa); //Checkpoint coordinate in drone body frame
     drone.drone_y_b = -drone.drone_x*sin(alfa) + drone.drone_y * cos(alfa);
     
     drone.drone_lin_vel_x_b = drone.drone_lin_vel_x *cos(alfa) + drone.drone_lin_vel_y * sin(alfa);
     drone.drone_lin_vel_y_b = -drone.drone_lin_vel_x*sin(alfa) + drone.drone_lin_vel_y* cos(alfa);
}

void Rotation_GF_to_BF_drone_vel(float velx, float vely, float alfa1)
{
    drone.drone_lin_vel_x_b = velx *cos(alfa1) + vely* sin(alfa1);
    drone.drone_lin_vel_y_b = -velx*sin(alfa1) + vely* cos(alfa1);
    saturation(drone.drone_lin_vel_x_b,drone.drone_lin_vel_y_b, 1.5, 1.5);
    
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "Altitude_controller");

    ros::NodeHandle nh;
    bool Take_off = false;
    
    
    

    Panel Panel1;
    Panel Panel2;
    
    //Define Panel Positions
    Panel1.x_center = 4.5;
    Panel1.y_center = 7;
    Panel1.theta = 45; //degree
    Panel1.length = 10;
    Panel1.size = 2;
    
    float distance_between_panels = 4;
    //Find center Position Panel 2
    Panel2.x_center = Panel1.x_center + ((distance_between_panels/2) * 1/cos(Panel1.theta));
    Panel2.y_center = Panel1.y_center - ((distance_between_panels/2) * 1/cos(Panel1.theta));
    Panel2.theta = Panel1.theta;
    Panel2.length = Panel1.length;
    Panel2.size = Panel1.size;
    
    //Find Panel1 Start Point 
    Panel1.P1_x = Panel1.x_center - ((Panel1.length/2)*cos(Panel1.theta));
    Panel1.P1_y = Panel1.y_center - ((Panel1.length/2)*sin(Panel1.theta));
    
    cout<< " Panel1.P1_x:  " <<  Panel1.P1_x << endl;
    cout<< " Panel1.P1_y:  " <<  Panel1.P1_y << endl;
    
    //Find Panel1 End Point 
    Panel1.P2_x = Panel1.x_center + ((Panel1.length/2)*cos(Panel1.theta));
    Panel1.P2_y = Panel1.y_center + ((Panel1.length/2)*sin(Panel1.theta));
    
    
    
    //Find Panel2 Start Point
    Panel2.P1_x = Panel2.x_center + ((Panel2.length/2)*cos(Panel2.theta));
    Panel2.P1_y = Panel2.y_center + ((Panel2.length/2)*sin(Panel2.theta));
    
    //Find Panel2 End Point
    Panel2.P2_x = Panel2.x_center - ((Panel2.length/2)*cos(Panel2.theta));
    Panel2.P2_y = Panel2.y_center - ((Panel2.length/2)*sin(Panel2.theta));
    
    
      
    //Define Gains Controller
    float Kp_z = 1.5;
    float Kd_z = 0.5;
    float Kp_yaw = 0.6;
    float Kd_yaw = 0.2;
    float Kp_x = 0.6;
    float Kp_y = 0.6;
    float Kd_x = 0.3;
    float Kd_y = 0.3;
    float Ki_x = 0.1;
    float Ki_y = 0.1;
    double integralx(0);
    double integraly(0);
    float dt = 0.01;
    float time = 0.0;
    
    float disturb = 0.0;
    //error on the drone body frame in PD controller
    float e_x = 0.0;
    float e_y = 0.0;
    float yaw_des = 0.0;
    float yaw_des_old = 0.0;
    float yaw_err = 0.0;
    float cartesian_distance_err = 0.0;
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

    while(nh.ok()) {
        geometry_msgs::Twist drone_vel_msg;
        
        
        if(Take_off == false) {
            takeOff.publish(myMsg);
            drone.desired_z = 2;
            //Increasing altitude 
            drone_vel_msg.linear.z = Kp_z * (drone.desired_z - drone.drone_z) + Kd_z * (0 - drone.drone_lin_vel_z);
            //Align Yaw drone with solar Panel
            
            yaw_des = atan2(Panel1.P1_y,Panel1.P1_x);
            yaw_err = yaw_des - drone.drone_Yaw;
            drone_vel_msg.angular.z = Kp_yaw * (yaw_err) + Kd_yaw * (0 - drone.drone_ang_vel_z);
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
        disturb = sin(0.1 * time);
        
        // Move Drone towards check points
        switch(checkpoint) {
        case 1: // First Checkpoint panel 1
        
            cartesian_distance_err = sqrt(pow(Panel1.P1_x - drone.drone_x,2) + pow(Panel1.P1_y - drone.drone_y,2));
            //Attitude
            yaw_des = atan2(Panel1.P1_y,Panel1.P1_x); //Yaw drone orientato verso il punto
            yaw_err = yaw_des - drone.drone_Yaw;
            drone_vel_msg.angular.z = Kp_yaw * (yaw_err) + Kd_yaw * (0 - drone.drone_ang_vel_z);
            
            //From ground to body frame
            Rotation_GF_to_BF_des_pos(Panel1.P1_x, Panel1.P1_y, yaw_err);
            Rotation_GF_to_BF_drone_pos(yaw_err);
            
            integralx += (Panel1.P1_x - drone.drone_x) * dt;
            integraly += (Panel1.P1_y - drone.drone_y) * dt;
            
            vel_x = Kp_x * (Panel1.P1_x - drone.drone_x) + Kd_x * (0 - drone.drone_lin_vel_x) + Ki_x *integralx;
            vel_y = Kp_y * (Panel1.P1_y - drone.drone_y) + Kd_y * (0 - drone.drone_lin_vel_y) + Ki_y * integraly;
            
            Rotation_GF_to_BF_drone_vel(vel_x, vel_y, drone.drone_Yaw);
            
            drone_vel_msg.linear.x =   drone.drone_lin_vel_x_b; //Output of Rotation of desired velocity
            drone_vel_msg.linear.y =  drone.drone_lin_vel_y_b + disturb;
            
            cout << "disturb intnsity: " << sin(0.1 * time)<< endl;
            cout << "drone_velocity_PID output: " << drone.drone_lin_vel_y_b<< endl;
            cout << "des velocity: " << drone_vel_msg.linear.y<< endl;
            
            
            cout << "Reaching CheckPoint 1 --> : " << cartesian_distance_err<< endl;
            if (cartesian_distance_err < distance_threshold)
            {
                integralx = 0;
                integraly = 0;
                ros::Duration d(2);
                secs = d.toSec();
                checkpoint = 2;
            }
            
            break;
        case 2: // First Checkpoint panel 1
            cartesian_distance_err = sqrt(pow(Panel1.P2_x - drone.drone_x,2) + pow(Panel1.P2_y - drone.drone_y,2));
            
            //Attitude
            yaw_des = atan2(Panel1.P2_y,Panel1.P2_x);//Yaw drone orientato verso il punto
            
            yaw_err = yaw_des - drone.drone_Yaw;
            drone_vel_msg.angular.z = Kp_yaw * (yaw_err) + Kd_yaw * (0 - drone.drone_ang_vel_z);
            cout << "yaw_des : " << yaw_des<< endl;
            cout << "drone.drone_Yaw: " <<  drone.drone_Yaw << endl;
            
            
            
           if(abs(yaw_err) < 0.2) 
            {
                integralx += (Panel1.P2_x - drone.drone_x) * dt;
                integraly += (Panel1.P2_y - drone.drone_y) * dt;
                
                vel_x = Kp_x * (Panel1.P2_x - drone.drone_x) + Kd_x * (0 - drone.drone_lin_vel_x) +  Ki_x *integralx;
                vel_y = Kp_y * (Panel1.P2_y - drone.drone_y) + Kd_y * (0 - drone.drone_lin_vel_y) +  Ki_y *integraly;
            
                Rotation_GF_to_BF_drone_vel(vel_x, vel_y, drone.drone_Yaw);
                
            
                drone_vel_msg.linear.x =   drone.drone_lin_vel_x_b;
                drone_vel_msg.linear.y =  drone.drone_lin_vel_y_b + disturb;
                cout << "Reaching CheckPoint 2 --> : " << cartesian_distance_err<< endl;
            
            }
            else
            {
                integralx += (Panel1.P1_x - drone.drone_x) * dt;
                integraly += (Panel1.P1_y - drone.drone_y) * dt;
                
                vel_x = Kp_x * (Panel1.P1_x - drone.drone_x) + Kd_x * (0 - drone.drone_lin_vel_x) +  Ki_x *integralx;
                vel_y = Kp_y * (Panel1.P1_y - drone.drone_y) + Kd_y * (0 - drone.drone_lin_vel_y) +  Ki_y *integraly;
             
                Rotation_GF_to_BF_drone_vel(vel_x, vel_y, drone.drone_Yaw);
            
                drone_vel_msg.linear.x =   drone.drone_lin_vel_x_b;
                drone_vel_msg.linear.y =  drone.drone_lin_vel_y_b;
                cout << "Waiting for yaw alignment" << endl;
                
            }
            
            cout << "Reaching CheckPoint 2 --> : " << cartesian_distance_err<< endl;
            
            if (cartesian_distance_err < distance_threshold)
            {
                integralx = 0;
                integraly = 0;
                checkpoint = 3;
            }
            
            yaw_des_old = yaw_des;
            break;
        case 3: // First Checkpoint panel 1
            cartesian_distance_err = sqrt(pow(Panel2.P1_x - drone.drone_x, 2) + pow(Panel2.P1_y - drone.drone_y, 2));
            
            //Attitude
            //yaw_des = atan2(Panel2.P1_y,Panel2.P1_x) - M_PI/2; //Yaw drone orientato verso il punto
            yaw_des = yaw_des_old - M_PI;
            yaw_err = yaw_des - drone.drone_Yaw ;
            drone_vel_msg.angular.z = Kp_yaw * (yaw_err) + Kd_yaw * (0 - drone.drone_ang_vel_z);
            cout << "yaw_des : " << yaw_des<< endl;
            cout << "drone.drone_Yaw: " <<  drone.drone_Yaw << endl;
            
            if(abs(yaw_err) < 0.2) {
                integralx += (Panel2.P1_x - drone.drone_x) * dt;
                integraly += (Panel2.P1_y - drone.drone_y) * dt;
                
                vel_x = Kp_x * (Panel2.P1_x - drone.drone_x) + Kd_x * (0 - drone.drone_lin_vel_x) + Ki_x *integralx;
                vel_y = Kp_y * (Panel2.P1_y - drone.drone_y) + Kd_y * (0 - drone.drone_lin_vel_y)  + Ki_y *integraly;

                Rotation_GF_to_BF_drone_vel(vel_x, vel_y, drone.drone_Yaw);

                drone_vel_msg.linear.x = drone.drone_lin_vel_x_b;
                drone_vel_msg.linear.y = drone.drone_lin_vel_y_b;
                cout << "Reaching CheckPoint 3 --> : " << cartesian_distance_err<< endl;
            }
            else
            {
                integralx += (Panel1.P2_x - drone.drone_x) * dt;
                integraly += (Panel1.P2_y - drone.drone_y) * dt;
                
                vel_x = Kp_x * (Panel1.P2_x - drone.drone_x) + Kd_x * (0 - drone.drone_lin_vel_x) + Ki_x *integralx;
                vel_y = Kp_y * (Panel1.P2_y - drone.drone_y) + Kd_y * (0 - drone.drone_lin_vel_y) + Ki_y *integraly;
             
                Rotation_GF_to_BF_drone_vel(vel_x, vel_y, drone.drone_Yaw);
            
                drone_vel_msg.linear.x =   drone.drone_lin_vel_x_b;
                drone_vel_msg.linear.y =  drone.drone_lin_vel_y_b;
                cout << "Waiting for yaw alignment" << endl;
                
            }
            
            
            
            
            if(cartesian_distance_err < distance_threshold) {
                integralx = 0;
                integraly = 0;
                checkpoint = 4;
            }

            break;
            
        case 4: // First Checkpoint panel 1
            cartesian_distance_err = sqrt(pow(Panel2.P2_x - drone.drone_x, 2) + pow(Panel2.P2_y - drone.drone_y, 2));
            
            //Attitude
            //yaw_des = atan2(Panel2.P2_y,Panel2.P2_x) - M_PI/2; //Yaw drone orientato verso il punto
            yaw_err = yaw_des - drone.drone_Yaw;
            drone_vel_msg.angular.z = Kp_yaw * (yaw_err) + Kd_yaw * (0 - drone.drone_ang_vel_z);
            cout << "yaw_des : " << yaw_des<< endl;
            cout << "drone.drone_Yaw: " <<  drone.drone_Yaw << endl;
            
            if(abs(yaw_err) < 0.2) {
                integralx += (Panel2.P2_x - drone.drone_x) * dt;
                integraly += (Panel2.P2_y - drone.drone_y) * dt;
                
                vel_x = Kp_x * (Panel2.P2_x - drone.drone_x) + Kd_x * (0 - drone.drone_lin_vel_x) + Ki_x *integralx;
                vel_y = Kp_y * (Panel2.P2_y - drone.drone_y) + Kd_y * (0 - drone.drone_lin_vel_y) + Ki_y *integraly;
            
                 Rotation_GF_to_BF_drone_vel(vel_x, vel_y, drone.drone_Yaw);
            
                 drone_vel_msg.linear.x =   drone.drone_lin_vel_x_b;
                 drone_vel_msg.linear.y =  drone.drone_lin_vel_y_b;
                cout << "Reaching CheckPoint 4 --> : " << cartesian_distance_err<< endl;
            }
            else
            {
                integralx += (Panel2.P1_x - drone.drone_x) * dt;
                integraly += (Panel2.P1_y - drone.drone_y) * dt;
                
                vel_x = Kp_x * (Panel2.P1_x - drone.drone_x) + Kd_x * (0 - drone.drone_lin_vel_x) + Ki_x *integralx;;
                vel_y = Kp_y * (Panel2.P1_y - drone.drone_y) + Kd_y * (0 - drone.drone_lin_vel_y) +  Ki_y *integraly;;
             
                Rotation_GF_to_BF_drone_vel(vel_x, vel_y, drone.drone_Yaw);
            
                drone_vel_msg.linear.x =   drone.drone_lin_vel_x_b;
                drone_vel_msg.linear.y =  drone.drone_lin_vel_y_b;
                cout << "Waiting for yaw alignment" << endl;
                
            }
            
            
            
            if(cartesian_distance_err < distance_threshold) {
                checkpoint = 4;
            }

            break;
        }
        
        
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




 
