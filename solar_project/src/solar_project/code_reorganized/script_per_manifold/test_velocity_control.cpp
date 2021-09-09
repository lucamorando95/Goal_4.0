
/*
 * Codice Ufficiale controllo in velocità drone
 * 
 * Il drone effettua un quadrato utilizzando un controllo PID in velocità
 * Il drone posiziona un frame nel punto di decollo fissandosi le coordinate GPS in quel punto.
 * Man mano che si sposta eseguo calcolo lo spazio percorso eseguendo la differenza tra 
 * la posizione attauale (in lat e lon) e la posizione iniziale.
 * 
 * Questa differenza vine convertita in una distanza in metri dal pu nto (0,0,0), il luogo dove è stato collocato il frame 
 * nel punto di decollo
 * 
 * Sucessivame nte calcolo la differenza xOffsetRemaining  per definire la distanza da percorrere tar la posizione di target neol punto x, 
 * la posizione raggiunta nello spazio tenedo conto della distanza percorsa.
 * 
 * La differenza viene data in input al controllore come comando in velocita sui tre assi del drone.
 * 
 * 
 * 
 * 
 */







//INCLUDE
#include <ros/ros.h>

#include "mission.h"
#include <dji_sdk/dji_sdk_node.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

// DJI SDK includes
#include <dji_osdk_ros/DroneTaskControl.h>
#include <dji_osdk_ros/SDKControlAuthority.h>
#include <dji_osdk_ros/QueryDroneVersion.h>
#include <dji_osdk_ros/SetLocalPosRef.h>


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include <cmath> 

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
//#include <dji_osdk_ros/FlightTaskControl.h>

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

using namespace dji_osdk_ros;


struct Drone {
    float drone_x;
    float drone_y;
    float drone_z;
	
	float drone_vel_x;
	float drone_vel_y;
	float drone_vel_z;
    
	float drone_gps_origin_pos_x;
	float drone_gps_origin_pos_y;
	float drone_gps_current_lat;
	float drone_gps_current_lon;
	
	float Drone_Target_yaw = 0.0;
	float Drone_Target_x = 0.0;
	float Drone_Target_y = 0.0;
	float Drone_Target_z = 0.0;
	
	float cartesian_distance = 0.0;
	
    
};
 



Drone drone;


//Call to library Mission
Mission square_mission;

//CAll to ROSSERVICE 
ros::ServiceClient drone_task_service;
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;
ros::Publisher ctrlVelYawPub;


// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;





//SUBSCRIBER TOPIC
sensor_msgs::NavSatFix current_gps; 
sensor_msgs::NavSatFix  start_gps_location;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;
geometry_msgs::Vector3Stamped drone_velocity;

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  drone_velocity.vector = msg->vector;
  //std::cout<<"drone_velocity: "<<drone_velocity<<std::endl;
  
  drone.drone_vel_x =  drone_velocity.vector.x;
  drone.drone_vel_y =  drone_velocity.vector.y;
  drone.drone_vel_z =  drone_velocity.vector.z;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
  drone.drone_gps_origin_pos_x = current_local_pos.x;
  drone.drone_gps_origin_pos_y = current_local_pos.y;
}

/*
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;
}
*/
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}
 
 
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
 
  current_gps = *msg;
  drone.drone_gps_current_lat = current_gps.latitude;
  drone.drone_gps_current_lon = current_gps.longitude;
}  
  
//GLOBAL VARIABVLES 
bool take_off = false;



//FUNCTION TO OBTAIN DRONE CONTROL 
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

//TAKE OFF LANDING TASK

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

bool set_local_position()
{
  SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}


bool go_home()
{
	DroneTaskControl droneTaskControl;
    droneTaskControl.request.task = DroneTaskControl::Request ::TASK_GOHOME;
	
	drone_task_service.call(droneTaskControl);
    return droneTaskControl.response.result;
	
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}






//Evaluiate GPS OFFSET ---> Calco0lo distanza percorsa da posizione iniziale ad attuale del drone 
//ripsetto al frame mondo in questo caso posizionato nel punto di decollo

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








int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_vel");
  ros::NodeHandle nh;
  
 // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_osdk_ros/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_osdk_ros/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_osdk_ros/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_osdk_ros/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_osdk_ros/local_position", 10, &local_position_callback);
  ros::Subscriber localvelocity = nh.subscribe("dji_osdk_ros/velocity", 10, &velocity_callback);
  
  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_osdk_ros/flight_control_setpoint_ENUposition_yaw", 10);
  ctrlVelYawPub = nh.advertise<sensor_msgs::Joy>("dji_osdk_ros/flight_control_setpoint_ENUvelocity_yawrate", 10);
  set_local_pos_reference    = nh.serviceClient<SetLocalPosRef> ("dji_osdk_ros/set_local_pos_ref");
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_osdk_ros/flight_control_setpoint_generic", 10);
  
  //ROS DJI SDK services
  sdk_ctrl_authority_service = nh.serviceClient<SDKControlAuthority> ("dji_osdk_ros/sdk_control_authority");
  drone_task_service         = nh.serviceClient<DroneTaskControl>("dji_osdk_ros/drone_task_control");
 
    
	//geometry_msgs::Vector3 localOffset;
	
 
 
  //Obtain Control 
  bool obtain_control_result = obtain_control();

  //Set Position as Reference Frame ---> Posiziona frame per GO_HOME
  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }
 
 
  //ARM motors

 //Take OFF 
	if (takeoff() )
	  {
		 ROS_INFO_STREAM("Take OFF Successfull");
		  ros::Duration(3.0).sleep(); 
		  take_off = true; 
	  }
	
	
	
	/*
	double xOffsetRemaining = 0.0;
	double yOffsetRemaining = 0.0;
    double zOffsetRemaining  = 0.0;
	double yawDesiredRad     =0.0;
    double yawThresholdInRad = 0.0;
    double yawInRad          = 0.0;
	float speedFactor         = 2;
    float yawThresholdInDeg   = 2;
     
	
    float xCmd, yCmd, zCmd;
	*/
	
	//Set Target 
	
	
	/*
	 * IMPORTANTE 
	 * Frame fissato con      
	 * 
	 * 
	 * ^ y
	 * |
	 * |
	 * |
	 * |--------> x 
	 * 
	 * 
	 * Se cambio yaw viene fatta comunque la trasformazione per riferisi alle coordinate del frame locale posizionato
	 * 
	 * 
	 * QUindi se dico di muoversi in x: 10 y: 5 ci si sposta sia con yaw di 0 gradi sia con yaw di 40 gradi.
	 * Viene effettuata la rotazione del bodyframe rispetto al frame locale 
	 * 
	 * Quando posiziono un nuovo frame, definsico solo le coordinate lat lon , quindi posiozne x,y,z .. Se voglio spostarmi 
	 * rispetto coordinate drone devo effettuare una rotazione tra il frame locale e la posizione del drone rispetto a quel frame tenendo conto dello yaw.
	 */
	
	int count = 0;
	bool start_mission = true;
	bool go_home_flag = false;
	
	while(nh.ok())
	{
		
		//Necessario attendere qualche istante per sincronizzaree i topic 
	  if (count <  4 and start_mission == true)
	     {
	        square_mission.start_gps_location = current_gps;
        	 square_mission.start_local_position = current_local_pos;
			  
			  //square_mission.setTarget(20, 0, 0, 0);
			 // std::cout <<"current_local_pos: " <<current_local_pos<< std::endl;
			  //Set Target 
			  drone.Drone_Target_x = 0.0; //10
	          drone.Drone_Target_y = 0.0;
	          drone.Drone_Target_z = 40.0;
	          drone.Drone_Target_yaw = 0.0;//atan2(drone.Drone_Target_y,drone.Drone_Target_x); //x su y perche asse y drone è frontale 
	          drone.cartesian_distance = sqrt(pow( drone.Drone_Target_x - 0,2) + pow(drone.Drone_Target_y - 0,2) + pow( drone.Drone_Target_z - 0, 2));
                  square_mission.state = 1;
	           ROS_INFO("##### Initializing GPS and route for mission %d ....", square_mission.state);
			   count = count + 1;
			   ros::Duration(0.2).sleep(); 
			   ros::spinOnce();
			   continue;
		}
		
		start_mission = false;
		//Start Mission
		if (square_mission.state == 0){
                        break;
                }
		
		//Define a square respect the world frame fixed in the take off position 
		switch(square_mission.state)
                {
                case 0:
                      std::cout<<"sono qui"<< std::endl;
                    break;

                 case 1:
                      if(drone.cartesian_distance > 1.5 and go_home_flag == false)
                       {
		 //Procede con la missione 
                            square_mission.step();
                       }
                       else
                       {
         //riposizione frame locale punto di arrivo --> se non posiziono frame fa tutto rispetto al frame del punto di decollo
                         square_mission.start_gps_location = current_gps;
                         square_mission.start_local_position = current_local_pos;
		  
		  
               //Set new Target 
			  drone.Drone_Target_x = 30.0;
	                 drone.Drone_Target_y = 0.0;
	                 drone.Drone_Target_z = 0.0;
	                 drone.Drone_Target_yaw = atan2(drone.Drone_Target_y,drone.Drone_Target_x);
		 	  
		           drone.cartesian_distance = sqrt(pow(drone.Drone_Target_x -current_local_pos.x,2) + pow(drone.Drone_Target_y- current_local_pos.y - 0,2) + pow( drone.Drone_Target_z - current_local_pos.y , 2));

                            square_mission.state = 2;
		 
                           ROS_INFO("##### Start route %d ....", square_mission.state);
		  //ros::Duration(10).sleep(); 
                             }
                        break;
       
	           case 2:
                           if(drone.cartesian_distance > 1.5 and go_home_flag == false)
                          {
	 	 //Procede con la missione 
                            square_mission.step();
                          }
                            else
                           {
                          //riposizione frame locale punto di arrivo --> se non posiziono frame fa tutto rispetto al frame del punto di decollo
                           square_mission.start_gps_location = current_gps;
                            square_mission.start_local_position = current_local_pos;
		  
		  
          //Set new Target 
		       	   drone.Drone_Target_x = 0.0;
	                    drone.Drone_Target_y = -30.0;
	                    drone.Drone_Target_z = 0.0;
	                   drone.Drone_Target_yaw = 0.0; //atan2(abs(drone.Drone_Target_y),drone.Drone_Target_x);
			  
		            drone.cartesian_distance = sqrt(pow(drone.Drone_Target_x -current_local_pos.x,2) + pow(drone.Drone_Target_y- current_local_pos.y - 0,2) + pow( drone.Drone_Target_z - current_local_pos.y , 2));

                           square_mission.state = 3;
		 
                          ROS_INFO("##### Start route %d ....", square_mission.state);
		  //ros::Duration(10).sleep(); 
                            }
                            break;
		
		
                    case 3:
                            if(drone.cartesian_distance > 1.5 and go_home_flag == false)
                           {
                             square_mission.step();
                           }
                            else
                           {
         //riposizione frame locale punto dia rrivo 
         
	 	                square_mission.start_gps_location = current_gps;
                               square_mission.start_local_position = current_local_pos;
         //Set Target 
			      drone.Drone_Target_x = -30.0; // -10
	                       drone.Drone_Target_y = 0.0;
	                       drone.Drone_Target_z = 0.0;
	                        drone.Drone_Target_yaw =0.0; // atan2(drone.Drone_Target_y,drone.Drone_Target_x) + C_PI; //se x< 0 e y > 0
			        drone.cartesian_distance = sqrt(pow(drone.Drone_Target_x -current_local_pos.x,2) + pow(drone.Drone_Target_y- current_local_pos.y - 0,2) + pow( drone.Drone_Target_z - current_local_pos.y , 2));

                            square_mission.state = 5;
                           ROS_INFO("##### Start route %d ....", square_mission.state);
		  // ros::Duration(10).sleep(); 
                     }
                    break;
         

                   case 4:
        if(drone.cartesian_distance > 0.5 and go_home_flag == false)
        {
          square_mission.step();
        }
        else
        {
          //Riposiziono frame local nel punto di arrivo, in questo caso i setpoint sono sempre posizionati rispetto al 
		  //body frwame del drone 
          
		  square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
          //Set Target 
			  drone.Drone_Target_x = 0.0;
	          drone.Drone_Target_y = 30.0;
	          drone.Drone_Target_z = 0.0;
	          drone.Drone_Target_yaw = 0.0;
	     	  drone.cartesian_distance = sqrt(pow(drone.Drone_Target_x -current_local_pos.x,2) + pow(drone.Drone_Target_y- current_local_pos.y - 0,2) + pow( drone.Drone_Target_z - current_local_pos.y , 2));
           square_mission.state = 0;
          ROS_INFO("##### Start route %d ....", square_mission.state);
		  // ros::Duration(10).sleep(); 
        }
        break;
     
     case 5:
        if(drone.cartesian_distance > 1.5 and go_home_flag == false)
        {
         // square_mission.step();
		if (go_home())
	     {
		   ROS_INFO_STREAM("GO_HOME "); //Per GO HOME deve essere almeno a 20 metri 
		   go_home_flag = true;
	     }
        }
        else
        {
          ROS_INFO("##### Mission %d Finished ....", square_mission.state);
          square_mission.state = 0;
        }
        break;
    }
		
	  count = count + 1;
	  ros::Duration(0.1).sleep(); 
         ros::spinOnce();
      
    }
	
	
	  if (land())
	  {
		   ROS_INFO_STREAM("LANDING ");
	  }
	  
  return 0;
}

	
	
	
//MISSION STEP
void Mission::step()
{
    static int info_counter = 0;
  //Definsico un vettore di punti
    geometry_msgs::Vector3     localOffset;

    double xOffsetRemaining = 0.0;
    double yOffsetRemaining = 0.0;
    double zOffsetRemaining  = 0.0;
    double yawDesiredRad     =0.0;
    double yawThresholdInRad = 0.0;
    double yawInRad          = 0.0;
    float speedFactor         = 2;
    float yawThresholdInDeg   = 2;
     
  //Velocity command 
    float xCmd, yCmd, zCmd;

  //calcolo spostamento rispetto al frame locale della poszione del drone  
    localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  //Offset rispetto posizione da raggiungere di Target
  xOffsetRemaining = drone.Drone_Target_x- localOffset.x;
  yOffsetRemaining =  drone.Drone_Target_y - localOffset.y;
  zOffsetRemaining =  drone.Drone_Target_z- localOffset.z;

  //Yaw desired 
  yawDesiredRad     = drone.Drone_Target_yaw;//deg2rad * drone.Drone_Target_yaw;
  yawThresholdInRad = yawThresholdInDeg;
  yawInRad          = toEulerAngle(current_atti).z;

  //Cartesian distance respect target Point
  drone.cartesian_distance = sqrt(pow(xOffsetRemaining,2) + pow(yOffsetRemaining,2) + pow(zOffsetRemaining, 2));
  std::cout<<"Cartesian Distance: " << drone.cartesian_distance << std::endl;
		//Simple Velocity Control 

 ROS_INFO("##### Target Psoition: x: %f, y: %f, z: %f ", drone.Drone_Target_x, drone.Drone_Target_y, drone.Drone_Target_z);

//Calcolo Offset su ogni asse del frame mondo tra la posizone percorsa dal drone e quella che rimane per raggiu ngere il target
  std::cout<<"xOffsetRemaining: " << xOffsetRemaining << std::endl;
  std::cout<<"yOffsetRemaining: " << yOffsetRemaining << std::endl;
  std::cout<<"zOffsetRemaining: " << zOffsetRemaining << std::endl;
  
  if (abs(xOffsetRemaining) >= speedFactor)
   {
      xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
	        //std::cout << "xCmd: "<<xCmd<<std::endl;
	
    }
  else
  {
     xCmd = 0.4*  xOffsetRemaining + 0.1 * ( 1  -  drone.drone_vel_x); //Velocity control only in position 
  }
	
  if (abs(yOffsetRemaining) >= speedFactor)
  {
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
 
  }
  else
  {
    yCmd = 0.4* yOffsetRemaining + 0.1 * ( 1  -  drone.drone_vel_y);
  }
       
  zCmd = 0.4*zOffsetRemaining + 0.1* (0 - drone.drone_vel_z);
   
  
  if (drone.cartesian_distance > 1.5)
  {
		//Send Command To Vel Topic
   sensor_msgs::Joy ctrlVelYaw;
    
    uint8_t flag = (Control::VERTICAL_VELOCITY   |
                Control::HORIZONTAL_VELOCITY |
                Control::YAW_ANGLE            |    //YAWRATE o YAW_ANGLE
                Control::HORIZONTAL_BODY   |     // HORIZONTAL_BODY rispetto frame locale 
											     // HORIZONTAL_GROUND rispetto al body frasme 
                Control::STABLE_ENABLE);
    ctrlVelYaw.axes.push_back(xCmd);
    ctrlVelYaw.axes.push_back(yCmd);
    ctrlVelYaw.axes.push_back(zCmd);
    ctrlVelYaw.axes.push_back(yawDesiredRad);
    ctrlVelYaw.axes.push_back(flag);

    ctrlVelYawPub.publish(ctrlVelYaw);
  
    return;
  
  }
  else
  {
    sensor_msgs::Joy ctrlVelYaw;
   uint8_t flag = (Control::VERTICAL_VELOCITY   |
         Control::HORIZONTAL_VELOCITY |
         Control::YAW_ANGLE            |
         Control::HORIZONTAL_GROUND   |
         Control::STABLE_ENABLE);
   ctrlVelYaw.axes.push_back(0);
   ctrlVelYaw.axes.push_back(0);
   ctrlVelYaw.axes.push_back(0);
   ctrlVelYaw.axes.push_back(0);
   ctrlVelYaw.axes.push_back(0);
         
   ctrlVelYawPub.publish(ctrlVelYaw);
  }

}

	
	
	
	

	





