#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>


#include "/home/dji/DATA/dji_ws/src/solar_project/include/solar_project/Drone.h"
//#include <dji_sdk/dji_sdk_node.h>

using namespace dji_osdk_ros;





//Setup FPV or Main Camera Stream
bool setup_camera_stream(Drone *drone, char inputChar)
{
    bool output = false;
    switch (inputChar) 
    {
      case 'm':
      {
         output = drone->setup_camera_stream_main_cam();

         if (output == false)
         {
            ROS_ERROR("Impossible to Start Main Camera Stream");  
         }
         break;
       }

      case 'f':
      {
         output = drone->setup_camera_stream_fpv_cam();
         if (output == false)
         {
            ROS_ERROR("Impossible to Start FPV Camera Stream");  
         }
         break;
       }
    }
    return output;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "Start_main_camera_stream");
  ros::NodeHandle nh;
  
  Drone drone;
  
  
  
 //Required to make the sdk able to control the drone 
  bool obtain_control_result = drone.obtain_control();
  
  //Select if streaming from Main or FPV camera --> MAin Camera as Default
  char inputChar =  'm';
   
  if (!setup_camera_stream(&drone, inputChar))
  {
      ROS_ERROR("Impossible to Start FPV or Camera Stream");
  }
  else
  {
      ROS_INFO_STREAM("Camera Start Successfully");
  }

  ros::spin();
  
  return 0;
}
