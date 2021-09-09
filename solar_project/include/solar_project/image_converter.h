

#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H



// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>

static const std::string OPENCV_WINDOW = "Image window";


class ImageConverter
{

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  
  // image_transport::Publisher  image_pub;
  // image_transport::Publisher  HSV_image_pub;
  // image_transport::Publisher  FT_RGB_image_pub;
  //image_transport::Publisher  cluster_RGB_image_pub;
  image_transport::Publisher  thermo_gray_image_pub;
  image_transport::Publisher  thermo_FT_image_pub;
  image_transport::Publisher  thermo_distance_image_pub;
  image_transport::Publisher  thermo_image_pub;

  // image_transport::Subscriber RGB_elaborated_image;
  // image_transport::Subscriber HSV_image_sub;
  // image_transport::Subscriber FT_RGB_image_sub;
  //image_transport::Subscriber cluster_RGB_image_sub;
  image_transport::Subscriber thermo_gray_image_sub;
  image_transport::Subscriber thermo_FT_image_sub;
  image_transport::Subscriber thermo_distance_image_sub;
  image_transport::Subscriber thermo_elab_imaorated_image_sub;

  




public:
//Subscribe to image_transport 
//Define ImageCoverter class constructor 
  ImageConverter()
    : it_(nh_)
  {
    // image_pub = it_.advertise("OVTA_camera_vision_RGB_output", 1);
    // HSV_image_pub = it_.advertise("OVTA_DEBUG_HSV_image", 1);
    //FT_RGB_image_pub = it_.advertise("OVTA_DEBUG_FIRST_THRESHOLD_image", 1);
    //cluster_RGB_image_pub = it_.advertise("OVTA_DEBUG_CLUSTERING_image", 1);
    //Pub for thermal image debuggung
    thermo_gray_image_pub = it_.advertise("OVTA_DEBUG_THERMAL_GRAY", 1);
    thermo_FT_image_pub = it_.advertise("OVTA_DEBUG_THERMAL_TH", 1);
    thermo_distance_image_pub = it_.advertise("OVTA_DEBUG_THERMAL_DISTANCE", 1);
    thermo_image_pub = it_.advertise("OVTA_DEBUG_camera_vision_thermal_output", 1);


    // RGB_elaborated_image = it_.subscribe("camera_vision_RGB_output", 1, &ImageConverter::RGB_image_callback, this);
    // HSV_image_sub = it_.subscribe("DEBUG_HSV_image", 1, &ImageConverter::HSV_image_callback, this);
    // FT_RGB_image_sub =  it_.subscribe("DEBUG_FIRST_THRESHOLD_image", 1, &ImageConverter::FT_RGB_image_callback, this);
    //cluster_RGB_image_sub =  it_.subscribe("DEBUG_CLUSTERING_image", 1, &ImageConverter::cluster_RGB_image_callback, this);
//Sub for thermal image debuging
    thermo_gray_image_sub =  it_.subscribe("THERMAL_GRAY", 1, &ImageConverter::thermo_gray_image_callback, this);
    thermo_FT_image_sub =  it_.subscribe("THERMAL_TH", 1, &ImageConverter::thermo_FT_image_callback, this);
    thermo_distance_image_sub =  it_.subscribe("THERMAL_DISTANCE", 1, &ImageConverter::thermo_distance_image_callback, this);
    thermo_elab_imaorated_image_sub =  it_.subscribe("camera_vision_output", 1, &ImageConverter::thermo_image_callback, this);
  }

//Define Image COnverter Class Desctructor
  ~ImageConverter()
  {
  
  }
//Republish Image Node --Image Transport compres the data in order to see the image topic on other computer running ROS connnected to the same Network
// void RGB_image_callback(const sensor_msgs::ImageConstPtr& msg)
// {
//   cv_bridge::CvImagePtr cv_RGB_ptr;
//   try
//   {
//      cv_RGB_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//   }
//   catch(cv_bridge::Exception& e) 
//   {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//   }

//    //cv::imshow(OPENCV_WINDOW, cv_RGB_ptr->image);
//    //cv::waitKey(1);
//   //Publish the compressed image over the air
//   image_pub.publish(cv_RGB_ptr->toImageMsg());
  
// }


// void HSV_image_callback(const sensor_msgs::ImageConstPtr& msg)
// {
//   cv_bridge::CvImagePtr cv_HSV_ptr;
//   try
//   {
//      cv_HSV_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//   }
//   catch(cv_bridge::Exception& e) 
//   {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//   }

//    //cv::imshow(OPENCV_WINDOW, cv_RGB_ptr->image);
//    //cv::waitKey(1);
//   //Publish the compressed image over the air
//   HSV_image_pub.publish(cv_HSV_ptr->toImageMsg());
  
// }

// void FT_RGB_image_callback(const sensor_msgs::ImageConstPtr& msg)
// {
//   cv_bridge::CvImagePtr cv_FT_RGB_ptr;
//   try
//   {
//      cv_FT_RGB_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
//   }
//   catch(cv_bridge::Exception& e) 
//   {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//   }

//    //cv::imshow(OPENCV_WINDOW, cv_RGB_ptr->image);
//    //cv::waitKey(1);
//   //Publish the compressed image over the air
//   FT_RGB_image_pub.publish(cv_FT_RGB_ptr->toImageMsg());
  
// }

// void cluster_RGB_image_callback(const sensor_msgs::ImageConstPtr& msg)
// {
//   cv_bridge::CvImagePtr cv_cluster_RGB_ptr;
//   try
//   {
//      cv_cluster_RGB_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
//   }
//   catch(cv_bridge::Exception& e) 
//   {
//     ROS_ERROR("cv_bridge exception: %s", e.what());
//   }

//    //cv::imshow(OPENCV_WINDOW, cv_RGB_ptr->image);
//    //cv::waitKey(1);
//   //Publish the compressed image over the air
//   cluster_RGB_image_pub.publish(cv_cluster_RGB_ptr->toImageMsg());
  
// }


//THERMAL CALLBACK
void thermo_gray_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_thermo_gray_ptr;
  try
  {
     cv_thermo_gray_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch(cv_bridge::Exception& e) 
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

   //cv::imshow(OPENCV_WINDOW, cv_RGB_ptr->image);
   //cv::waitKey(1);
  //Publish the compressed image over the air
  thermo_gray_image_pub.publish(cv_thermo_gray_ptr->toImageMsg());
  
}

//THERMAL CALLBACK
void thermo_FT_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_thermo_FT_ptr;
  try
  {
     cv_thermo_FT_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch(cv_bridge::Exception& e) 
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

   //cv::imshow(OPENCV_WINDOW, cv_RGB_ptr->image);
   //cv::waitKey(1);
  //Publish the compressed image over the air
  thermo_FT_image_pub.publish(cv_thermo_FT_ptr->toImageMsg());
  
}

//THERMAL CALLBACK
void thermo_distance_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_thermo_distance_ptr;
  try
  {
     cv_thermo_distance_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_64FC1);
  }
  catch(cv_bridge::Exception& e) 
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

   //cv::imshow(OPENCV_WINDOW, cv_RGB_ptr->image);
   //cv::waitKey(1);
  //Publish the compressed image over the air
  thermo_distance_image_pub.publish(cv_thermo_distance_ptr->toImageMsg());
  
}



void thermo_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_thermo_image_ptr;
  try
  {
     cv_thermo_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch(cv_bridge::Exception& e) 
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

   //cv::imshow(OPENCV_WINDOW, cv_RGB_ptr->image);
   //cv::waitKey(1);
  //Publish the compressed image over the air
  thermo_image_pub.publish(cv_thermo_image_ptr->toImageMsg());
}

};

#endif