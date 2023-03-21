#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>


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

#include <iostream>
#include <string>


//#include "image_converter.h"
using namespace std;

void read_record()
{
  
    // File pointer
    std::fstream fin;
  
    // Open an existing file
    fin.open("/home/lucamora/Desktop/result.csv", std::ios::in);
  
    // Get the roll number
    // of which the data is required
    int rollnum, roll2, count = 0;
    
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
        while (getline(s, word, ',')) {
            
            // add all the column data
            // of a row to a vector
            row.push_back(word);
	  
	    
        }
        

	for (int ii = 0; ii <  row.size(); ii ++ )
	{
             cout << row[ii] << endl;
	}


        // convert string to integer for comparision
        roll2 = stoi(row[0]);
  
        // // Compare the roll number
        // if (roll2 == rollnum) {
  
        //     // Print the found data
        //     count = 1;
        //     cout << "Details of Roll " << row[0] << " : \n";
        //     cout << "Name: " << row[1] << "\n";
        //     cout << "Maths: " << row[2] << "\n";
        //     cout << "Physics: " << row[3] << "\n";
        //     cout << "Chemistry: " << row[4] << "\n";
        //     cout << "Biology: " << row[5] << "\n";
        //     break;
        // }
    }
    if (count == 0)
        cout << "Record not found\n";
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "Solar_fligth_control_sim");

    ros::NodeHandle nh;
     
    read_record();
      
    ros::Rate r(50);
  
    ros::spinOnce();
        r.sleep();
    
    return 0;
}
