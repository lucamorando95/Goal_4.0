#ifndef _KF_H_
#define _KF_H_

#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>


using namespace std;
using namespace Eigen;

class KF_Impl;

class KF
{
    private:
    KF_Impl *KF_implementation; 

    public:
    //Costruttore classe Panel
    KF();
    //Eigen::Vector2f target_point_P1, target_point_P2 --> Real states 
    //Eigen::Vector2f obs_points_P1, obs_points_P1 --> observed points required to estimate the line parameters 
    void Kalman_Filter_calculate(float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2x, float obs_points_P2_y );
    void EKF_calculate(float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2_x, float obs_points_P2_y,
                                                                                            float X_drone_GF, float Y_drone_GF, float Yaw_drone_GF);  

    void pass_to_class_initialization_waiting_value(int des_init_value);                                                                                       
    void pass_to_class_initialization_matrices(Eigen::Matrix2f Px, Eigen::Matrix2f R);
    void pass_to_KF_class_OBS_in_GF(float a,float c);
    void Kalman_filter_initialization( float GPS_P1_x, float GPS_P1_y, float GPS_P2_x, float GPS_P2_y);
    void Obtain_OBS_Jacobian_Matrix(float X_GF, float Y_GF, float Yaw, float a_GF, float c_GF, float gamma);
    void obtain_estimated_observations(float X_GF, float Y_GF, float Yaw);

    //Output --> estimated line paramters in world frame
    Eigen::Vector2f Obtain_Kalman_filter_estimated_state();
    Eigen::Vector2f Obtain_Kalman_filter_observation(); 
    Eigen::Vector2f Obtain_Kalman_filter_GPS_state();
    Eigen::Vector2f Obtain_Kalman_filter_estimated_observation();
    bool Obtain_KF_initialization_flag();

    
    ~KF();
};

#endif