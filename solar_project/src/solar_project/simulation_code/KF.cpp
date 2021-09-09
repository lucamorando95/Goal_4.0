#ifndef _KF_SOURCE_
#define _KF_SOURCE_

#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "/home/lucamora/catkin_ws/src/solar_panel_project/include/solar_panel_project/KF.h"

using namespace std;
using namespace Eigen;

class KF_Impl
{
    private:
    int K_count = 0;
    Eigen::Vector2f xh; //GPS line parameters initial state
    Eigen::Matrix2f _Px;
    Eigen::Matrix2f _R;
    Eigen::Vector2f X;
    Eigen::Vector2f X_old;
    Eigen::Vector2f y;
    Eigen::Matrix2f  A;
    Eigen::Matrix2f  C;
    Eigen::Matrix2f  K;
    Eigen::Matrix2f  Px_;
    Eigen::Vector2f  xh_old;
    Eigen::Vector2f yh_;
    Eigen::Vector2f obs;
   
    //Jacobian H EKF
    Eigen::Matrix2f H_J;
    Eigen::Vector2f Z_pred_BF;
    
    Eigen::Vector2f eigen_Px;
    //Estimation Vector 
    Eigen::Vector2f xh_;
    float stdev;
    vector<float> c_obs_v;
    
    public:
 
    //Class constructor, prende i termini dati in input alla class PID
    KF_Impl();
    
    void Kalman_Filter_calculate(float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2_x, float obs_points_P2_y  );
    void EKF_calculate(float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2_x, float obs_points_P2_y,
                                                                                            float X_drone_GF, float Y_drone_GF, float Yaw_drone_GF);  
    void pass_to_class_initialization_matrices( Eigen::Matrix2f Px, Eigen::Matrix2f R);
    void Kalman_filter_initialization( float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float GPS_P1_x, float GPS_P1_y, float GPS_P2_x, float GPS_P2_y); 
    
    void Obtain_OBS_Jacobian_Matrix(float X_GF, float Y_GF, float Yaw, float a_GF, float c_GF, float gamma);
    void obtain_estimated_observations(float X_GF, float Y_GF, float Yaw);
    Eigen::Vector2f Obtain_Kalman_filter_estimated_state();
    Eigen::Vector2f Obtain_Kalman_filter_observation();
    Eigen::Vector2f Obtain_Kalman_filter_estimated_observation();
    
    //Destructor
     ~KF_Impl()
    {

    }
    
};

//Class PanelImpl call from header
KF::KF()
{
    KF_implementation = new KF_Impl();
}

//Function dcall
void KF::Kalman_Filter_calculate(float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2_x, float obs_points_P2_y )
{
    return KF_implementation -> Kalman_Filter_calculate(target_point_P1_x,   target_point_P1_y,  target_point_P2_x,   target_point_P2_y,  obs_points_P1_x,  obs_points_P1_y , obs_points_P2_x,  obs_points_P2_y  );
}


void KF::EKF_calculate(float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, 
                                float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2_x, float obs_points_P2_y,
                                float X_drone_GF, float Y_drone_GF, float Yaw_drone_GF)
{
     return KF_implementation -> EKF_calculate(target_point_P1_x,   target_point_P1_y,  target_point_P2_x,   target_point_P2_y,  obs_points_P1_x,  obs_points_P1_y , obs_points_P2_x,  obs_points_P2_y , X_drone_GF, Y_drone_GF, Yaw_drone_GF);
}

void KF::obtain_estimated_observations(float X_GF, float Y_GF, float Yaw)
{
    return KF_implementation -> obtain_estimated_observations(X_GF, Y_GF, Yaw);
}


void KF::Obtain_OBS_Jacobian_Matrix(float X_GF, float Y_GF, float Yaw, float a_GF, float c_GF, float gamma)
{
    return KF_implementation -> Obtain_OBS_Jacobian_Matrix(X_GF,  Y_GF, Yaw, a_GF, c_GF, gamma);
}

void KF::pass_to_class_initialization_matrices(Eigen::Matrix2f Px, Eigen::Matrix2f R)
{
    return KF_implementation -> pass_to_class_initialization_matrices(Px, R);
}

void KF::Kalman_filter_initialization(float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float GPS_P1_x, float GPS_P1_y, float GPS_P2_x, float GPS_P2_y)
{
    return KF_implementation -> Kalman_filter_initialization(target_point_P1_x, target_point_P1_y, target_point_P2_x,  target_point_P2_y, GPS_P1_x, GPS_P1_y, GPS_P2_x, GPS_P2_y);
}


/* ------------------> TO MAIN */

Eigen::Vector2f KF::Obtain_Kalman_filter_estimated_state()
{
    return KF_implementation ->Obtain_Kalman_filter_estimated_state();
}

Eigen::Vector2f KF::Obtain_Kalman_filter_observation()
{
     return KF_implementation ->Obtain_Kalman_filter_observation();
}

Eigen::Vector2f KF::Obtain_Kalman_filter_estimated_observation()
{
     return KF_implementation ->Obtain_Kalman_filter_estimated_observation();
}



//Il distruttore dealloca anche la classe Panel
KF::~KF() 
{
    delete KF_implementation;
}

/* Implementation */

KF_Impl::KF_Impl():
//Definisco TUTTE le variabiliche verranno utilizzate nelle funzioni e definite in PanelImpl
//Le variabili passate con costruttore devono essere rinominate e passate al file cpp definendole in private
K_count(),
c_obs_v(),
stdev(),
xh_(),
X(),
y(),
A(),
C(),
K(),
_Px(),
_R(),
Px_(),
xh_old(),
xh(),
X_old(),
yh_(),
eigen_Px(),
obs(),
H_J(),
Z_pred_BF()

{
}

void KF_Impl::Kalman_Filter_calculate(float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2_x, float obs_points_P2_y )
{    
    //Evaluate a,b,c equation parameters given the the target points in GFs
       
    
      float a_target = (target_point_P2_y - target_point_P1_y)/(target_point_P2_x - target_point_P1_x);
      float b_target = 1;
      float c_target = ((-1*a_target * target_point_P1_x) + target_point_P1_y);
      
       
    //   obs_points_P2_x = 1.0;
    //   obs_points_P1_x = 0.0;
      //Evaluate a,b,c equation parameters given the the obs points in GF
      float a_obs = (obs_points_P2_y -obs_points_P1_y)/(obs_points_P2_x -obs_points_P1_x);
      float b_obs = 1;
      float c_obs = ((-1*a_obs * obs_points_P1_x) + obs_points_P1_y);

      cout<< "[KALMAN FILTER] a_target : "<< a_target << "     c_target: "<< c_target << endl;
      cout<< "[KALMAN FILTER] OBSERVATIONS:  a_obs : "<< a_obs <<"    c_obs: "<< c_obs << endl;
      

      //Define sitution when the KF is not updated
      if (isnan(a_obs) == 1 || isnan(c_obs) == 1)
      {
          return;
      }

      if (K_count < 10)
     {
        if (c_obs < xh_[1] - 3.5 || c_obs > xh_[1] + 3.5)
           {
             return;
           }

     } 
     else
      {
          if (a_obs < xh_[0] - 0.2 || a_obs >  xh_[0] + 0.2)
          {
             //Eliminate very wrong predictions
             return;
           }
           
            if (c_obs < xh_[1] - 2 || c_obs > xh_[1] + 2)
           {
              
               cout<< "KF.xh_[1]: " <<xh_[1]<<endl;
               return;
           }
      }
      //Evaluate mean and covariance of c_obs
      c_obs_v.push_back(c_obs);
      float sum = accumulate(c_obs_v.begin(), c_obs_v.end(), 0.0); //SOmma i valori nel vettore partendo da zero
      float mean = sum / c_obs_v.size();
      vector<double> diff(c_obs_v.size());
      transform(c_obs_v.begin(), c_obs_v.end(), diff.begin(),
               bind2nd(std::minus<double>(), mean));
      double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
      stdev = sqrt(sq_sum / c_obs_v.size());
       
      //cout<< "[KALMAN FILTER]  mean: "<< mean << endl;
      //cout<< " [KALMAN FILTER]  stdev: "<< stdev << endl;
      if (abs(c_obs) - abs(xh_[1]) >  2.5* stdev and K_count > 40)
      {
          cout<<"SONO IN RETURN PER STD"<<endl;
            return;
      }
     //Define Matrices for aritmetic evaluations 
      Eigen::Matrix2f res;
      Eigen::Matrix2f res1;
      Eigen::Matrix2f I;
      Eigen::Vector2f inov(0.0,0.0);

      I << 1.0 ,0.0,
      0.0, 1.0;
      
      //Update state iteration
      X << X_old[0], X_old[1]; 
      y << a_obs, c_obs;

    //Update state estimation and estimation covariance matrix with the previous values 
     xh_ =  xh;
     
     Px_ = A * _Px * A;

//------------------- MEASUREMENT UPDATE: 
//---------Kalamn Filter update K(t) = P(t|t-1) * C' * inv(C*P(t|t-1) * C' + R)
       res = Px_ * C;
      
       res1 = C * Px_* C + _R;
      
       res1 = res1.inverse().eval();
      
       K = res * res1;
//------------  Estimated  observation y(t|t-1) = C*X(t|t-1)
       yh_ = C * xh_;
// ----------- Measuremet residual innovation error y(t) - y(t|t-1)

       inov = y.transpose() - yh_.transpose();
       
//------------ A posteriori updated estimate of the current state x(t|t
       xh = xh_ + K * inov;
    

//------------ A posteriori updated state covariance matrix P(t|t) = (I - K(t)*C)
     
     _Px = (I - K* C) * Px_;

  //cout << "[KALMAN FILTER] Estimated state xh_:" << xh_ << endl; 

     Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver(_Px);
     if (eigensolver.info() != Eigen::Success) abort();
     Eigen::Vector2f eigenvalues1 =  eigensolver.eigenvalues();
    
     eigen_Px = eigenvalues1; 
    // cout<<" Eigen values of KF RGB Px: "<< eigenvalues1<< endl;
     
    //Insert observation inside vector obs to return to main 
     obs << a_obs, c_obs;
     
     K_count = K_count + 1;
     xh_old =  xh;      

}


float find_gamma(float Yaw, float a_GF)
{
  //Find the angle gamma relative to the orientation of the drone 
  //Yaw respect the orientation of the state GPS Line
  float alfa_GF = atan(a_GF);
  float gamma = 0;
  if (Yaw > alfa_GF - M_PI/2 && Yaw < alfa_GF + M_PI/2)
  {
      gamma = -Yaw;
  }
  else if ( Yaw > alfa_GF + M_PI/2 && Yaw < M_PI)
  {
      gamma = M_PI - Yaw;
  }
  else
  {
      gamma = -M_PI - Yaw;
  }

   return gamma;
}


void KF_Impl::Obtain_OBS_Jacobian_Matrix(float X_GF, float Y_GF, float Yaw, float a_GF, float c_GF, float gamma)
{
    /*
    Evaluate the Jacobian matrix relatively to the OBS matrix in order to linearize the non linear observation matrix 
    h(X,t) dove X è lo stato X = [a_GF, c_GF] da stimare 
    */
    float h11 = (1/pow(cos(Yaw),2))/pow(1 + a_GF*tan(Yaw),2);
    float h12 = 0;
    float h21_1 = (X_GF * cos(Yaw))/(pow(cos(Yaw) + sin(Yaw)*a_GF,2));
    float h21_2 = - (c_GF*sin(Yaw))/(pow(cos(Yaw) + sin(Yaw)*a_GF,2));
    float h21_3 = (Y_GF * sin(Yaw))/(pow(cos(Yaw) + sin(Yaw)*a_GF,2));

    float h21 = h21_1 + h21_2 + h21_3;

    float h22 = 1/(cos(Yaw) + sin(Yaw)*a_GF);

  
    H_J << h11, h12,
    h21, h22;

    Z_pred_BF = H_J*X;


}

void KF_Impl:: obtain_estimated_observations(float X_GF, float Y_GF, float Yaw)
{
    /*
    EValuiate the estimated observation appliyng the non linear function Z = h(xh_,t)  è lo stato stimato xh_
    */
    
    //cout <<" _-----> X_GF: " << X_GF << " _-----> Y_GF: " << Y_GF << endl;
    float a_GF_est = xh_[0];
    float c_GF_est = xh_[1];

    float alfa_GF = atan(a_GF_est);
    //Estimated observations in BF
    float a_BF_est = 0.0;
    float c_BF_est = 0.0;
    float gamma = 0.0;

    if (Yaw > alfa_GF - M_PI/2 && Yaw < alfa_GF + M_PI/2)
    {
        a_BF_est = tan(alfa_GF -Yaw);
    }
    else if ( Yaw > alfa_GF + M_PI/2 && Yaw < M_PI)
    {
        gamma = M_PI - Yaw;
        a_BF_est = tan(gamma +alfa_GF);
       
    }
    else
    {
        gamma = -M_PI - Yaw;
        a_BF_est = tan(gamma +alfa_GF);
    }
    
    float num = (1/cos(Yaw))*(X_GF *tan(alfa_GF) + c_GF_est - Y_GF);
    float den = 1 + (tan(Yaw)*tan(alfa_GF));

    c_BF_est = num/den;
    yh_ << a_BF_est, c_BF_est;

    

}

void KF_Impl::EKF_calculate(float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, 
                                float obs_points_P1_x, float obs_points_P1_y ,float obs_points_P2_x, float obs_points_P2_y,
                                float X_drone_GF, float Y_drone_GF, float Yaw_drone_GF)
{
    //EValuate GPS state
    float a_target = (target_point_P2_y - target_point_P1_y)/(target_point_P2_x - target_point_P1_x);
    float b_target = 1;
    float c_target = ((-1*a_target * target_point_P1_x) + target_point_P1_y);
      
       
  
    //Evaluate a,b,c equation parameters given the the obs points in BF
    float a_obs_BF = (obs_points_P2_y -obs_points_P1_y)/(obs_points_P2_x -obs_points_P1_x);
    float b_obs = 1;
    float c_obs_BF = ((-1*a_obs_BF * obs_points_P1_x) + obs_points_P1_y);


    //Mettere se necessari i check sulle osservazioni

   //Define sitution when the KF is not updated
    if (isnan(a_obs_BF) == 1 || isnan(c_obs_BF) == 1)
    {
        return;
    }

    //FInd the associated angle relative to the orientation Yaw of the drone respect the orientation of the line
   float gamma = find_gamma(Yaw_drone_GF, a_target);

   //Estimate the H jacobian Observation Matrix
   Obtain_OBS_Jacobian_Matrix(X_drone_GF, Y_drone_GF, Yaw_drone_GF, a_target, c_target, gamma);
   
    //Define Matrices for aritmetic evaluations 
      Eigen::Matrix2f res;
      Eigen::Matrix2f res1;
      Eigen::Matrix2f I;
      Eigen::Vector2f inov(0.0,0.0);

      I << 1.0 ,0.0,
      0.0, 1.0;
      
      //Update state iteration
      X << X_old[0], X_old[1]; 
      y << a_obs_BF, c_obs_BF;

      //Update state estimation and estimation covariance matrix with the previous values 
     xh_ =  A* xh;
    
     Px_ = A * _Px * A.transpose();

     //------------------- MEASUREMENT UPDATE: 
     //---------Kalamn Filter update K(t) = P(t|t-1) * C' * inv(C*P(t|t-1) * C' + R)
     res = Px_ * H_J.transpose();
     res1 = H_J * Px_* H_J.transpose() + _R;
    
     res1 = res1.inverse().eval();
    
     K = res * res1;
//------------  Estimated  observation y(t|t-1) = C*X(t|t-1)
//Obtain the estimated observation applying the non linear function yh_ = h(xh_,t) given as input the estimated state xh_
//-----> yh_ = h(xh_, t)
       obtain_estimated_observations(X_drone_GF, Y_drone_GF, Yaw_drone_GF);
    // cout<<"----> yh_: " << yh_ << endl;
    
// ----------- Measuremet residual innovation error y(t) - y(t|t-1)

       inov = y.transpose() - yh_.transpose();
       
//------------ A posteriori updated estimate of the current state x(t|t
       xh = xh_ + K * inov;
    

//------------ A posteriori updated state covariance matrix P(t|t) = (I - K(t)*C)
     
     _Px = (I - K* H_J) * Px_;

     Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver(_Px);
     if (eigensolver.info() != Eigen::Success) abort();
     Eigen::Vector2f eigenvalues1 =  eigensolver.eigenvalues();
    
     eigen_Px = eigenvalues1; 
    // cout<<" Eigen values of KF RGB Px: "<< eigenvalues1<< endl;
     
    //Insert observation inside vector obs to return to main 
     obs << a_obs_BF, c_obs_BF;
     
     K_count = K_count + 1;
     xh_old =  xh;      
     
     cout<< "[KALMAN GAIN] K: " << K << endl;
     
}



//Funzione necessaria per inizializzare le matrici e sopratutto dare un valore iniziale allo stato da stimare 
//relativo alla posizione della vela. 
//Chiamare la funzione al momento del passaggio da una vela a quella sucessiva 
void KF_Impl::Kalman_filter_initialization( float target_point_P1_x,  float target_point_P1_y, float target_point_P2_x,  float target_point_P2_y, float GPS_P1_x, float GPS_P1_y, float GPS_P2_x, float GPS_P2_y) 
{      
      
    
      float a_target = (target_point_P2_y -target_point_P1_y)/(target_point_P2_x -target_point_P1_x);
      float b_target = 1;
      float c_target = ((-1*a_target * target_point_P1_x) + target_point_P1_y);

      
      
      //Evalaute line equation from GPS waypoints for initialization for each panels line
      float a_GPS = (GPS_P2_y -GPS_P1_y)/(GPS_P2_x - GPS_P1_x);
      float b_GPS = 1;
      float c_GPS = ((-1*a_GPS * GPS_P1_x) + GPS_P1_y);

    //Initilize state and matrices of the Kalman each time a new panel array is started 
      X << a_target, c_target; 
      X_old << a_target, c_target;
      //y << a_obs, c_obs;
      a_GPS = a_GPS + 0.01;
      c_GPS = c_GPS - 0.4;
      xh << a_GPS, c_GPS; 
      
      cout << "[KF INIT] X: " << X << endl;
      cout << "[KF INIT] xh: " << xh << endl;

      A << 1.0, 0.0,
          0.0,1.0;
      C << 1.0, 0.0,
          0.0,1.0;
      K << 0.0, 0.0,
          0.0, 0.0;

      Px_ << 0.0, 0.0,
          0.0, 0.0;
      
      xh_old << 0.0, 0.0;

      xh_ <<  xh[0], xh[1];
      

      K_count = 0;

      cout << "[KALMAN FILTER] State Initialization with GPS line parameters: " << xh << endl;
}






//Obtain Initialization Matrices  from MAin 
//Inizializzazione delle matrici di covarianza
//Chiamare la funzione subito dopo aver inizializzato la calsse 
void KF_Impl::pass_to_class_initialization_matrices(Eigen::Matrix2f Px, Eigen::Matrix2f R)
{
     
     _Px = Px;
     _R = R;


     
}

// Kalman FIlter Output
Eigen::Vector2f KF_Impl::Obtain_Kalman_filter_estimated_state()
{
    return xh_;
}

 Eigen::Vector2f KF_Impl::Obtain_Kalman_filter_observation()
 {
     return obs;
 }

Eigen::Vector2f KF_Impl::Obtain_Kalman_filter_estimated_observation()
{
    return yh_;
}
#endif