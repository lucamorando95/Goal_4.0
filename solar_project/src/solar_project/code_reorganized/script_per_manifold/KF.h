
#ifndef _KF_H_
#define _KF_H_

#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include "Drone.h"

using namespace std;
using namespace Eigen;

class KalmanFilter
{

     private:
       //Observation Vector
    vector <float> a_obs_v;
    vector <float> c_obs_v;
    
    //State Matrix
    Eigen::Matrix2d A;
    
    //Observation matrix
    Eigen::Matrix2d C;
    
    //Estimated Observation
    Eigen::Vector2d yh_;
    
    //Real value X_old  
    Eigen::Vector2d X_old ;
   
    Eigen::Vector2d xh_old;
   
  
    //Kalman Gain 
    Eigen::Matrix2d K;
    
   //Eigen values vector of Px matrix
    Eigen::Vector2d  eigen_Px;
   
   //Obs std
    double stdev = 0.0;
    
    //a, c parameters estimated in body frame
    float a_b = 0.0;
    float c_b = 0.0;
  
    //Last Position when the KF was Active 
    float quad_y_old = 0;
    int switch_point = 1;
    //Kalman count
    int K_count = 0;
    
    //Serve per sapere se lo stato del filtro termico viene aggiornato, quindi se sono diposnibili osservazioni veritiere.
    //Lo stato se aggiornato puo essere utilizzato dal filtro in cascata.
    bool thermal_update = false;
    bool initialization;
    
    //Initialize Kalman Filter 
    int State_dim = 2;
    int Obs_dim = 2;

    //Instantiate a link to class Drone
    Drone drone_kf;

    //Variables related to matrix rotation 
    float check_x_w = 0.0;
    float check_y_w = 0.0;
    
    public:
    
    //Definisco delle variabili di traslazione rispetto al frame fissatto artificialmnete in kaboratorio:
    //Pongo frame fisso in pavimento  calcolo traslazione camera rispetto frame fissato. translation_x e translation_y devono essere calcolate a mano col righello
    float translation_x = 0.0;
    float translation_y = 0.0;
    float yaw = 0.0; //fissare come variabile definita a priori nei test in laboratorio
    

     //Import control points from RGB or thermal camera expressed in body frame in the same variables
    float control_point1_x_b = 0.0;
    float control_point1_y_b = 0.0;
    float control_point2_x_b = 0.0;
    float control_point2_y_b = 0.0;




    //Parametri retta target 
    float a_target = 0.0;
    float c_target = 0.0;
    //parametri retta osservata
    float a_obs = 0.0;
    float c_obs = 0.0;
  
  //################## KALMAN FILTER VARIABLES  #################
  //Da inizializare in main .cpp
  // State Vector 
   
    Eigen::Vector2d X ; //Nello scenario reale questi valori non sono conosciuti
  //Observation vector 
    Eigen::Vector2d y; //aggiornata nel codice sotto
   //A posteriori (updated) estimate of the current state 
    Eigen::Vector2d xh; //-----> inizializzare con i parametri della retta passanti per i primi due waypoints
     
  //A posteriori (updated) state Covariance matrix
    Eigen::Matrix2d Px; //KF.Px << 4.0, 0.0, .0, 4.0;
   //COvariace Noise measurement matrix
    Eigen::Matrix2d R;
  //A priori Estimated State
     Eigen::Vector2d xh_;
   //A priori estiumate of the state covariance matrix
    Eigen::Matrix2d Px_; 

   
    //Class constructor
    KalmanFilter(){
      //Initialization of matrices 
       A << 1.0, 0.0,
          0.0,1.0; 
    
       C << 1.0, 0.0,
          0.0,1.0;
    
       K << 0.0, 0.0,
          0.0, 0.0;
     }
    
    
    
    
    
    //Destructor
    ~KalmanFilter(){

    }
    
    void Rotation_BF_to_GF_des_pos(float x_pos, float y_pos, float alfa)
    {
    check_x_w = x_pos * cos(alfa) - y_pos * sin(alfa) + translation_x; //drone_kf.drone_x;
    check_y_w = x_pos * sin(alfa) + y_pos * cos(alfa) + translation_y; //drone_kf.drone_y;
    }


   void Kalman_Filter(float x_target_P1, float y_target_P1, float x_target_P2, float y_target_P2, int target_point)
  {
   
   
    //Primo Kalman FIlter nella sequenza 
     float x1_obs = 0.0;
     float y1_obs = 0.0;
     float x2_obs = 0.0;
     float y2_obs = 0.0;

     
   
    //Ruoto Punti termo nel GF 
    
      Rotation_BF_to_GF_des_pos(drone_kf.control_RGB_point1_x, drone_kf.control_RGB_point1_y, yaw); //drone_kf.drone_Yaw); //---RIsultato finisce in check_x_w e check_y_w variabili globali
      x1_obs = check_x_w;
      y1_obs = check_y_w;
     
      
      Rotation_BF_to_GF_des_pos(drone_kf.control_RGB_point2_x, drone_kf.control_RGB_point2_y, yaw); //drone_kf.drone_Yaw); 
      x2_obs = check_x_w;
      y2_obs = check_y_w;
     
     //########## DEBUGGING ##############
       a_obs = (drone_kf.control_RGB_point2_y -  drone_kf.control_RGB_point1_y)/(drone_kf.control_RGB_point2_x - drone_kf.control_RGB_point1_x);
      float b_obs = 1;
      c_obs = ((-1*a_obs * drone_kf.control_RGB_point1_x) +  drone_kf.control_RGB_point1_y);
      cout<< "[KALMAN FILTER RGB] a_obs^B: "<< a_obs <<"    c_obs^B: "<< c_obs << endl;
      //############################à

      //Evaluate a,b,c equation parameters given the the target points in GFs
      a_target = (y_target_P2 - y_target_P1)/(x_target_P2 - x_target_P1);
      float b_target = 1;
      c_target = ((-1*a_target * x_target_P1) + y_target_P1);
      
      //Evaluate a,b,c equation parameters given the the obs points in GF
      a_obs = (y2_obs - y1_obs)/(x2_obs - x1_obs);
      b_obs = 1;
      c_obs = ((-1*a_obs * x1_obs) + y1_obs);
    
      
      //Initilize real state (only known in simulation)
      X << a_target, c_target; 
      X_old = X;
      
      y << a_obs, c_obs;

      //Define sitution when the KF is not updated
      if (isnan(a_obs) == 1 or isnan(c_obs) == 1)
      {
          return;
      }
      
       //Check per riportare il xh al valore reale una volta che diverge 
      
      if (K_count < 10)
      {
      
        if (c_obs < xh_[1] - 3.5 or c_obs > xh_[1] + 3.5)
         {
            return;
         }
      }
      else
      {
        if (a_obs < xh_[0] - 3.2 or a_obs >  xh_[0] + 3.2)
          {
            //Eliminate very wrong predictions
            return;
          }
           
          if (c_obs < xh_[1] - 2 or c_obs > xh_[1] + 2)
          {
            cout<< "KF_RGB.xh_[1]: " <<xh_[1]<<endl;
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
       
      
   
      if (abs(c_obs) - abs(xh_[1]) >  2.5* stdev and K_count > 40)
      {
          cout<<"SONO IN RETURN PER STD"<<endl;
          
            return;
      }
      

      //Define Matrices for aritmetic evaluations 
      Eigen::Matrix2d res;
      Eigen::Matrix2d res1;
      Eigen::Matrix2d I;
      Eigen::Vector2d inov(0.0,0.0);
      
      I << 1.0 ,0.0,
      0.0, 1.0;
    

     //Estimated State and Covariance matrix update ---> In main faccio update prima dell RGB poi del thermo in sequenza chiamando due istanze della classe KF
      xh_ <<  xh[0], xh[1];
      Px_ = A * Px * A;

      //------------------- MEASUREMENT UPDATE: 
      //---------Kalamn Filter update K(t) = P(t|t-1) * C' * inv(C*P(t|t-1) * C' + R)
       res = Px_ * C;
       res1 = C * Px_* C + R;
       res1 = res1.inverse().eval();
       K = res * res1;
       //------------  Estimated  observation y(t|t-1) = C*X(t|t-1)
       yh_ = C * xh_;
       // ----------- Measuremet residual innovation error y(t) - y(t|t-1)
       inov = y.transpose() - yh_.transpose();
       //------------ A posteriori updated estimate of the current state x(t|t) = x(t|t-1) + K(t)*((y(t) - y(t|t-1))
       xh = xh_ + K * inov;
      //------------ A posteriori updated state covariance matrix P(t|t) = (I - K(t)*C) * P(t|t-1))
       Px = (I - K* C) * Px_;

       Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(Px);
       if (eigensolver.info() != Eigen::Success) abort();
       Eigen::Vector2d eigenvalues1 =  eigensolver.eigenvalues();
       eigen_Px = eigenvalues1; 
       
       K_count = K_count + 1;
       //quad_y_old = drone.drone_y; --> Nel main 
       xh_old =  xh;

    }

    
};

#endif
