#ifndef _WAYPOINTS_SOURCE_H_
#define _WAYPOINTS_SOURCE_H_

#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
//#include "Panel.h"
#include "/home/lucamora/catkin_ws/src/solar_panel_project/include/solar_panel_project/waypoints.h"

//#include "Drone.h"

using namespace std;
using namespace Eigen;

class WaypointsImpl   
{
    private:
    

    //Variabili passate da costruttore in class initialization
    float _delta;
    float _eta;
    float _gamma;


    public:
    float GPS1_x = 0.0;
    float GPS1_y = 0.0;
    
    vector <float> _waypoints_x_coo_world_frame;
    vector <float> _waypoints_y_coo_world_frame;

   

    //Costruttore classe implementativa
    WaypointsImpl(float delta, float eta, float gamma );
    

   
   
   ~WaypointsImpl()
  {
  
  }
};

//Assegno il puntatore alla cllase WaypointsImpl quando WayPoints viene istanziata
WayPoints::WayPoints(float delta, float eta, float gamma)
{
    waypoints_implementation = new WaypointsImpl(delta, eta, gamma);
}




//Il distruttore dealloca anche la classe WayPoints
WayPoints::~WayPoints() 
{
    delete waypoints_implementation;
}

/* Implementation */
//Inizializzo nel costruttore SOLO le variabili passate nel costruttore della classe WaypPoints quando chiamata da main
WaypointsImpl::WaypointsImpl(float delta, float eta, float gamma ) :
//Definisco le variabili passate da PID a PIDIMPL
    _delta(delta),
    _eta(eta),
    _gamma(gamma)
    
{
}




//Definisco qui le funzioni che posso chiamare con la classe WayPoints che utilizzano le variabili salavate nella classe WayPointsImpl


#endif