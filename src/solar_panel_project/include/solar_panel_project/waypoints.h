#ifndef _WAYPOINTS_H_
#define _WAYPOINTS_H_

#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
//#include "Drone.h"


using namespace std;
using namespace Eigen;

class WaypointsImpl;

class WayPoints
{
  private:
  //Pointer to the implementation class
  //Non è accessibile da file esterni (soilo la classe WayPoints) può essere chiamata da main, quindi va nel private 
   WaypointsImpl *waypoints_implementation; //mi riferisco alla classe PIDimpl tramite il puntatore *impl
  public:
  
   //Costruttore 
  WayPoints(float delta, float eta, float gamma );
  
   ~WayPoints();


};


#endif