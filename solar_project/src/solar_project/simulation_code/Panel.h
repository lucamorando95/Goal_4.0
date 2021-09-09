#ifndef _PANEL_H_
#define _PANEL_H_

#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
//#include "Drone.h"

using namespace std;
using namespace Eigen;


class PanelImpl;

class Panel
{
    private:
    //Puntatore alla classe impleentativa
    PanelImpl *panel_implementation; //mi riferisco alla classe PIDimpl tramite il puntatore *impl
     
    public:
    
    //Costruttore classe Panel
    Panel(Eigen::Vector2f panel_center_W, float theta, float size, float length);
    
    //Dichiarazione funzioni implementate nel file panel.cpp
    void init(float n_panels, float distance_between_panels,int configuration);
    
    //Function that pass variables from class Panel to PanelImpl
    void pass_to_class_GPS_error(float GPS_gamma, float GPS_eta, float GPS_delta);

    //Functions to obtain variables in file main from functions of class PanelImpl
    vector<float> obtain_waypoints_x_coo_gps_frame();
    vector<float> obtain_waypoints_y_coo_gps_frame();
    vector<float> obtain_waypoints_x_coo_world_frame();
    vector<float> obtain_waypoints_y_coo_world_frame();
    vector<float> obtain_xcoo_panel_world_frame();
    vector<float> obtain_ycoo_panel_world_frame();
    float obtain_theta();
    


    
     ~Panel();



};

#endif