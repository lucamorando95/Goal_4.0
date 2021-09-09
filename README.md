# Goal_4.0
PV monitoring via autonomous UAV

This repository contains the Goal 4.0 application, available for the aircraft DJI Matrice 210 V2 and DJI Matrice 300, only if Manifold 2G or Manifold 2C is available.

In this repository it is described how to install and to run the available code on the Manifold 2 to use *only* in real test. 
The use of this software in simulation is described in this repository [USE in Simulation]().

 
# How to Install the Software 
* Navigate to [DJI Official page](https://developer.dji.com/onboard-sdk/documentation/introduction/how-to-use-OSDK.html) and istall the DJI OSDK and the DJI ROS OSDK.
* Install ROS kinetic if required from [Install ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* In the catkin workspace where the DJI ROS OSDK is installed, download this repo via the command 
* Create a catkin workspace following the instruction in [Create ROS Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
* Download this repository in the src folder via the command: 
 ```console
 $ git clone https://github.com/lucamorando95/Goal_4.0.git
  ```   
 
* Navigate to the root of the created ROS workspace and type:
 ```console
 $ catkin_make
  ```  
 
 # How to Launch the Software for Real Fligth
 **Setup phase**
 * To launch remotely the script an additional PC is required, connected to the Manifold via ssh.
 * Add these lines on the Manifold *.bashrc*:
 ```console
 export ROS_IP=ip_manifold
 export ROS_HOSTNAME=ip_manifold
 export ROS_MASTER_URI=http://ip_manifold:11311/
 ```  
* Define the IP of the manifold as loopback address typing the command:
 ```console
mettere comando su quaderno rosso
 ```  

* On the external PC modyfing the file *.bashrc* as follow:
 ```console
 export ROS_IP=ip_external_PC
 export ROS_HOSTNAME=ip_external_PC
 export ROS_MASTER_URI=http://ip_manifold:11311/
 ```  

* Turn on the UAV in a safe area and connect the external PC to the manifold via ssh. In each terminal open a screen session with the command:
 ```console
 screen -S name_of_the_session
 ```
 check all the screen available command in [screen](https://linuxize.com/post/how-to-use-linux-screen/)
 
 Start a screen session permits to the aircraft to continue to fly in autonomous way also if the connection is lost. 
 
 
**Launch script**

On the external PC:
* Run the DJI OSDK ROS: 
 ```console
 roslaunch dji_osdk_ros dji_osdk_node.launch
 ```  

* Run the flight control script and the keyboard record script:
```console
 roslaunch solar_project solar_flight_control.launch 
 ```   
* Start the main camera stream:
```console
 rosrun  solar_project start_camera_stream
 ```   
 * Start the RGB acquisition image node:
 ```console
 roslaunch solar_project RGB_visual_ros_sim_reordered
 ```   
* Start the Thermo acquisition image node:
 ```console
 roslaunch solar_project termo_cluster_regression_reorganized
 ```   
* If the velocity control is used: 
 ```console
 rosrun solar_project select_desired_velocity
 ```   
* Start the camera photo shooting task:
```console
 rosrun solar_project take_overlapped_photos
 ```   
* Start the Optimization task (if desired):
```console
 rosrun solar_project parameters_optimization_task_matrice
 ```   
* If the optimization task is launched on manifold, launch the image optimized visualizer script on the external PC:
```console
rosrun solar_panel_project optimization_images_visualizer
``` 
Once the optimization is completed the script required the user intervention to select the desired image. 


**Parameters to set before launching the application**

In /src/solar_project/config are present all the configuration files, which defines various functionalities of the software.
* In solar_flight_params.yaml (only some parameters are described here, see the comments in the file for additional information):
```
 enable_velocity_navigation_control_flag: true   // Velocity control enabled, if not the predictive control based on carrot chasing is used
 enable_take_off: false    //If true, the take of is autonomous, then the UAV navigates automatically to the first waypoints and it starts the mission.
 desired_waiting_KF_init_it: waiting_iteration   //select the number of iteration required to initialize the KF
 control_altitude_on_heigth_sensor: false  //USE ONLY UNDER 7 METERS OF HEIGTH --> DANGEROUS
 Detection_param_Optimization_enabled: true //If true, setup the code to wait the optimization results before the KF initialization.
 use_only_two_fixed_waypoints_for_a_single_array_inspection: true  //Create a barrier knowing only two waypoints
 

```

* In RGB_detection_params.yaml: 
```
Matrice: False //Keep it False
parameters_optimization: False   //If True the script wait the results of the optimization task and the ask teh user to select the desired image typing 1 or 2 on the keyboard 
XT2_FOV: [45, 37] #Field of vuew of the XT2 camera 
//HSV parameters to be set, reading related paper
H_MIN: 90 
S_MIN: 30
V_MIN: 10 

H_MAX: 255
S_MAX: 180
V_MAX: 220 

simulation_mode_enabled: False #True if the Matrice is connected to DJI Assistant 

```
* In THERMO_detection_params.yaml: 
```
// Parameters to be set by the user --> read paper
th_2: 8 #6         #Distance Matrix Ra
th_3: 5000        #Minimum Area Value
th_4: 0.05        #ApproxPolyDp Value

parameters_optimization_enabled: False //If True the script wait the results of the optimization task and the ask teh user to select the desired image typing 1 or 2 on the keyboard 
```

* Camera_mission_params.yaml:
```
//XT2 field of view
XT2_FOV_angle_1: 45
XT2_FOV_angle_2: 37

// Desired Overla between photo
Overlap: 80

Mission_enabled: true //If the task is active 
```



**General description of the Aircraft behaviour during a flight:**
1) Complete autonomous fligth:
The first and the last waypoints of each inspected PV array must be upload to the software expressed in latitude and longitude.
(spiegare come caricarli)

* The drone take off in a complete autonomous way. It reach the desired altitude before starting the navigation to the first waypoint.
* The drone Initialize the KF and it starts the navigation along the PV array.
* The drone change automatically the PV array once a time the end waypoint is reached.

2) Autonomous Navigation: 
Two waypoints separated by a distance equal to the length of a single PV array must be uploaded to the software.

* The drone is manually piloted towards the start position on the PV array to be inspected.
* When the Aircraft is well aligned and the PV array correctly detected, the user has to press a key (the number of the PV array to be inspected) and then to press Enter. 
* The drone takes the control, it start the optimization process (if selected), it initialize the EKF, then it start the navigation along the PV array at the desired heigth and velocity. 
* The velocity could be changed online, typing the float value in the terminal wehere the script select_desired_velocity.py is running.
* When the drone has navigated for a distance equal to the distance between the two given waypoints, it start to hover in place and release the control to the user. 
* The shooting task is automatic managed by the UAV taking into account the heigth, the overlap and the drone velocity.
* The pitch and the yaw of the camera is automatically managed by the UAV.
