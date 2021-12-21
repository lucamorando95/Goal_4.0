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

 
**Launch script**
The launch of the all scripts is completely managed by the file ...sh. 

* Make sure the Manifold and the Remote PC are connected via Wi-Fi Network.
* Open the folder in Remote PC were the file launcher.sh is contained.
* Open a terminal inside the folder.
* Launch the file typing chmod +x ... .sh  and ./launcher.sh 

Follow the Procedure suggested by the terminal and type yes or no for each selection.
The terminal provide to you a connection with the Manifold and it launch the desired script.

The automatic script laucnh if desired all the scripts that permits to safely fligth the UAV in a real environment.
 
 The Required launch files execute by the autonomatic ...sh script are:

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
 roslaunch solar_project RGB_visual_ros_sim_reordered.launch
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
The User has to type 1 or 2 inside the RGB and Thermal bash (farlo con luca)


!!!!! NOTE 
If you want to use the NN script instead of the parameter one on the thermal images you have to do:
* Type NO when the automatic script suggest to launch the thermal detection node.
* If you commit a mistake and you type YES: 1) Don't insert the password in the new terminal 2) Type Yes when it ask if the script is coorectly launched 

In this way the NN detection script is automatically selected.

If the NN detection script does not start properly:
* Open a terminal manually on the external PC
* Connect to the Manifold via ssh dji@<ipmanifold>
```console
cd ..
cd dji_ws/devel/lib/solar_project
screen -S NN
./make_mask config.json
 ```  
This sequence launch the script for the detection via the NN.

In order to use the detected image is required to launch another script:
```console
cd ..
screen -S NN_det
roslaunch solar_project termo_cluster_NN.launch
```  

!! The image are not visible when the script are launched automatically but are visible if the script are launched manually
 
**HOW To Launch The Script Manually for the Real Fligth**
 1) Launch the SDK Node:
 ```console
ssh dji@<ip_manifold>
screen -S sdk
roslaunch dji_osdk_ros dji_sdk_node.launch 
 ``` 
 
2)Launch the Camera:
 ```console
ssh dji@<ip_manifold>
screen -S camera
rosrun solar_project start_camera_stream
 ```  

3) Launch the Control:
 ```console
ssh dji@<ip_manifold>
screen -S control
roslaunch solar_project solar_fligth_control.launch
 ``` 
4) Launch the RGB Detection:
 ```console
ssh dji@<ip_manifold>
screen -S RGB
roslaunch solar_project RGB_visual_ros_sim_reordered.launch
 ```  
If you want 
5) Launch the Thermal Detection:
 ```console
ssh dji@<ip_manifold>
screen -S thermo
roslaunch solar_project termo_cluster_regression_reorganized.launch
 ``` 

If you want 
5) Launch the NN Thermal Detection:
```console
cd ..
cd dji_ws/devel/lib/solar_project
screen -S NN
./make_mask config.json
 ```  
This sequence launch the script for the detection via the NN.

In order to use the detected image is required to launch another script:
```console
cd ..
screen -S NN_det
roslaunch solar_project termo_cluster_NN.launch
```  
 
6) Launch the Velocity Control
 ```console
ssh dji@<ip_manifold>
screen -S vel
rosrun solar_project select_desired_vel.py
 ``` 

7) Launch the camera Photo Mission.
 ! Every time you launch it ypu have to clear the sd or to check the number of the last photo
 ```console
ssh dji@<ip_manifold>
screen -S photo
roslaunch solar_project camera_photo_mission.launch
 ```  

To stop the execution of one terminal press: cntrl+C

The screen commands avoid the drone to stop during fligth when it disconnects from the JPAntenna.
It is required to launch it only a time:
Example: If you run the script with the procedure described before and the you stop a script with cntrl + C, you can simply relaunch it pressing up_key then start or relaunching it via roslaunch or rosrun.
If ypu close the terminal on your laptopt then, at the next connection, before launching other scripts, to quit from all the active Screen session.
 At this purpose verify on a terminal connected to the drone the active screen session typing:
 ```console
ssh dji@<ip_manifold>
screen -ls 
 ``` 
 Quit From each screen session typing
 ```console
screen -XS <screen session> quit 
 ```  
**Parameters to set before launching the application**

In /src/solar_project/config are present all the configuration files, which defines various functionalities of the software.

1) To Use the Desired Velocity and the single Array mode: 

* In solar_flight_params.yaml (only some parameters are described here, see the comments in the file for additional information):
```
 enable_velocity_navigation_control_flag: true   // Velocity control enabled, if not the predictive control based on carrot chasing is used
 enable_take_off: false    //If true, the take of is autonomous, then the UAV navigates automatically to the first waypoints and it starts the mission.
 desired_waiting_KF_init_it: waiting_iteration   //select the number of iteration required to initialize the KF
 control_altitude_on_heigth_sensor: false  //USE ONLY UNDER 7 METERS OF HEIGTH --> DANGEROUS
 Detection_param_Optimization_enabled: false //If true, setup the code to wait the optimization results before the KF initialization.
 use_only_two_fixed_waypoints_for_a_single_array_inspection: true  //Create a barrier knowing only two waypoints
 simulation_mode: false

```

* In RGB_detection_params.yaml: 
```
Matrice: False //Keep it False
parameters_optimization: False   //If True the script wait the results of the optimization task and the ask teh user to select the desired image typing 1 or 2 on the keyboard 
XT2_FOV: [45, 37] #Field of vuew of the XT2 camera 
focus_image: True
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


2) To Use the Desired Velocity and the Four Array Mode:

* In solar_flight_params.yaml (only some parameters are described here, see the comments in the file for additional information):
```
 enable_velocity_navigation_control_flag: true   // Velocity control enabled, if not the predictive control based on carrot chasing is used
 enable_take_off: false    //If true, the take of is autonomous, then the UAV navigates automatically to the first waypoints and it starts the mission.
 desired_waiting_KF_init_it: waiting_iteration   //select the number of iteration required to initialize the KF
 control_altitude_on_heigth_sensor: false  //USE ONLY UNDER 7 METERS OF HEIGTH --> DANGEROUS
 Detection_param_Optimization_enabled: false //If true, setup the code to wait the optimization results before the KF initialization.
 use_only_two_fixed_waypoints_for_a_single_array_inspection: false  //Create a barrier knowing only two waypoints
 autonomous_mission_with_manual_take_off: true
 

```

* In RGB_detection_params.yaml: 
```
Matrice: False //Keep it False
parameters_optimization: False   //If True the script wait the results of the optimization task and the ask teh user to select the desired image typing 1 or 2 on the keyboard 
XT2_FOV: [45, 37] #Field of vuew of the XT2 camera 
focus_image: True
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



**How to Setup The Parameters for the first fligth:**
For the first fligth of the day could be required to set up the perception parameters inside the config file  RGB_detection_params.yaml and THERMO_detection_params.yaml.
1) Connect the Manifold and the external computer to the JPANTENNA
2) Open a terminal and write: 
 ```console
ssh dji@<ip_manifold>
 ``` 
 The password is dji
4) Launch the ROS master on the manifold 
 ```console
roslaunch dji_osdk_ros dji_sdk_node.launch
 ``` 
6) On the external computer open a terminal and check if it is possible to see all teh topic published by the drone.
```console
rostopic list
 ``` 

7) On the external PC open another terminal and launch the RGB detection node
 ```console
ssh dji@<ip_manifold>
roslaunch solar_project RGB_visual_ros_sim_reordered.launch
 ``` 
8) On the external PC subscribe to the topic /camera_vision_RGB_output visualizing the images 
 ```console
rosrun image_view image_view image:/camera_vision_RGB_output 
 ``` 
or 
 ```console
rosrun image_view image_view image:/OVTA_DEBUG_HSV_image
 ``` 

The first image streaming permits to visualize the green rectangle around the array if correctly detected.
The second image streaming permits to visualize the output of the HSV transormation

NOTE!!!!
Modify the RGB_detection_params.yaml with the HSV value relative to the array area.
The value are visible on the bottom left of the figure created by the command image_view.
Move the mouse on the array area on the image and visualize the values.
B --> H
G --> S
R --> V

A range is required to correctly config the perception. Check the max value in the brighets area in the array image and the min value in the darkest area.

------ Example of Correctly defined Parameters:
5 August:
H_MIN: 90
S_MIN: 80 
V_MIN: 10

H_MAX: 140
S_MAX: 170
V_MAX: 130


29 October:
H_MIN: 0
S_MIN: 10 
V_MIN: 120

H_MAX: 180
S_MAX: 140
V_MAX: 255

12 Novemeber:
H_MIN: 0
S_MIN: 30 
V_MIN: 150

H_MAX: 190
S_MAX: 120
V_MAX: 255







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
* The drone starts its inspection only if the user type the number of the PV array that is going to be inspected. (the Number must be typed in the terminal that managed the solar_fligth_control task).
* The drone stops when: i) the end of the PV array is reached, ii) the user press a key. 

If the shooting task is executed, the user can find a file with the information of every single photo taken in the following directory in the Manifold: 
