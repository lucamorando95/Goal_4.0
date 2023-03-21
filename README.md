# Goal_4.0
This Repository permits to run the same exact code used in a real scenario inside a simulation based environment. 
The packe cvg_sim_gazebo provide all the launch files required to load the simulation environment.
Please remeber that the dynamics of the robot is simulated by the DJI Matrice Simulator, running inside the DJI Assistant Application. 
This simulates the DJI onboard SDK, providing all the topics needed to iteract with the lower control framework. 




This repository contains the Goal 4.0 application, available for the aircraft DJI Matrice 210 V2 and DJI Matrice 300, only if Manifold 2G or Manifold 2C is available.

The material contained in this repository permits to run a complete simulation of the autonomous navigation. 

# Software required:
For what it concern the code running on manifold, please download the material contained in this repositpry [Navigation Code]() and follow the instruction.
The Software required is the same contained in the other repository.

# Install The simulation code on the external PC:
Create a catkin ws: 
 ```console
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/
 $ catkin_make
  ```  
  
* Download this repository in the src folder via the command: 
 ```console
 $ git clone https://github.com/lucamorando95/Goal_4.0.git
  ```  
  
 * Navigate to the root of the created ROS workspace and type:
 ```console
 $ catkin_make
  ```  

# Running the simulator on the external PC:
Navigate to the folder .... via the following commands:


Mount and Execute the automatic script with the following command:



The script execute the following launch files:
* Create the Gazebo Scenario:
* roslaunch cvg_sim_gazebo ardrone_Matrice_testworld.launch

*  Spawn the cameras
* roslaunch box_urdf spawn_camera_model.launch

* Script that control the cameras
* rosrun solar_fligth_control solar_fligth_control_only_camera



# Running the Autonomous driving script stored in the manifold: 
If the manifold is not connected to a monitor, execute the .sh script as described in [Navigation Code]()].

If the Manifold is connected to a monitor, launch the file listed [Navigation Code]()].
