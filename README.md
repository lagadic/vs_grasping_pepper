# Grasping demo with Pepper
In this demo Pepper has to track a box, grasp it and deliver it to a human while avoiding obstacles.


## Prerequisites
* ViSP
* Visp_naoqi
* pepper_control
* [QuadProgpp](https://github.com/liuq/QuadProgpp)

## Installation
* Build visp_naoqi (you will need to build ViSP and pepper_control, if not already in the robot)
* Build and install [QuadProgpp](https://github.com/liuq/QuadProgpp) in `$QUADPROG_HOME`
* Set environment variable $QUADPROG_HOME, pointing to the install folder of QuadProgg
* Clone the repo in your `catkin/src`   
 `$ cd ~/catking_ws/src`   
 `$ git clone https://github.com/lagadic/vs_grasping_pepper.git`   
* Build the package:   
 `$ cd ~/catkin_ws`   
 `$ catkin_make -Dvisp_naoqi_DIR=/path/to/visp_naoqi/build -Dvisp_DIR=/path_to_build_visp `   


## Instructions 

### Launch whycon and hand pose estimation:
`$ roslaunch pepper_launch hand_pose_camera_bottom_naoqi_driver.launch`

### Launch MBT for the box
`$ roslaunch pepper_launch mbt_tabasco_naoqi_driver.launch`

### Launch Demo
`$ roslaunch vs_grasping_pepper pbvs_arm_servo_right.launch`
