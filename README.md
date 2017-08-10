# Grasping demo with Pepper
In this demo Pepper has to track a box, grasp it and deliver it to a human.


## Prerequisites
* ViSP
* Visp_naoqi
* pepper_control

## Installation
* Build visp_naoqi (you will need to build ViSP and pepper_control, if not already in the robot)
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
