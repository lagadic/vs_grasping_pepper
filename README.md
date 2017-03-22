# Grasping demo with Pepper
In this demo Pepper has to track a box and grasp it. Demo under construction.

## Instructions using pepper_sensor_py to get images:

### Launch camera, whycon and hand pose estimation:
 `$ roslaunch pepper_launch hand_pose_camera_bottom.launch`

### Launch demo grasping
`$ roslaunch vs_grasping_pepper pbvs_arm_servo_right.launch`

### Launch MBT for the box
`$ roslaunch pepper_launch mbt_tabasco.launch`

## Instructions using naoqi_driver to get images:

### Publish images from pepper:
`$ roslaunch pepper_launch pepper_full_wifi.launch`
  
### Launch whycon and hand pose estimation:
`$ roslaunch pepper_launch hand_pose_camera_bottom_naoqi_driver.launch`

### Launch MBT for the box
`$ roslaunch pepper_launch mbt_tabasco_naoqi_driver.launch`
