# Grasping demo with Pepper

## Instructions:

### Launch camera, whycon and hand pose estimation:
 	`$ roslaunch pepper_launch hand_pose_camera_bottom.launch`

### Launch demo grasping
 	`$ roslaunch vs_grasping_pepper pbvs_arm_servo_right.launch`

### Launch MBT for the box
 	`$ roslaunch pepper_launch mbt_tabasco.launch`

### Launch controll in velocity
	`$ roslaunch visp_naoqi_ros pepper_cmd_vel.launch`
