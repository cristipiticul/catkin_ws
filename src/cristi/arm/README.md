# README #

Required packages: 

* arm control: MoveIt!, ROS Industrial, dynamixel;
* chessboard detection: marker_pose_detection, aruco, uvc_camera, pal_vision_segmentation.

### What is this repository for? ###

This is a part of a bigger [project](http://projects.busoniu.net/?q=node/75). The collection of packages are used to control a robotic arm, Cyton Gamma 1500 by using a webcam.

### How to use it? ###

ROS commands to control the arm and start detecting the chessboard:

```
#!shell

$ sudo chmod 777 /dev/ttyUSB0
$ roslaunch cyton_controllers controller_manager.launch
$ roslaunch cyton_controllers start_controller.launch
$ rosrun cyton_controllers dynamixel_joint_state_publisher.py
$ roslaunch cyton_camera_moveit_conf move_group.launc
$ rosrun pick_place_demo pick_place_demo
$ rosrun rqt_reconfigure rqt_reconfigure
```

### Arm configuration ###

There are two functional models of the Cyton Gamma 1500. One is with a camera mounted on the wrist_roll link and the other one is without the camera (`cyton_gamma_1500.urdf`).
The `cyton_gamma_1500_camera.urdf` model has the camera. The meshes for the camera box are not available yet, but the collision boxes for the camera box is included in the model. The camera box is composed of two parts: `camera_back` (- connected to the wrist roll with a fixed link) and `camera_front` (- connected to the back part with a fixed link).

### Contact ###
Elod Pall

https://sites.google.com/site/timecontroll/