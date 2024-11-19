# Project "Hermes" - Autonomus parcel courier for closed areas.
Clevon academy project.

SSH connection: ssh -X hermes@10.0.3.255

Launcher command:
ROS cmd: ros2 launch multi_package_launcher launch_all_packages.launch.py

Cameras - package for camera raw and depth image. Publishes images to "image_raw" and "image_depth" topic.
ROS cmd: ros2 run cameras cameras_node

Object detection - Object detection node that publishes detected objects and distance to "object_detection" topic.
ROS cmd: ros2 run object_detection object_detection_node

Live video stream over network: Use ros2 built in "rqt_image_view" package with image transport plugin for fast rtsp compressed image viw.
ROS cmd: ros2 run rqt_image_view rqt_image_view --ros-args --remap _image_transport:=compressed

How to organize Ros2 packages:
https://automaticaddison.com/organizing-files-and-folders-inside-a-ros-2-package/
