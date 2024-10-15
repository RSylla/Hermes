# Project "Hermes" - Autonomus parcel courier for closed areas.
Clevon academy project.

SSH connection: ssh -X hermes@10.0.3.255

Launcher command:
ROS cmd: ros2 launch multi_package_launcher launch_all_packages.launch.py

Cameras - package for camera image, depth, object detection.
ROS cmd: ros2 run cameras cameras_node

Teleop camera stream server - Server for streaming realtime camera feed with objects.
ROS cmd: ros2 run teleop_camera teleop_camera_node
Connection: ffplay rtsp://10.0.3.225:5000/stream  OR vlc rtsp://10.0.225.5000/stream

How to organize Ros2 packages:
https://automaticaddison.com/organizing-files-and-folders-inside-a-ros-2-package/