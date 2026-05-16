#!/bin/bash
roslaunch lidar_to_mavros lidar_to_mavros.launch & sleep 5
roslaunch pointcloud_to_laserscan point_to_scan.launch & sleep 2
roslaunch cartographer_ros livox.launch & sleep 2
roslaunch move_base nav_3dto2d.launch & sleep 2
wait;