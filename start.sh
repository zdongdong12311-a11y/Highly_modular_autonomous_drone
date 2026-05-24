#!/bin/bash
set -e

echo "[start.sh] Starting LiDAR to MAVROS bridge..."
roslaunch lidar_to_mavros lidar_to_mavros.launch &
LIDAR_PID=$!
sleep 5

echo "[start.sh] Starting 3D-to-2D laser conversion..."
roslaunch pointcloud_to_laserscan point_to_scan.launch &
SCAN_PID=$!
sleep 2

echo "[start.sh] Starting Cartographer SLAM..."
roslaunch cartographer_ros livox.launch &
CARTO_PID=$!
sleep 2

echo "[start.sh] Starting move_base navigation..."
roslaunch move_base nav_3dto2d.launch &
MOVE_PID=$!
sleep 2

echo "[start.sh] All nodes launched. PIDs: $LIDAR_PID $SCAN_PID $CARTO_PID $MOVE_PID"
trap "kill $LIDAR_PID $SCAN_PID $CARTO_PID $MOVE_PID 2>/dev/null" EXIT
wait