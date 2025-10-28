#!/bin/bash
# FILE: build_ground_station.sh

# ==============================================================================
# GeoScan GROUND STATION - Unified Build Script
# ==============================================================================
# 功能: 一键编译所有地面端需要的代码包，主要是解压和可视化相关的。
# ==============================================================================
set -e

echo "=========================================="
echo "==>   STARTING GROUND STATION BUILD    <=="
echo "=========================================="
cd ~/GeoScan/GeoScan_Mesh/

echo "--> Step 1: Cleaning GROUND STATION workspaces..."
rm -rf custom_msgs_ros2_ws/build custom_msgs_ros2_ws/install custom_msgs_ros2_ws/log \
       jxl_decompress_ros2_ws/build jxl_decompress_ros2_ws/install jxl_decompress_ros2_ws/log \
       pointcloud_tools_ros2_ws/build pointcloud_tools_ros2_ws/install pointcloud_tools_ros2_ws/log
echo "Clean complete."

echo -e "\n>>> Step 2: Building GROUND STATION (pure ROS2) workspaces..."
source /opt/ros/foxy/setup.bash

echo "--> Building custom_msgs_ros2_ws..."
cd custom_msgs_ros2_ws/; colcon build; source install/setup.bash; cd ..

echo "--> Building jxl_decompress_ros2_ws..."
cd jxl_decompress_ros2_ws/; colcon build; source install/setup.bash; cd ..

echo "--> Building pointcloud_tools_ros2_ws..."
cd pointcloud_tools_ros2_ws/; colcon build; source install/setup.bash; cd ..

echo "====================================================="
echo "  SUCCESS! GROUND STATION system compiled correctly.   "
echo "====================================================="
