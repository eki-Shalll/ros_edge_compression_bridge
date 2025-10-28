#!/bin/bash
# FILE: build_onboard.sh

# ==============================================================================
# GeoScan ONBOARD System - Unified Build Script
# ==============================================================================
# 功能: 一键编译所有机载端需要的代码包，包括压缩器和至关重要的 ros1_bridge。
# ==============================================================================
set -e

echo "=========================================="
echo "==>     STARTING ONBOARD BUILD         <=="
echo "=========================================="
cd ~/GeoScan/GeoScan_Mesh/

echo "--> Step 1: Cleaning ONBOARD workspaces..."
rm -rf bridge_ws/build bridge_ws/install bridge_ws/log \
       custom_msgs_ros1_ws/build custom_msgs_ros1_ws/devel \
       jxl_compress_ros1_ws/build jxl_compress_ros1_ws/devel \
       custom_msgs_ros2_ws/build custom_msgs_ros2_ws/install custom_msgs_ros2_ws/log \
       pointcloud_tools_ros2_ws/build pointcloud_tools_ros2_ws/install pointcloud_tools_ros2_ws/log
echo "Clean complete."

# --- 构建混合环境以编译 Bridge ---
echo -e "\n>>> Step 2: Building mixed environment for the Bridge..."
echo "--> Sourcing ROS1 Noetic & building ROS1 components..."
source /opt/ros/noetic/setup.bash
cd custom_msgs_ros1_ws/; catkin_make; source devel/setup.bash; cd ..
cd jxl_compress_ros1_ws/; catkin_make; source devel/setup.bash; cd ..

echo "--> Sourcing ROS2 Foxy & building ROS2 dependencies..."
source /opt/ros/foxy/setup.bash
cd custom_msgs_ros2_ws/; colcon build; source install/setup.bash; cd ..
# 注意：点云工具包也需要在机载端编译，因为压缩节点在这里面
cd pointcloud_tools_ros2_ws/; colcon build; source install/setup.bash; cd ..

echo -e "\n>>> Step 3: Building the ros1_bridge in the final mixed environment..."
cd bridge_ws/
colcon build --symlink-install
cd ..

echo "====================================================="
echo "  SUCCESS! ONBOARD system compiled correctly.          "
echo "====================================================="
