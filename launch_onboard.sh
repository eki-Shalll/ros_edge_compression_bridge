#!/bin/bash
# FILE: launch_onboard.sh

# ==============================================================================
# GeoScan ONBOARD System - Launch Script
# ==============================================================================
# 功能: 在一个窗口中，启动所有机载端节点：Bridge, JXL压缩, 点云压缩。
# ==============================================================================

echo ">>> Launching ONBOARD System (Bridge + Compressors)..."

gnome-terminal \
  --tab --title="1. ONBOARD: Bridge" -e "bash --norc -i -c ' \
    echo --- 机载端核心: ROS1/2 Bridge ---; \
    unset ROS_DISTRO ROS_PACKAGE_PATH; \
    source /opt/ros/foxy/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/bridge_ws/install/setup.bash; \
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics; \
    exec bash'" \
\
  --tab --title="2. ONBOARD: JXL Compressor" -e "bash --norc -i -c ' \
    echo --- 机载端: JXL图像压缩 (ROS1) ---; \
    sleep 3; \
    unset ROS_DISTRO ROS_PACKAGE_PATH; \
    source /opt/ros/noetic/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros1_ws/devel/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/jxl_compress_ros1_ws/devel/setup.bash; \
    roslaunch jxl_compression_tools bridge_compress_only.launch; \
    exec bash'" \
\
  --tab --title="3. ONBOARD: PC Compressor" -e "bash --norc -i -c ' \
    echo --- 机载端: 点云压缩 (ROS2) ---; \
    sleep 5; \
    unset ROS_DISTRO ROS_PACKAGE_PATH; \
    source /opt/ros/foxy/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/pointcloud_tools_ros2_ws/install/setup.bash; \
    ros2 launch pointcloud_compressor pointcloud_compressor.launch.py; \
    exec bash'"

echo ""
echo "ONBOARD system launched in a new gnome-terminal window."
