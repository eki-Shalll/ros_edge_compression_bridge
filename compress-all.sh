#!/bin/bash

# ==============================================================================
# GeoScan Ultimate Launcher (V6.1 - Cleaned Syntax)
# ==============================================================================
#
# 功能: 一键启动所有流程！
#       此脚本会启动一个共享的 ROS Bridge，然后并行启动 JXL 图像流程和
#       点云处理流程的所有相关节点。所有节点都在一个 gnome-terminal
#       窗口的独立选项卡中运行，方便管理和监控。
#
# ==============================================================================

echo ">>> Launching ALL GeoScan Pipelines (Image + Point Cloud)..."

# 确保 gnome-terminal 命令的所有部分被正确地连接
gnome-terminal \
--tab --title="1. SHARED Bridge" -e "bash --norc -i -c ' \
  echo --- 终端1: 共享的 ROS1/2 Bridge ---; \
  unset ROS_DISTRO ROS_PACKAGE_PATH; \
  source /opt/ros/foxy/setup.bash; \
  source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
  source ~/GeoScan/GeoScan_Mesh/bridge_ws/install/setup.bash; \
  ros2 run ros1_bridge dynamic_bridge --bridge-all-topics; \
  exec bash'" \
\
--tab --title="2. JXL Compressor (ROS1)" -e "bash --norc -i -c ' \
  echo --- 图像流程: ROS1 JXL Compressor ---; \
  sleep 3; \
  unset ROS_DISTRO ROS_PACKAGE_PATH; \
  source /opt/ros/noetic/setup.bash; \
  source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros1_ws/devel/setup.bash; \
  source ~/GeoScan/GeoScan_Mesh/jxl_compress_ros1_ws/devel/setup.bash; \
  roslaunch jxl_compression_tools bridge_compress_only.launch; \
  exec bash'" \
\
--tab --title="3. JXL Decompressor (ROS2)" -e "bash --norc -i -c ' \
  echo --- 图像流程: ROS2 JXL Decompressor ---; \
  sleep 6; \
  unset ROS_DISTRO ROS_PACKAGE_PATH; \
  source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
  source ~/GeoScan/GeoScan_Mesh/jxl_decompress_ros2_ws/install/setup.bash; \
  ros2 run jxl_compression_tools_ros2 jxl_transcoder_node; \
  exec bash'" \
\
--tab --title="4. PC Decompressor (ROS2)" -e "bash --norc -i -c ' \
  echo --- 点云流程: ROS2 Point Cloud Decompressor ---; \
  sleep 5; \
  unset ROS_DISTRO ROS_PACKAGE_PATH; \
  source /opt/ros/foxy/setup.bash; \
  source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
  source ~/GeoScan/GeoScan_Mesh/pointcloud_tools_ros2_ws/install/setup.bash; \
  ros2 launch pointcloud_compressor pointcloud_decompressor.launch.py; \
  exec bash'" \
\
--tab --title="5. PC Compressor (ROS2)" -e "bash --norc -i -c ' \
  echo --- 点云流程: ROS2 Point Cloud Compressor ---; \
  sleep 7; \
  unset ROS_DISTRO ROS_PACKAGE_PATH; \
  source /opt/ros/foxy/setup.bash; \
  source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
  source ~/GeoScan/GeoScan_Mesh/pointcloud_tools_ros2_ws/install/setup.bash; \
  ros2 launch pointcloud_compressor pointcloud_compressor.launch.py; \
  exec bash'" \
\
--tab --title="6. SHARED RViz2" -e "bash --norc -i -c ' \
  echo --- 共享的可视化工具 ---; \
  sleep 10; \
  unset ROS_DISTRO ROS_PACKAGE_PATH; \
  source /opt/ros/foxy/setup.bash; \
  rviz2; \
  exec bash'"

echo ""
echo "All pipelines launched in a single gnome-terminal window. Check all tabs!"
