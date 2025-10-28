#!/bin/bash
# FILE: launch_ground_station.sh

# ==============================================================================
# GeoScan GROUND STATION - Launch Script
# ==============================================================================
# 功能: 在一个窗口中，启动所有地面端节点：JXL解压, 点云解压, RViz2。
# ==============================================================================

echo ">>> Launching GROUND STATION (Decompressors + RViz)..."

gnome-terminal \
  --tab --title="1. GROUND: JXL Decompressor" -e "bash --norc -i -c ' \
    echo --- 地面端: JXL图像解压 (ROS2) ---; \
    unset ROS_DISTRO ROS_PACKAGE_PATH; \
    source /opt/ros/foxy/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/jxl_decompress_ros2_ws/install/setup.bash; \
    ros2 run jxl_compression_tools_ros2 jxl_transcoder_node; \
    exec bash'" \
\
  --tab --title="2. GROUND: PC Decompressor" -e "bash --norc -i -c ' \
    echo --- 地面端: 点云解压 (ROS2) ---; \
    sleep 2; \
    unset ROS_DISTRO ROS_PACKAGE_PATH; \
    source /opt/ros/foxy/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/pointcloud_tools_ros2_ws/install/setup.bash; \
    ros2 launch pointcloud_compressor pointcloud_decompressor.launch.py; \
    exec bash'" \
\
  --tab --title="3. GROUND: RViz2" -e "bash --norc -i -c ' \
    echo --- 地面端: 可视化工具 ---; \
    sleep 4; \
    unset ROS_DISTRO ROS_PACKAGE_PATH; \
    source /opt/ros/foxy/setup.bash; \
    rviz2; \
    exec bash'"

echo ""
echo "GROUND STATION system launched in a new gnome-terminal window."
