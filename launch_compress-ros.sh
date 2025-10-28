#!/bin/bash

# ==============================================================================
# GeoScan 点云流程启动脚本 (V4.2 - gnome-terminal & 强力隔离)
# ==============================================================================
#
# 功能: 使用 gnome-terminal 在单个窗口的多个选项卡中，启动完整的点云处理流程。
#       每个选项卡都有绝对隔离的ROS环境，确保运行稳定。
#
# ==============================================================================

echo "正在使用 gnome-terminal (强力隔离模式) 启动点云流程..."

# V4.2 核心改动: 在每个选项卡的命令开头，使用 `unset` 清理掉可能冲突的
#               ROS 环境变量，然后再 source 指定版本的环境。

gnome-terminal \
  --tab --title="1. ROS Bridge" -e "bash --norc -i -c ' \
    echo --- 终端1: ROS1/2 Bridge ---; \
    unset ROS_DISTRO ROS_PACKAGE_PATH; \
    source /opt/ros/foxy/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/bridge_ws/install/setup.bash; \
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics; \
    exec bash'" \
  \
  --tab --title="2. PC Decompressor" -e "bash --norc -i -c ' \
    echo --- 终端2: Point Cloud Decompressor ---; \
    sleep 3; \
    unset ROS_DISTRO ROS_PACKAGE_PATH; \
    source /opt/ros/foxy/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/pointcloud_tools_ros2_ws/install/setup.bash; \
    ros2 launch pointcloud_compressor pointcloud_decompressor.launch.py; \
    exec bash'" \
  \
  --tab --title="3. PC Compressor" -e "bash --norc -i -c ' \
    echo --- 终端3: Point Cloud Compressor ---; \
    sleep 5; \
    unset ROS_DISTRO ROS_PACKAGE_PATH; \
    source /opt/ros/foxy/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/custom_msgs_ros2_ws/install/setup.bash; \
    source ~/GeoScan/GeoScan_Mesh/pointcloud_tools_ros2_ws/install/setup.bash; \
    ros2 launch pointcloud_compressor pointcloud_compressor.launch.py; \
    exec bash'"

# 注意：如果你需要播放 rosbag 来提供数据，可以取消下面这段代码的注释。
#       它会添加第四个选项卡。
#
#  --tab --title="4. Play Bag" -e "bash --norc -i -c ' \
#    echo --- 终端4: 播放ROS Bag ---; \
#    sleep 8; \
#    unset ROS_DISTRO ROS_PACKAGE_PATH; \
#    source /opt/ros/foxy/setup.bash; \
#    ros2 bag play my_ros2_slam_bag.bag; \
#    exec bash'"

echo ""
echo "gnome-terminal 窗口已启动，请检查点云流程的各个选项卡！"
