#!/bin/bash

# ==============================================================================
# GeoScan Project - Unified Build Script
# ==============================================================================
#
# 功能:
#   一键清理并编译项目中的所有 ROS1 和 ROS2 工作空间。
#   此脚本严格遵循依赖顺序，特别为 ros1_bridge 创建了正确的
#   混合编译环境，以解决自定义消息的桥接问题。
#
# 使用方法:
#   1. 将此脚本放置在 ~/GeoScan/GeoScan_Mesh/ 目录下。
#   2. 给予执行权限: chmod +x build_all.sh
#   3. 运行: ./build_all.sh
#
# ==============================================================================

# set -e: 如果任何命令失败（返回非零退出码），脚本将立即退出。
# 这可以防止在一个失败的步骤后继续执行，从而导致更混乱的错误。
set -e

echo "=========================================="
echo "==>     STARTING UNIFIED BUILD ALL     <=="
echo "=========================================="

# --- 第零步: 进入主工作目录 ---
cd ~/GeoScan/GeoScan_Mesh/

# --- 第一步: 清理所有旧的编译产物 ---
echo "--> Step 1: Cleaning all previous build artifacts..."
rm -rf */build/ */install/ */log/ */devel/
echo "Clean complete."

# --- 第二步: 按顺序构建 ROS1 环境 ---
echo -e "\n>>> Step 2: Building ROS1 Workspaces in sequence..."

# 加载基础 ROS1 环境
echo "--> Sourcing ROS1 Noetic..."
source /opt/ros/noetic/setup.bash

# 编译 ROS1 自定义消息 (无依赖)
echo "--> Building custom_msgs_ros1_ws..."
cd custom_msgs_ros1_ws/
catkin_make
source devel/setup.bash
cd ..

# 编译 ROS1 JXL 压缩器 (依赖于自定义消息)
echo "--> Building jxl_compress_ros1_ws..."
cd jxl_compress_ros1_ws/
catkin_make
source devel/setup.bash
cd ..

echo ">>> ROS1 Workspaces build successful."

# --- 第三步: 在现有环境上叠加并构建 ROS2 环境 ---
echo -e "\n>>> Step 3: Overlaying and building ROS2 Workspaces..."

# 在已加载 ROS1 环境的终端中，加载基础 ROS2 环境
echo "--> Sourcing ROS2 Foxy (Overlay Mode)..."
source /opt/ros/foxy/setup.bash

# 编译 ROS2 自定义消息
echo "--> Building custom_msgs_ros2_ws..."
cd custom_msgs_ros2_ws/
colcon build --symlink-install
source install/setup.bash
cd ..

# 编译 ROS2 JXL 解压器
echo "--> Building jxl_decompress_ros2_ws..."
cd jxl_decompress_ros2_ws/
colcon build --symlink-install
source install/setup.bash
cd ..

# 编译 ROS2 点云工具
echo "--> Building pointcloud_tools_ros2_ws..."
cd pointcloud_tools_ros2_ws/
colcon build --symlink-install
source install/setup.bash
cd ..

# --- 第四步: 在完美的混合环境中编译 ROS Bridge ---
echo -e "\n>>> Step 4: Building the ros1_bridge in the final mixed environment..."

# 此时，这个脚本的 Shell 环境同时知道所有 ROS1 和 ROS2 的自定义包
echo "--> Building bridge_ws..."
cd bridge_ws/
colcon build --symlink-install
cd ..

echo ""
echo "====================================================="
echo "  SUCCESS! All workspaces have been compiled correctly.  "
echo "  You are now ready to launch your pipelines.       "
echo "====================================================="
