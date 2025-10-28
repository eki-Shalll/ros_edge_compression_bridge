cmake_minimum_required(VERSION 3.5)
project(custom_compression_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # 确保找到消息生成器

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ========================================================
# === 核心：指令消息文件并生成接口
# ========================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompressedJpegXL.msg"
  "msg/CompressedPointCloud.msg"
  DEPENDENCIES std_msgs # 声明消息所依赖的其他消息包
)

# ========================================================
# === 包安装和注册：rosidl_default_generators 会自动处理头文件的安装
# ========================================================
ament_package()