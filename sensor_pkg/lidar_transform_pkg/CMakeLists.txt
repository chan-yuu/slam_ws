cmake_minimum_required(VERSION 3.0.2)
project(lidar_transform_pkg)

# 添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖 (ROS 1 风格)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  std_msgs
)

# 查找 PCL
find_package(PCL REQUIRED)

# 2. catkin 配置
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES lidar_transform_pkg
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions
  # DEPENDS system_lib
)

# 添加头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 添加 PCL 定义
add_definitions(${PCL_DEFINITIONS})

# ============================================
# 节点 1: 变换节点
# ============================================
add_executable(lidar_transform_node src/lidar_transform_node.cpp)
target_link_libraries(lidar_transform_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ============================================
# 节点 2: 自动标定节点
# ============================================
add_executable(lidar_calibration_node src/lidar_calibration_node.cpp)
target_link_libraries(lidar_calibration_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ============================================
# 安装规则
# ============================================
install(TARGETS 
  lidar_transform_node
  lidar_calibration_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)