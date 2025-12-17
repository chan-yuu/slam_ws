cmake_minimum_required(VERSION 2.8.3)
project(map_load)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
)

find_package (Eigen3 REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
)


add_executable(mapload src/mapLoader.cpp)
target_link_libraries(mapload ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
