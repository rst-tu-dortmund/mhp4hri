cmake_minimum_required(VERSION 3.0.2)
project(pcl_filters)


## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## Release mode
set(CMAKE_BUILD_TYPE Debug)

# Optimization, use -O2 if any problems occur
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Suppress deprecated warnings
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  mhp_robot
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES filters 
 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs mhp_robot
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_library(filters STATIC
  src/downsampling_filter.cpp
  src/outlier_filter.cpp
  src/pcl_filter_processor.cpp
)
target_link_libraries(filters 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)


add_executable(filter_node src/filter_node.cpp)
target_link_libraries(filter_node 
  filters
  ${catkin_LIBRARIES}
)

add_executable(rosbag_pcl_play_node src/rosbag_pcl_play.cpp)
target_link_libraries(rosbag_pcl_play_node 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)
#############
## Install ##
#############


install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
) 

install(TARGETS filter_node rosbag_pcl_play_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)