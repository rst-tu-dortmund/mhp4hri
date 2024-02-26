cmake_minimum_required(VERSION 3.0.2)
project(ufomap_bundled)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
## Release mode
set(CMAKE_BUILD_TYPE Release)

# Optimization, use -O2 if any problems occur
add_compile_options(-O3)

# suppress register warnings
add_compile_options(-Wno-register)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  dynamic_reconfigure
  nav_msgs
  rviz
  roscpp
  std_msgs
  std_srvs
  visualization_msgs
  message_generation
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

find_package(CUDA REQUIRED)

add_subdirectory(ufomap)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  DIRECTORY msg
  FILES
    AABB.msg
    BoundingVolume.msg
#  Capsule.msg
#  Cone.msg
#  Cylinder.msg
#  Ellipsoid.msg
    Frustum.msg
    LineSegment.msg
    OBB.msg
    Plane.msg
    Point.msg
    Ray.msg
    Sphere.msg
#  Triangle.msg
    UFOMap.msg
    UFOMapMetaData.msg
    UFOMapStamped.msg
    MsgInfoDist.msg
)

add_service_files(
  FILES
  ClearVolume.srv
	GetMap.srv
	Reset.srv
	SaveMap.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)
set(CMAKE_CUDA_FLAGS ${CMAKE_CUDA_FLAGS} "-g -G")  # enable cuda-gdb
set(CUDA_NVCC_FLAGS "-arch=sm_86 --std=c++17  --expt-relaxed-constexpr" CACHE STRING "nvcc flags" FORCE)
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE) 
set(LIB_TYPE SHARED)

cuda_add_library(
  CUDASampLib ${LIB_TYPE} src/ufomap_mapping/calcInfoGain.cu
  )

target_link_libraries(CUDASampLib   
UFO::Map)
# target_compile_options(CUDASampLib PRIVATE "$<$<AND:$<CONFIG:Debug,RelWithDebInfo>,$<COMPILE_LANGUAGE:CUDA>>:-G>")


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Server.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include extern/eigen3 extern/EigenRand
 LIBRARIES ufomap_rviz_plugins ufomap_msgs ufomap_ros geometry_msgs
 CATKIN_DEPENDS roscpp rviz 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  extern/eigen3
  extern/EigenRand
  ${CUDA_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
) 

QT5_WRAP_CPP(MOC_FILES
  # include/ufomap_rviz_plugins/ufomap_base_display.h
  include/ufomap_bundled/ufomap_rviz_plugins/ufomap_display.h
  OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_LEXICAL_CAST_INCLUDED 
)

add_library(ufomap_rviz_plugins
  # src/ufomap_base_display.cpp
  src/ufomap_rviz_plugins/ufomap_display.cpp
  ${MOC_FILES} 
)

target_link_libraries(ufomap_rviz_plugins
  UFO::Map
  ufomap_msgs  
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES} 
)

add_library(ufomap_msgs
  # src/ufomap_base_display.cpp
  src/ufomap_msgs/conversions.cpp
)

target_link_libraries(ufomap_msgs
  UFO::Map
  ${catkin_LIBRARIES}
)

add_library(ufomap_ros
  # src/ufomap_base_display.cpp
  src/ufomap_ros/conversions.cpp
)

target_link_libraries(ufomap_ros
  UFO::Map
  ${catkin_LIBRARIES}
)

add_executable(ufomap_server_node
  src/ufomap_mapping/server_node.cpp
  src/ufomap_mapping/server.cpp
)
add_dependencies(ufomap_server_node ${UFOMAP_BUNDLED_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ufomap_server_node
  UFO::Map
  ufomap_ros
  ufomap_msgs
  CUDASampLib
  ${catkin_LIBRARIES}
)

add_executable(ufomap_saver_node
  src/ufomap_mapping/ufomap_saver.cpp
)
target_link_libraries(ufomap_saver_node
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############
install(TARGETS ufomap_rviz_plugins ufomap_msgs ufomap_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ufomap_server_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ extern/eigen3/ extern/EigenRand/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


