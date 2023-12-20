cmake_minimum_required(VERSION 2.8.3)
project(ur_gazebo)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## Release mode
set(CMAKE_BUILD_TYPE Release)

# Optimization, use -O2 if any problems occur
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Suppress deprecated warnings
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  trajectory_msgs
  std_msgs
  control_msgs
  actionlib
  controller_manager_msgs
  gazebo_ros_control
  gazebo_msgs
  ur_utilities
  ur_msgs
)

add_compile_options(-std=c++11)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

# Gazebo cxx flags should have all the required C++ flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  # INCLUDE_DIRS 
  # LIBRARIES 
  CATKIN_DEPENDS roscpp gazebo_ros trajectory_msgs std_msgs control_msgs actionlib controller_manager_msgs ros_control ur_utilities gazebo_ros_control gazebo_msgs ur_msgs
  DEPENDS gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
)


#######################
# ur_msgs_translation #
####################### 
add_executable(ur_msgs_translation
    src/ur_msgs_translation/ur_msgs_translation_main.cpp
)

target_link_libraries(ur_msgs_translation
    ${catkin_LIBRARIES}
)

#######################
#gazebo_wrapper       #
#######################
add_executable(gazebo_service_wrapper
    src/gazebo_service_wrapper/gazebo_service_wrapper_main.cpp
)

target_link_libraries(gazebo_service_wrapper
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS ur_msgs_translation
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY controller/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/controller
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
