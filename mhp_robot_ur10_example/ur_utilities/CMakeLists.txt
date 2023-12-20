cmake_minimum_required(VERSION 2.8.3)
project(ur_utilities)

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
  kdl_parser # tests only
  sensor_msgs
  visualization_msgs
  mhp_robot
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
  INCLUDE_DIRS include
  LIBRARIES ur_kinematic ur_description ur_collision ur_manipulator ur_misc
  CATKIN_DEPENDS roscpp kdl_parser sensor_msgs visualization_msgs mhp_robot
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

####################
# ur_description   #
####################
add_library(ur_description STATIC
  src/ur_description/ur3_description.cpp
  src/ur_description/ur10_description.cpp
  src/ur_description/ur_description.cpp
)
target_link_libraries(ur_description
  ${catkin_LIBRARIES}
)

####################
# ur_kinematic     #
####################
add_library(ur_kinematic STATIC
  src/ur_kinematic/ur_kinematic.cpp
  src/ur_kinematic/ur_inverse_kinematic.cpp
)
target_link_libraries(ur_kinematic
  ur_misc
  ur_description
  ${catkin_LIBRARIES}
)

####################
# ur_collision     #
####################
add_library(ur_collision STATIC
  src/ur_collision/ur_collision.cpp
)
target_link_libraries(ur_collision
  ur_kinematic
  ${catkin_LIBRARIES}
)

####################
# ur_manipulator   #
####################
add_library(ur_manipulator STATIC
  src/ur_manipulator/ur_manipulator.cpp
)
target_link_libraries(ur_manipulator
  ${catkin_LIBRARIES}
)

####################
# ur_misc          #
####################
add_library(ur_misc STATIC
  src/ur_misc/ur_utility.cpp
)
target_link_libraries(ur_misc
  ${catkin_LIBRARIES}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

###########################
# ur_distance_publisher   #
###########################
add_executable(ur_distance_publisher
  src/ur_tools/collision_marker_publisher.cpp
)
target_link_libraries(ur_distance_publisher
  ur_collision
  ur_misc
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS ur_kinematic ur_description ur_collision ur_manipulator ur_misc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS ur_distance_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
