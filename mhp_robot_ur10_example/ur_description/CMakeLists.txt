cmake_minimum_required(VERSION 2.8.3)
project(ur_description)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## Release mode
set(CMAKE_BUILD_TYPE Release)

# Optimization, use -O2 if any problems occur
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#file(GLOB_RECURSE xacro_files "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro")

#add_custom_command(
#    COMMAND xacro --inorder ./urdf/ur10_joint_limited_robot.urdf.xacro -o ./urdf/urdf_gen/ur10.urdf
#    COMMAND xacro --inorder ./urdf/ur3_joint_limited_robot.urdf.xacro -o ./urdf/urdf_gen/ur3.urdf
#    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#    DEPENDS ${xacro_files}
#    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/urdf/urdf_gen/ur10.urdf ${CMAKE_CURRENT_SOURCE_DIR}/urdf/urdf_gen/ur3.urdf
#)

#add_custom_target(urdf_autogeneration_target ALL
#    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/urdf_gen/ur10.urdf ${CMAKE_CURRENT_SOURCE_DIR}/urdf/urdf_gen/ur3.urdf
#)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_description
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(ur_description
#   src/${PROJECT_NAME}/ur_description.cpp
# )

## Declare a cpp executable
# add_executable(ur_description_node src/ur_description_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ur_description_node ur_description_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ur_description_node
#   ${catkin_LIBRARIES}
# )

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

#add_executable(static_obstacle_publisher
#  src/obstacle_publisher/obstacle_publisher.cpp
#)

#add_executable(interactive_obstacle_publisher
#  src/obstacle_publisher/interactive_obstacle_publisher.cpp
#)

#add_executable(ur_distance_publisher
#  src/distance_publisher/ur_distance_publisher.cpp
#)

#target_link_libraries(static_obstacle_publisher
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(interactive_obstacle_publisher
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(ur_distance_publisher
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
