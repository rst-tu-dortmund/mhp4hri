cmake_minimum_required(VERSION 3.0.2)
project(mhp_robot)

option(USE_ARMADILLO "Use Armadillo for GMM uncertainty estimation" ON)
option(USE_DKM "Use DKM for GMM initialization (required if you intend to use GMM uncertainty estimation)" ON)
option(USE_SAVITZKY_GOLAY "Use Savitzky-Golay filter for human state estimation" OFF)
option(USE_SPL "Use SPL for human motion prediction" OFF)
option(USE_NATNET "Use NatNet SDK for motion capture system" OFF)

## Compile as C++17
add_compile_options(-std=c++17)

## Release Mode
set(CMAKE_BUILD_TYPE Release)

## Compiler optimization flags
add_compile_options(-O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  gazebo_msgs
  gazebo_ros
  visualization_msgs
  message_generation
  urdf
  tf
  tf_conversions
  interactive_markers
  view_controller_msgs
  rosbag
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
if(${USE_SPL})
  catkin_python_setup()
endif(${USE_SPL})

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  MsgBoundingBox.msg
  MsgObstacle.msg
  MsgObstacleList.msg
  MsgDistances.msg
  MsgHuman.msg
  MsgPlane.msg
  MsgUtilityObject.msg
  MsgRelativeOrientation.msg
  MsgExtrapolatedAngle.msg
  MsgFutureAngles.msg
  MsgUncertaintyState.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  SrvStartSimulation.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include extern/eigen3 extern/natnet/include
  LIBRARIES robot_description robot_kinematic robot_collision robot_obstacle robot_misc robot_trajectory_optimization robot_manipulator robot_dynamic
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs trajectory_msgs gazebo_msgs visualization_msgs message_generation urdf tf tf_conversions interactive_markers message_runtime rosbag 
  DEPENDS gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  extern/eigen3 
  ${catkin_INCLUDE_DIRS}
)

if (${USE_NATNET})
  include_directories(
    extern/natnet/include
  )
endif(${USE_NATNET})

#####################
# robot_description #
#####################

add_library(robot_description STATIC
  src/robot_description/robot_description.cpp
)

target_link_libraries(robot_description
    robot_misc
  ${catkin_LIBRARIES}
)

#####################
# robot_kinematic   #
#####################

# robot_kinematic library
add_library(robot_kinematic STATIC
  src/robot_kinematic/robot_kinematic.cpp
  src/robot_kinematic/robot_inverse_kinematic.cpp
  src/robot_kinematic/inverse_kinematic_algorithm_step.cpp
)
target_link_libraries(robot_kinematic
  robot_description
  robot_misc
  ${catkin_LIBRARIES}
)

#####################
# robot_dynamic     #
#####################

# robot_dynamic library
add_library(robot_dynamic STATIC
  src/robot_dynamic/robot_dynamic.cpp
)
target_link_libraries(robot_dynamic
  robot_description
  robot_misc
  ${catkin_LIBRARIES}
)

#####################
# robot_collision   #
#####################

# robot_collision library
add_library(robot_collision STATIC
  src/robot_collision/robot_collision.cpp
  src/robot_collision/robot_collision_marker_publisher.cpp
)
target_link_libraries(robot_collision
  robot_kinematic
  robot_misc
  robot_description
  ${catkin_LIBRARIES}
)

#####################
# robot_obstacle    #
#####################

# robot_obstacle library
add_library(robot_obstacle STATIC
  src/robot_obstacle/obstacle_list.cpp

  src/robot_obstacle/obstacle_publisher.cpp
  src/robot_obstacle/obstacle_processor.cpp

  src/robot_obstacle/obstacle_pipeline/base_process.cpp
  src/robot_obstacle/obstacle_pipeline/manual_obstacle_simulator_process.cpp
  src/robot_obstacle/obstacle_pipeline/automatic_obstacle_simulator_process.cpp
  src/robot_obstacle/obstacle_pipeline/workspace_filter_process.cpp
  src/robot_obstacle/obstacle_pipeline/load_obstacle_process.cpp
  src/robot_obstacle/obstacle_pipeline/tf_obstacle_process.cpp
  src/robot_obstacle/obstacle_pipeline/state_estimation_process.cpp
  src/robot_obstacle/obstacle_pipeline/state_extrapolation_process.cpp
  src/robot_obstacle/obstacle_pipeline/motion_classification_process.cpp
  src/robot_obstacle/obstacle_pipeline/human_motion_prediction_process.cpp
  src/robot_obstacle/obstacle_pipeline/forecast_uncertainty_process.cpp

  src/robot_obstacle/state_estimators/base_state_estimator_task_space.cpp
  src/robot_obstacle/state_estimators/base_state_estimator_joint_space.cpp
  src/robot_obstacle/state_estimators/kalman_state_estimator_task_space.cpp
  src/robot_obstacle/state_estimators/kalman_state_estimator_joint_space.cpp
  src/robot_obstacle/state_estimators/polynom_state_estimator_task_space.cpp
  src/robot_obstacle/state_estimators/polynom_state_estimator_joint_space.cpp
  src/robot_obstacle/state_estimators/savitzky_golay_state_estimator_joint_space.cpp  
  src/robot_obstacle/state_estimators/savitzky_golay_state_estimator_task_space.cpp

  src/robot_obstacle/obstacle_trajectory_generator.cpp
)
target_link_libraries(robot_obstacle
  robot_collision
  robot_misc
  ${catkin_LIBRARIES}
)
add_dependencies(robot_obstacle mhp_robot_generate_messages_cpp)

#####################
# robot_manipulator #
#####################

# robot_manipulator library
add_library(robot_manipulator STATIC
  src/robot_manipulator/robot_manipulator.cpp
)
target_link_libraries(robot_manipulator
  ${catkin_LIBRARIES}
)

#####################
# robot_mocap       #
#####################

if(${USE_NATNET})
  # robot_mocap library
  add_library(robot_mocap STATIC
    src/robot_mocap/robot_mocap.cpp
  )
  target_link_libraries(robot_mocap
    natnet
    ${catkin_LIBRARIES}
  )
endif(${USE_NATNET})
#####################
# robot_misc        #
#####################

# robot_misc library
add_library(robot_misc STATIC
  src/robot_misc/robot_utility.cpp
  src/robot_misc/common.cpp
  src/robot_misc/state.cpp
  src/robot_misc/state_observer.cpp
  src/robot_misc/human.cpp
  src/robot_misc/planning_delay.cpp
  src/robot_misc/gmm.cpp
)
target_link_libraries(robot_misc
  robot_description
  ${catkin_LIBRARIES}
)
add_dependencies(robot_misc mhp_robot_generate_messages_cpp)

#################################
# robot_trajectory_optimization #
#################################

# robot_trajectory_optimization library
add_library(robot_trajectory_optimization STATIC
  src/robot_trajectory_optimization/robot_stage_preprocessor.cpp

  src/robot_trajectory_optimization/robot_state_constraint.cpp
  src/robot_trajectory_optimization/robot_acceleration_constraint.cpp
  src/robot_trajectory_optimization/robot_collision_constraint.cpp

  src/robot_trajectory_optimization/robot_final_state_constraint.cpp

  src/robot_trajectory_optimization/robot_cost_function.cpp
  src/robot_trajectory_optimization/robot_quadratic_cost_joint_space.cpp

  src/robot_trajectory_optimization/robot_collision_potential.cpp
  src/robot_trajectory_optimization/robot_collision_potential_mohri.cpp

  src/robot_trajectory_optimization/robot_p_controller_joint_space.cpp

)
target_link_libraries(robot_trajectory_optimization
  ${catkin_LIBRARIES}
)
add_dependencies(robot_trajectory_optimization mhp_robot_generate_messages_cpp)

#####################
# NATNET            #
#####################

# add shared lib natnet
if(${USE_NATNET})
  FILE(GLOB_RECURSE NATNET_LIB
      "${CMAKE_CURRENT_SOURCE_DIR}/extern/natnet/lib/libNatNet.so"
  )
  add_library(natnet SHARED IMPORTED)
  set_target_properties(natnet PROPERTIES IMPORTED_LOCATION ${NATNET_LIB})
endif(${USE_NATNET})

#####################
# Executables       #
#####################

# workspace monitor
add_executable(robot_workspace_monitor
  src/robot_tools/robot_workspace_monitor.cpp
)
target_link_libraries(robot_workspace_monitor
  robot_obstacle
  ${catkin_LIBRARIES}
)

if (${USE_NATNET})
  # mocap client
  add_executable(robot_mocap_client
    src/robot_tools/robot_mocap_client.cpp
  )
  target_link_libraries(robot_mocap_client
    robot_mocap
    ${catkin_LIBRARIES}
  )
endif(${USE_NATNET})

# TF bag parser
add_executable(robot_tf_bag_parser
  src/robot_tools/rosbag_play.cpp
)
add_dependencies(robot_tf_bag_parser mhp_robot_generate_messages_cpp)

target_link_libraries(robot_tf_bag_parser
  ${catkin_LIBRARIES}
)


# Check if Armadillo is installed for GMM uncertainty estimation
if(${USE_ARMADILLO})
  message(STATUS "Use ARMADILLO.")
  find_package(Armadillo)
  if(ARMADILLO_FOUND)
      message(STATUS "ARMADILLO found.")
      target_compile_definitions(robot_misc PUBLIC ${ARMADILLO_DEFINITION} -DARMADILLO)
      target_include_directories(robot_misc PUBLIC ${ARMADILLO_INCLUDE_DIRS})
      SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${ARMADILLO_LINK_FLAGS}" )
      target_link_libraries(robot_misc ${ARMADILLO_LIBRARIES})
  else(ARMADILLO_FOUND)
      message(STATUS "ARMADILLO not found. ")
  endif(ARMADILLO_FOUND)
else(${USE_ARMADILLO})
  message(STATUS "Do not use ARMADILLO.")
endif(${USE_ARMADILLO})

# Add DKM to include_directories for GMM initialization
if(${USE_DKM} AND ARMADILLO_FOUND)
  message(STATUS "Use DKM.")
  target_include_directories(robot_misc PUBLIC extern/dkm/include/)
  target_compile_definitions(robot_misc PUBLIC ${DKM_DEFINITIONS} -DDKM)
else(${USE_DKM} AND ARMADILLO_FOUND)
  message(STATUS "Do not use DKM.")
endif(${USE_DKM} AND ARMADILLO_FOUND)

# Add Savitzky-Golay filter
if(${USE_SAVITZKY_GOLAY})
  message(STATUS "Use Savitzky-Golay filter.")
  include_directories(extern/SavitzkyGolayFilter/include)
  add_library(savitzky_golay_filter STATIC
      extern/SavitzkyGolayFilter/src/gram_savitzky_golay.cpp
      extern/SavitzkyGolayFilter/src/spatial_filters.cpp
  )
  target_compile_definitions(robot_obstacle PUBLIC ${savitzky_golay_filter_DEFINITION} -DSG)

  set_target_properties(savitzky_golay_filter PROPERTIES COMPILE_FLAGS "-DGRAM_SAVITZKY_GOLAY_EXPORTS")
  target_include_directories(savitzky_golay_filter PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

  target_link_libraries(robot_obstacle savitzky_golay_filter)

  install(TARGETS savitzky_golay_filter
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
else(${USE_SAVITZKY_GOLAY})
  message(STATUS "Do not use Savitzky-Golay filter.")
endif(${USE_SAVITZKY_GOLAY})

# Add SPL
if(${USE_SPL})
  message(STATUS "Use RNN-SPL Prediction.")
  catkin_install_python(PROGRAMS scripts/tfListener.py scripts/motion_prediction_spl.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else(${USE_SPL})
  message(STATUS "Do not use RNN-SPL Prediction.")
endif(${USE_SPL})


#############
## Install ##
#############

if (${USE_NATNET})
  # install for imported shared lib
  install(FILES ${NATNET_LIB}
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  install(DIRECTORY extern/natnet/include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )
  install(TARGETS robot_mocap_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif (${USE_NATNET})


install(DIRECTORY include/ extern/eigen3/ 
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(TARGETS robot_workspace_monitor robot_tf_bag_parser
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
