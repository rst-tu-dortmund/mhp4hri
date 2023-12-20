cmake_minimum_required(VERSION 3.0.2)
project(ur10_planner)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## Release mode
set(CMAKE_BUILD_TYPE Debug)

# Optimization, use -O2 if any problems occur
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Suppress deprecated warnings
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp    
    mhp_robot
    mhp_planner
    ur_utilities
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
 INCLUDE_DIRS include
 LIBRARIES ur10_ocp ur10_tasks ur10_plants
 CATKIN_DEPENDS roscpp mhp_robot mhp_planner ur_utilities
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

###################
# ur10_ocp        #
###################
add_library(ur10_ocp STATIC
src/ocp/ur_stage_cost_corbo.cpp
src/ocp/ur_final_stage_cost_corbo.cpp

src/ocp/costs/ur_quadratic_cost_joint_space_corbo.cpp

src/ocp/ur_final_state_constraint_joint_space_corbo.cpp

src/ocp/potentials/ur_collision_potential_mohri_corbo.cpp

src/ocp/ur_inequality_constraint_corbo.cpp
src/ocp/constraints/ur_acceleration_constraint_corbo.cpp
src/ocp/constraints/ur_collision_constraint_corbo.cpp

src/ocp/ur_stage_preprocessor_corbo.cpp
)
target_link_libraries(ur10_ocp
  ${catkin_LIBRARIES}
 )

###################
# ur10_plants     #
###################
add_library(ur10_plants STATIC 
src/plants/ur_robot_corbo.cpp
)
target_link_libraries(ur10_plants
  ${catkin_LIBRARIES}
)

###################
# ur10_tasks      #
###################
add_library(ur10_tasks STATIC 
src/tasks/ur_task_corbo.cpp
)
target_link_libraries(ur10_tasks
  ${catkin_LIBRARIES}
)

###################
# ur10_planner    #
###################
add_executable(ur_mhp_planner src/ur_mhp_planner_node.cpp)
target_link_libraries(ur_mhp_planner
    ur10_ocp
    ur10_plants
    ur10_tasks
    ${catkin_LIBRARIES}
)


#############
## Install ##
#############


install(TARGETS ur_mhp_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)