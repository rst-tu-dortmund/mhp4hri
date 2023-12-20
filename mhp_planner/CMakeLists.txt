cmake_minimum_required(VERSION 3.0.2)
project(mhp_planner)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## Release mode
set(CMAKE_BUILD_TYPE Release)

# Optimization, use -O2 if any problems occur
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    mhp_robot
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include extern/eigen3
  LIBRARIES core numerics solvers master ocp systems tasks plants hypergraph controllers
  CATKIN_DEPENDS roscpp mhp_robot
#  DEPENDS system_lib
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

##########
# Core   #
##########

add_library(core STATIC
    src/core/time_series.cpp
    src/core/time.cpp    
    src/core/global.cpp
)

#################
# Controllers   #
#################

add_library(controllers STATIC
    src/controllers/predictive_controller.cpp
)
target_link_libraries(controllers
    core
    systems
    ocp
    solvers
    hypergraph
    ${catkin_LIBRARIES}

)

#################
# Numerics      #
#################
add_library(numerics STATIC
    src/numerics/finite_differences.cpp
)
target_link_libraries(numerics
    core
    systems
    ${catkin_LIBRARIES}
)

#################
# Systems       #
#################
add_library(systems STATIC
    src/systems/system_dynamics_interface.cpp
    src/systems/standard_filters.cpp
)
target_link_libraries(systems  
    core
    numerics
    ${catkin_LIBRARIES}
)

#################
# Plants        #
#################
add_library(plants STATIC
    src/plants/plant_interface.cpp
)
target_link_libraries(plants
    core
    systems
    ${catkin_LIBRARIES}
)

#################
# Master        #
#################
add_library(master STATIC
  src/master/master.cpp
)
target_link_libraries(master
    tasks
    controllers
    core
    plants
    tasks
    ocp
    hypergraph
    solvers
    ${catkin_LIBRARIES}
)

#################
# Hypergraph    #
#################
add_library(hypergraph STATIC
    src/hypergraph/optimization_problem_interface.cpp
    src/hypergraph/graph/hyper_graph_optimization_problem_base.cpp
    src/hypergraph/graph/hyper_graph_optimization_problem_vertex_based.cpp
    src/hypergraph/graph/edge_interface.cpp
    src/hypergraph/graph/edge_set.cpp
    src/hypergraph/graph/vertex_set.cpp
    src/hypergraph/graph/vertex_interface.cpp
    src/hypergraph/graph/hyper_graph.cpp
    )
target_link_libraries(hypergraph
    core
    numerics
    systems
    ocp
    solvers
    ${catkin_LIBRARIES}
)

#################
# OCP           #
#################
add_library(ocp STATIC 
src/ocp/structured_ocp/structured_optimal_control_problem.cpp
src/ocp/structured_ocp/discretization_grids/full_discretization_grid_base.cpp
src/ocp/structured_ocp/discretization_grids/finite_differences_grid.cpp
src/ocp/functions/stage_functions.cpp
src/ocp/functions/stage_preprocessor.cpp
src/ocp/functions/nlp_functions.cpp
src/ocp/functions/final_state_constraints.cpp
)
target_link_libraries(ocp
    core
    hypergraph
    numerics
    systems
    solvers
    ${catkin_LIBRARIES}
)

#################
# Solvers       #
#################
add_library(solvers STATIC
    src/solvers/nlp_solver_ipopt.cpp
    src/solvers/nlp_solver_ipopt_wrapper.cpp
)
target_link_libraries(solvers  
    core
    numerics
    systems
    ocp
    hypergraph
    ${catkin_LIBRARIES}
)

#################
# Tasks       #
#################
add_library(tasks STATIC
    src/tasks/environment.cpp
)
target_link_libraries(tasks
    core
    controllers
    systems
    numerics
    plants  
    ${catkin_LIBRARIES}
)


#################
# Ipopt       #
#################
# Set CMAKE_MODULE_PATH so that FindIPOPT.cmake is found
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

# Find IPOPT
find_package(IPOPT REQUIRED)
if(IPOPT_FOUND)
    message(STATUS "IPOPT found.")
    target_compile_definitions(solvers PUBLIC ${IPOPT_DEFINITIONS} -DIPOPT)
    target_include_directories(solvers PUBLIC ${IPOPT_INCLUDE_DIRS})
    SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${IPOPT_LINK_FLAGS}" )
    target_link_libraries(solvers ${IPOPT_LIBRARIES})
else(IPOPT_FOUND)
    message(STATUS "IPOPT not found. Install IPOPT or a different solver.")
endif(IPOPT_FOUND)


#############
## Install ##
#############

install(DIRECTORY include/ extern/eigen3/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
