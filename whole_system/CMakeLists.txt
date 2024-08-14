cmake_minimum_required(VERSION 3.0.2)
project(amr_system)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O2 -g)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  tf2
  roomba_500driver_meiji
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES amr_system
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########
## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(global_path_planner src/global_path_planner.cpp)
add_library(local_goal_creator  src/local_goal_creator.cpp)
add_library(local_map_creator   src/local_map_creator.cpp)
add_library(local_path_planner  src/local_path_planner.cpp)
add_library(obstacle_expander   src/obstacle_expander.cpp)
add_library(obstacle_detector   src/obstacle_detector.cpp)
add_library(emcl                src/emcl.cpp)
add_library(odom_model          src/odom_model.cpp)
add_library(particle            src/particle.cpp)
add_library(pose                src/pose.cpp)

## Declare a C++ executable
add_executable(global_path_planner_node src/global_path_planner_node.cpp)
add_executable(local_goal_creator_node  src/local_goal_creator_node.cpp)
add_executable(local_map_creator_node   src/local_map_creator_node.cpp)
add_executable(local_path_planner_node  src/local_path_planner_node.cpp)
add_executable(obstacle_expander_node   src/obstacle_expander_node.cpp)
add_executable(obstacle_detector_node   src/obstacle_detector_node.cpp)
add_executable(localizer_node           src/localizer_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(global_path_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(local_goal_creator_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(local_map_creator_node   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(local_path_planner_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(obstacle_expander_node   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(obstacle_detector_node   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(localizer_node           ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(global_path_planner_node global_path_planner ${catkin_LIBRARIES})
target_link_libraries(local_goal_creator_node  local_goal_creator  ${catkin_LIBRARIES})
target_link_libraries(local_map_creator_node   local_map_creator   ${catkin_LIBRARIES})
target_link_libraries(local_path_planner_node  local_path_planner  ${catkin_LIBRARIES})
target_link_libraries(obstacle_expander_node   obstacle_expander   ${catkin_LIBRARIES})
target_link_libraries(obstacle_detector_node   obstacle_detector   ${catkin_LIBRARIES})
target_link_libraries(localizer_node emcl odom_model particle pose ${catkin_LIBRARIES})
