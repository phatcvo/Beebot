cmake_minimum_required(VERSION 3.8)
project(recovery_behavior_ros)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  std_srvs
  amr_navigation_utils_ros
)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(recovery_behavior_node src/recovery_behavior.cpp)
target_link_libraries(recovery_behavior_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
