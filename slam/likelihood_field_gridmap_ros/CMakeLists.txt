cmake_minimum_required(VERSION 3.8)
project(likelihood_field_gridmap_ros)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

add_executable(likelihood_field_gridmap_node src/likelihood_field_gridmap.cpp)
target_link_libraries(likelihood_field_gridmap_node ${catkin_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
