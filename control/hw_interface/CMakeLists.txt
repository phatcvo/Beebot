cmake_minimum_required(VERSION 3.0.2)
project(hw_interface)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  serial
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hw_interface
  CATKIN_DEPENDS roscpp geometry_msgs serial
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ executable
add_executable(hw_interface src/hw_interface.cpp)

# Link libraries
target_link_libraries(hw_interface
  ${catkin_LIBRARIES}
)
