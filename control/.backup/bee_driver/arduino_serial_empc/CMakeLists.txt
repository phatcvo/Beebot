cmake_minimum_required(VERSION 3.0.2)
project(arduino_serial_empc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  geometry_msgs
  message_generation
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
  rospy std_msgs
  DEPENDS system_lib
  serial
  geometry_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  arduino_serial_empc 
  src/arduino_serial_empc.cpp
)
target_link_libraries(
  arduino_serial_empc
  ${catkin_LIBRARIES}
)

install(
  TARGETS
  arduino_serial_empc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

