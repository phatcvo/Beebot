cmake_minimum_required(VERSION 3.0.2)
project(arduino_serial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
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
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  arduino_serial 
  src/arduino_serial.cpp
)
target_link_libraries(
  arduino_serial
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    arduino_serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

