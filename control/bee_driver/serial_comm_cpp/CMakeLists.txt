cmake_minimum_required(VERSION 3.16)
project(serial_comm_cpp)

# FIND_PACKAGE(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

# include_directories(include)
# add_executable( motor_control src/motor_control.cpp )
# target_link_libraries( motor_control serial )





catkin_package(
	INCLUDE_DIRS include
	LIBRARIES motor_control
	CATKIN_DEPENDS 
	roscpp 
	geometry_msgs
	DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(motor_control src/motor_control.cpp
)

target_link_libraries(motor_control serial
  ${catkin_LIBRARIES}
)
add_dependencies(motor_control
  ${catkin_EXPORTED_TARGETS}
)
## Install executables and/or libraries
install(
	TARGETS
	motor_control
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

## Install project namespaced headers
install(DIRECTORY include
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	PATTERN ".svn" EXCLUDE
	)
# install(DIRECTORY config/
# 	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# 	PATTERN ".svn" EXCLUDE
# 	)
