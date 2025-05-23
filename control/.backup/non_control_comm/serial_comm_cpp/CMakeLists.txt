cmake_minimum_required(VERSION 3.16)
project(serial_comm_cpp)

# FIND_PACKAGE(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
  geometry_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES serial_comm_cpp
	CATKIN_DEPENDS 
	roscpp std_msgs
	geometry_msgs serial
	DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(serial_comm_cpp src/serial_comm_cpp.cpp
)

target_link_libraries(serial_comm_cpp serial
  ${catkin_LIBRARIES}
)
add_dependencies(serial_comm_cpp
  ${catkin_EXPORTED_TARGETS}
)
## Install executables and/or libraries
install(
	TARGETS
	serial_comm_cpp
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