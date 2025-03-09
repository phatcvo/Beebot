cmake_minimum_required(VERSION 2.8.3)
project(remote_control)

FIND_PACKAGE(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  joy
)


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES remote_control
	CATKIN_DEPENDS 
	joy 
	roscpp 
	geometry_msgs
	DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(remote_control
	src/remote_control.cpp
)

target_link_libraries(remote_control
  ${catkin_LIBRARIES}
)
add_dependencies(remote_control
  ${catkin_EXPORTED_TARGETS}
)
## Install executables and/or libraries
install(
	TARGETS
	remote_control
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
install(DIRECTORY config/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
	PATTERN ".svn" EXCLUDE
	)
