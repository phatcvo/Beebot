cmake_minimum_required(VERSION 3.0.2)
project(pca9685_servo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pca9685_servo src/pca9685_servo.cpp)

target_link_libraries(pca9685_servo
  ${catkin_LIBRARIES}
  pthread
  rt
)
