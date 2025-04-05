cmake_minimum_required(VERSION 3.16)
project(epmc_cpp LANGUAGES CXX)



include_directories(include)
add_executable( motor_control examples/motor_control.cpp )
target_link_libraries( motor_control serial )

