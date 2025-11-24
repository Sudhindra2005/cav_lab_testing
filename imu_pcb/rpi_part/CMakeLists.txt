cmake_minimum_required(VERSION 3.0)
project(udp_receiver)

set(CMAKE_CXX_STANDARD 11)

#target_link_libraries(udp_receiver) 

add_executable(debug_imu src/imu_debug.cpp)
