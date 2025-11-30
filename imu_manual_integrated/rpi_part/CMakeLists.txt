cmake_minimum_required(VERSION 3.0)
project(udp_receiver)

set(CMAKE_CXX_STANDARD 11)

add_executable(udp_receiver src/motor_control.cpp)
#target_link_libraries(udp_receiver) 

add_executable(spi_receiver src/spi_control.cpp)

add_executable(imu_receiver src/imu_control.cpp)

add_executable(yaw_receiver src/yaw_control.cpp)

add_executable(debug_imu src/imu_debug.cpp)

add_executable(pcb_imu src/imu_pcb.cpp)

add_executable(fused_receiver src/fused_control.cpp)

