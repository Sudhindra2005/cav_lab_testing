cmake_minimum_required(VERSION 3.0)
project(udp_receiver)

set(CMAKE_CXX_STANDARD 11)

#target_link_libraries(udp_receiver) 

add_executable(spi_receiver src/spi_control.cpp)


