cmake_minimum_required(VERSION 3.0.2)
project(motor) 

find_package(serial_motor REQUIRED)

add_executable(${PROJECT_NAME} motor.cpp)
target_link_libraries(${PROJECT_NAME} serial_motor::serial_motor)