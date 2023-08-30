cmake_minimum_required(VERSION 3.0.2)
project(motor) 

find_package(serial_motor REQUIRED)

add_executable(${PROJECT_NAME} motor.cpp)
target_link_libraries(${PROJECT_NAME} serial_motor::serial_motor)

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)