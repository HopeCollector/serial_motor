cmake_minimum_required(VERSION 3.0.2)
project(serial_motor)

include(GNUInstallDirs)

add_subdirectory(3rdpart)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} include)
add_subdirectory(serial_motor)

install(
  EXPORT ${PROJECT_NAME}-config
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)