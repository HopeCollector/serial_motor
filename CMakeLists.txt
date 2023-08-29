cmake_minimum_required(VERSION 3.0.2)
project(serial_motor)

find_package(serial REQUIRED)
add_library(${PROJECT_NAME} STATIC src/RMD.cpp)
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${SERIAL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  PRIVATE ${SERIAL_LIBRARIES})

add_executable(motor_demo src/motor.cpp)
add_dependencies(motor_demo ${PROJECT_NAME})
target_include_directories(motor_demo PRIVATE include)
target_link_libraries(motor_demo PRIVATE ${PROJECT_NAME})


include(GNUInstallDirs)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-config
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)

install(
  EXPORT ${PROJECT_NAME}-config
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)